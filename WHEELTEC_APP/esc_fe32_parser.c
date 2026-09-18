#include "esc_fe32_parser.h"

#include <string.h>

static const uint8_t kEscFe32Prefix[ESC_FE32_PREFIX_LEN] = {
    0xFEU, 0x01U, 0x00U, 0x03U, 0x30U
};

static uint16_t esc_fe32_read_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

uint16_t EscFe32_Crc16Modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;

    if (data == NULL && length != 0U)
    {
        return 0U;
    }

    for (index = 0U; index < length; ++index)
    {
        uint8_t bit;

        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

void EscFe32Parser_Init(EscFe32Parser_t *parser)
{
    if (parser == NULL)
    {
        return;
    }

    memset(parser, 0, sizeof(*parser));
    parser->next_sample_id = 1U;
}

static void esc_fe32_drop_buffer_prefix(EscFe32Parser_t *parser, size_t count)
{
    if (count >= parser->buffer_len)
    {
        parser->discarded_bytes += (uint32_t)parser->buffer_len;
        parser->buffer_len = 0U;
        return;
    }

    memmove(parser->buffer, &parser->buffer[count], parser->buffer_len - count);
    parser->buffer_len -= count;
    parser->discarded_bytes += (uint32_t)count;
}

static void esc_fe32_resync(EscFe32Parser_t *parser)
{
    for (;;)
    {
        size_t index;
        size_t limit;
        uint8_t prefix_ok = 1U;

        if (parser->buffer_len == 0U)
        {
            return;
        }

        for (index = 0U; index < parser->buffer_len; ++index)
        {
            if (parser->buffer[index] == kEscFe32Prefix[0])
            {
                break;
            }
        }

        if (index == parser->buffer_len)
        {
            esc_fe32_drop_buffer_prefix(parser, parser->buffer_len);
            return;
        }

        if (index != 0U)
        {
            esc_fe32_drop_buffer_prefix(parser, index);
        }

        limit = parser->buffer_len;
        if (limit > ESC_FE32_PREFIX_LEN)
        {
            limit = ESC_FE32_PREFIX_LEN;
        }

        for (index = 0U; index < limit; ++index)
        {
            if (parser->buffer[index] != kEscFe32Prefix[index])
            {
                prefix_ok = 0U;
                break;
            }
        }

        if (prefix_ok != 0U)
        {
            return;
        }

        parser->prefix_rejected_candidates++;
        esc_fe32_drop_buffer_prefix(parser, 1U);
    }
}

static void esc_fe32_decode_fields(EscFe32Sample_t *sample)
{
    sample->throttle_request_raw = sample->raw[9];
    sample->throttle_output_raw = sample->raw[10];
    sample->state_raw = sample->raw[11];

    switch (sample->state_raw)
    {
    case 0U:
        sample->state_candidate_valid = 1U;
        sample->state_candidate = ESC_FE32_STATE_CANDIDATE_NEUTRAL;
        break;
    case 1U:
        sample->state_candidate_valid = 1U;
        sample->state_candidate = ESC_FE32_STATE_CANDIDATE_DRIVE_AMBIGUOUS;
        break;
    case 2U:
        sample->state_candidate_valid = 1U;
        sample->state_candidate = ESC_FE32_STATE_CANDIDATE_BRAKE;
        break;
    default:
        sample->state_candidate_valid = 0U;
        sample->state_candidate = ESC_FE32_STATE_CANDIDATE_UNKNOWN;
        break;
    }

    sample->rpm_raw = esc_fe32_read_u16_le(&sample->raw[13]);
    sample->rpm_valid = (sample->rpm_raw != 0xFFFFU) ? 1U : 0U;
    sample->erpm_candidate = (sample->rpm_valid != 0U) ?
        ((uint32_t)sample->rpm_raw * 10UL) : 0UL;

    sample->voltage_raw = esc_fe32_read_u16_le(&sample->raw[15]);
    sample->voltage_valid = (sample->voltage_raw != 0xFFFFU) ? 1U : 0U;
    sample->voltage_deci_v = (sample->voltage_valid != 0U) ?
        sample->voltage_raw : 0U;

    sample->current_raw = esc_fe32_read_u16_le(&sample->raw[17]);
    sample->current_valid = (sample->current_raw != 0xFFFFU) ? 1U : 0U;
    sample->current_deci_a = (sample->current_valid != 0U) ?
        sample->current_raw : 0U;

    sample->esc_temperature_raw = sample->raw[19];
    sample->esc_temperature_valid =
        (sample->esc_temperature_raw != 0xFFU) ? 1U : 0U;
    sample->esc_temperature_c = (sample->esc_temperature_valid != 0U) ?
        (int16_t)sample->esc_temperature_raw : 0;

    sample->motor_temperature_raw = sample->raw[21];
    sample->motor_temperature_valid =
        (sample->motor_temperature_raw != 0xFFU) ? 1U : 0U;
    sample->motor_temperature_c = (sample->motor_temperature_valid != 0U) ?
        (int16_t)sample->motor_temperature_raw : 0;
}

EscFe32Status_t EscFe32_DecodeFrame(const uint8_t frame[ESC_FE32_FRAME_LEN],
                                    uint32_t sample_id,
                                    uint32_t received_tick_ms,
                                    EscFe32Sample_t *sample)
{
    size_t index;
    uint16_t stored_crc;
    uint16_t computed_crc;

    if (frame == NULL || sample == NULL)
    {
        return ESC_FE32_STATUS_INVALID_ARGUMENT;
    }

    for (index = 0U; index < ESC_FE32_PREFIX_LEN; ++index)
    {
        if (frame[index] != kEscFe32Prefix[index])
        {
            return ESC_FE32_STATUS_BAD_PREFIX;
        }
    }

    computed_crc = EscFe32_Crc16Modbus(frame, ESC_FE32_FRAME_LEN - 2U);
    stored_crc = esc_fe32_read_u16_le(&frame[ESC_FE32_FRAME_LEN - 2U]);
    if (computed_crc != stored_crc)
    {
        return ESC_FE32_STATUS_BAD_CRC;
    }

    memset(sample, 0, sizeof(*sample));
    memcpy(sample->raw, frame, ESC_FE32_FRAME_LEN);
    sample->sample_id = sample_id;
    sample->received_tick_ms = received_tick_ms;
    sample->crc_stored = stored_crc;
    sample->crc_computed = computed_crc;
    esc_fe32_decode_fields(sample);

    return ESC_FE32_STATUS_OK;
}

size_t EscFe32Parser_PushBytes(EscFe32Parser_t *parser,
                               const uint8_t *data,
                               size_t length,
                               uint32_t received_tick_ms,
                               EscFe32Sample_t *out_samples,
                               size_t out_capacity)
{
    size_t index;
    size_t emitted = 0U;

    if (parser == NULL || (data == NULL && length != 0U))
    {
        return 0U;
    }

    for (index = 0U; index < length; ++index)
    {
        EscFe32Sample_t sample;
        EscFe32Status_t status;

        if (parser->buffer_len >= ESC_FE32_FRAME_LEN)
        {
            parser->buffer_len = 0U;
        }

        parser->buffer[parser->buffer_len++] = data[index];
        esc_fe32_resync(parser);

        if (parser->buffer_len != ESC_FE32_FRAME_LEN)
        {
            continue;
        }

        status = EscFe32_DecodeFrame(parser->buffer,
                                     parser->next_sample_id,
                                     received_tick_ms,
                                     &sample);
        if (status == ESC_FE32_STATUS_OK)
        {
            if (out_samples != NULL && emitted < out_capacity)
            {
                out_samples[emitted] = sample;
                emitted++;
            }
            else
            {
                parser->output_overrun_frames++;
            }
            parser->next_sample_id++;
            parser->decoded_frames++;
            parser->buffer_len = 0U;
        }
        else
        {
            if (status == ESC_FE32_STATUS_BAD_CRC)
            {
                parser->crc_failed_frames++;
            }
            else if (status == ESC_FE32_STATUS_BAD_PREFIX)
            {
                parser->prefix_rejected_candidates++;
            }
            esc_fe32_drop_buffer_prefix(parser, 1U);
            esc_fe32_resync(parser);
        }
    }

    return emitted;
}
