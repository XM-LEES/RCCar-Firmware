#include "esc_fe32_parser.h"
#include "../fixtures/esc_fe32_samples.h"

#include <stdio.h>
#include <string.h>

#define EXPECT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "%s:%d: expectation failed: %s\n", \
                    __FILE__, __LINE__, #condition); \
            return 1; \
        } \
    } while (0)

static void write_crc(uint8_t frame[ESC_FE32_FRAME_LEN])
{
    const uint16_t crc = EscFe32_Crc16Modbus(frame, ESC_FE32_FRAME_LEN - 2U);
    frame[ESC_FE32_FRAME_LEN - 2U] = (uint8_t)crc;
    frame[ESC_FE32_FRAME_LEN - 1U] = (uint8_t)(crc >> 8);
}

static int test_real_fixture_fields(void)
{
    EscFe32Sample_t sample;

    EXPECT_TRUE(EscFe32_DecodeFrame(ESC_FE32_FIXTURE_PEAK_DYN02,
                                    77U,
                                    1234U,
                                    &sample) == ESC_FE32_STATUS_OK);
    EXPECT_TRUE(sample.sample_id == 77U);
    EXPECT_TRUE(sample.received_tick_ms == 1234U);
    EXPECT_TRUE(sample.crc_stored == 0x257CU);
    EXPECT_TRUE(sample.crc_computed == 0x257CU);
    EXPECT_TRUE(sample.throttle_request_raw == 0x16U);
    EXPECT_TRUE(sample.throttle_output_raw == 0x17U);
    EXPECT_TRUE(sample.state_raw == 1U);
    EXPECT_TRUE(sample.state_candidate_valid == 1U);
    EXPECT_TRUE(sample.state_candidate == ESC_FE32_STATE_CANDIDATE_DRIVE_AMBIGUOUS);
    EXPECT_TRUE(sample.rpm_valid == 1U);
    EXPECT_TRUE(sample.rpm_raw == 4204U);
    EXPECT_TRUE(sample.erpm_candidate == 42040UL);
    EXPECT_TRUE(sample.voltage_valid == 1U);
    EXPECT_TRUE(sample.voltage_raw == 308U);
    EXPECT_TRUE(sample.voltage_deci_v == 308U);
    EXPECT_TRUE(sample.current_valid == 1U);
    EXPECT_TRUE(sample.current_raw == 25U);
    EXPECT_TRUE(sample.current_deci_a == 25U);
    EXPECT_TRUE(sample.esc_temperature_valid == 1U);
    EXPECT_TRUE(sample.esc_temperature_c == 28);
    EXPECT_TRUE(sample.motor_temperature_valid == 1U);
    EXPECT_TRUE(sample.motor_temperature_c == 28);
    EXPECT_TRUE(memcmp(sample.raw, ESC_FE32_FIXTURE_PEAK_DYN02,
                       ESC_FE32_FRAME_LEN) == 0);

    return 0;
}

static int test_state_candidates(void)
{
    EscFe32Sample_t sample;

    EXPECT_TRUE(EscFe32_DecodeFrame(ESC_FE32_FIXTURE_IDLE_FIRST,
                                    1U,
                                    10U,
                                    &sample) == ESC_FE32_STATUS_OK);
    EXPECT_TRUE(sample.state_raw == 0U);
    EXPECT_TRUE(sample.state_candidate == ESC_FE32_STATE_CANDIDATE_NEUTRAL);

    EXPECT_TRUE(EscFe32_DecodeFrame(ESC_FE32_FIXTURE_DRIVE_DYN01,
                                    2U,
                                    20U,
                                    &sample) == ESC_FE32_STATUS_OK);
    EXPECT_TRUE(sample.state_raw == 1U);
    EXPECT_TRUE(sample.state_candidate == ESC_FE32_STATE_CANDIDATE_DRIVE_AMBIGUOUS);

    EXPECT_TRUE(EscFe32_DecodeFrame(ESC_FE32_FIXTURE_BRAKE_DYN02,
                                    3U,
                                    30U,
                                    &sample) == ESC_FE32_STATUS_OK);
    EXPECT_TRUE(sample.state_raw == 2U);
    EXPECT_TRUE(sample.state_candidate == ESC_FE32_STATE_CANDIDATE_BRAKE);

    return 0;
}

static int test_invalid_markers_keep_raw_values(void)
{
    uint8_t frame[ESC_FE32_FRAME_LEN];
    EscFe32Sample_t sample;

    memcpy(frame, ESC_FE32_FIXTURE_PEAK_DYN02, sizeof(frame));
    frame[13] = 0xFFU;
    frame[14] = 0xFFU;
    frame[15] = 0xFFU;
    frame[16] = 0xFFU;
    frame[17] = 0xFFU;
    frame[18] = 0xFFU;
    frame[19] = 0xFFU;
    frame[21] = 0xFFU;
    write_crc(frame);

    EXPECT_TRUE(EscFe32_DecodeFrame(frame, 4U, 40U, &sample) == ESC_FE32_STATUS_OK);
    EXPECT_TRUE(sample.rpm_raw == 0xFFFFU);
    EXPECT_TRUE(sample.rpm_valid == 0U);
    EXPECT_TRUE(sample.erpm_candidate == 0UL);
    EXPECT_TRUE(sample.voltage_raw == 0xFFFFU);
    EXPECT_TRUE(sample.voltage_valid == 0U);
    EXPECT_TRUE(sample.current_raw == 0xFFFFU);
    EXPECT_TRUE(sample.current_valid == 0U);
    EXPECT_TRUE(sample.esc_temperature_raw == 0xFFU);
    EXPECT_TRUE(sample.esc_temperature_valid == 0U);
    EXPECT_TRUE(sample.motor_temperature_raw == 0xFFU);
    EXPECT_TRUE(sample.motor_temperature_valid == 0U);

    return 0;
}

static int test_stream_parser_resync_and_freshness(void)
{
    EscFe32Parser_t parser;
    EscFe32Sample_t out[4];
    uint8_t corrupt[ESC_FE32_FRAME_LEN];
    uint8_t stream[ESC_FE32_FRAME_LEN * 2U];
    static const uint8_t noise[] = {0x12U, 0xFEU, 0x99U, 0x00U};
    size_t produced;

    EscFe32Parser_Init(&parser);
    produced = EscFe32Parser_PushBytes(&parser, noise, sizeof(noise), 10U, out, 4U);
    EXPECT_TRUE(produced == 0U);
    EXPECT_TRUE(parser.prefix_rejected_candidates == 1U);

    produced = EscFe32Parser_PushBytes(&parser, ESC_FE32_FIXTURE_IDLE_FIRST, 9U,
                                       20U, out, 4U);
    EXPECT_TRUE(produced == 0U);
    produced = EscFe32Parser_PushBytes(&parser, &ESC_FE32_FIXTURE_IDLE_FIRST[9],
                                       ESC_FE32_FRAME_LEN - 9U, 30U, out, 4U);
    EXPECT_TRUE(produced == 1U);
    EXPECT_TRUE(out[0].sample_id == 1U);
    EXPECT_TRUE(out[0].received_tick_ms == 30U);

    produced = EscFe32Parser_PushBytes(&parser, NULL, 0U, 31U, out, 4U);
    EXPECT_TRUE(produced == 0U);
    EXPECT_TRUE(parser.next_sample_id == 2U);

    memcpy(corrupt, ESC_FE32_FIXTURE_PEAK_DYN02, sizeof(corrupt));
    corrupt[20] = 0xFEU;
    memcpy(stream, corrupt, ESC_FE32_FRAME_LEN);
    memcpy(&stream[ESC_FE32_FRAME_LEN], ESC_FE32_FIXTURE_BRAKE_DYN02,
           ESC_FE32_FRAME_LEN);

    produced = EscFe32Parser_PushBytes(&parser, stream, sizeof(stream), 50U,
                                       out, 4U);
    EXPECT_TRUE(produced == 1U);
    EXPECT_TRUE(out[0].sample_id == 2U);
    EXPECT_TRUE(out[0].state_candidate == ESC_FE32_STATE_CANDIDATE_BRAKE);
    EXPECT_TRUE(parser.crc_failed_frames == 1U);
    EXPECT_TRUE(parser.decoded_frames == 2U);
    EXPECT_TRUE(parser.next_sample_id == 3U);

    produced = EscFe32Parser_PushBytes(&parser, ESC_FE32_FIXTURE_PEAK_DYN02,
                                       ESC_FE32_FRAME_LEN, 60U, out, 4U);
    EXPECT_TRUE(produced == 1U);
    EXPECT_TRUE(out[0].sample_id == 3U);
    EXPECT_TRUE(parser.next_sample_id == 4U);

    return 0;
}

int main(void)
{
    if (test_real_fixture_fields() != 0)
    {
        return 1;
    }
    if (test_state_candidates() != 0)
    {
        return 1;
    }
    if (test_invalid_markers_keep_raw_values() != 0)
    {
        return 1;
    }
    if (test_stream_parser_resync_and_freshness() != 0)
    {
        return 1;
    }

    return 0;
}
