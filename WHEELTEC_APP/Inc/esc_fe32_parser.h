#ifndef __ESC_FE32_PARSER_H
#define __ESC_FE32_PARSER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ESC_FE32_FRAME_LEN 32U
#define ESC_FE32_PREFIX_LEN 5U

typedef enum
{
    ESC_FE32_STATUS_OK = 0,
    ESC_FE32_STATUS_INVALID_ARGUMENT,
    ESC_FE32_STATUS_BAD_PREFIX,
    ESC_FE32_STATUS_BAD_CRC
} EscFe32Status_t;

typedef enum
{
    ESC_FE32_STATE_CANDIDATE_NEUTRAL = 0,
    ESC_FE32_STATE_CANDIDATE_DRIVE_AMBIGUOUS = 1,
    ESC_FE32_STATE_CANDIDATE_BRAKE = 2,
    ESC_FE32_STATE_CANDIDATE_UNKNOWN = 255
} EscFe32StateCandidate_t;

typedef struct
{
    uint8_t raw[ESC_FE32_FRAME_LEN];
    uint32_t sample_id;
    uint32_t received_tick_ms;
    uint16_t crc_stored;
    uint16_t crc_computed;

    uint8_t throttle_request_raw;
    uint8_t throttle_output_raw;
    uint8_t state_raw;
    uint8_t state_candidate_valid;
    EscFe32StateCandidate_t state_candidate;

    uint16_t rpm_raw;
    uint8_t rpm_valid;
    uint32_t erpm_candidate;

    uint16_t voltage_raw;
    uint8_t voltage_valid;
    uint16_t voltage_deci_v;

    uint16_t current_raw;
    uint8_t current_valid;
    uint16_t current_deci_a;

    uint8_t esc_temperature_raw;
    uint8_t esc_temperature_valid;
    int16_t esc_temperature_c;

    uint8_t motor_temperature_raw;
    uint8_t motor_temperature_valid;
    int16_t motor_temperature_c;
} EscFe32Sample_t;

typedef struct
{
    uint8_t buffer[ESC_FE32_FRAME_LEN];
    size_t buffer_len;
    uint32_t next_sample_id;
    uint32_t decoded_frames;
    uint32_t crc_failed_frames;
    uint32_t prefix_rejected_candidates;
    uint32_t discarded_bytes;
    uint32_t output_overrun_frames;
} EscFe32Parser_t;

uint16_t EscFe32_Crc16Modbus(const uint8_t *data, size_t length);
void EscFe32Parser_Init(EscFe32Parser_t *parser);
EscFe32Status_t EscFe32_DecodeFrame(const uint8_t frame[ESC_FE32_FRAME_LEN],
                                    uint32_t sample_id,
                                    uint32_t received_tick_ms,
                                    EscFe32Sample_t *sample);
size_t EscFe32Parser_PushBytes(EscFe32Parser_t *parser,
                               const uint8_t *data,
                               size_t length,
                               uint32_t received_tick_ms,
                               EscFe32Sample_t *out_samples,
                               size_t out_capacity);

#ifdef __cplusplus
}
#endif

#endif
