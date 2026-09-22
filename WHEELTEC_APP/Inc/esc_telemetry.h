#ifndef __ESC_TELEMETRY_H
#define __ESC_TELEMETRY_H

#include <stddef.h>
#include <stdint.h>

#include "esc_fe32_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESC_TELEMETRY_DMA_BUFFER_LEN 128U
#define ESC_TELEMETRY_PENDING_CHUNK_SIZE 64U
#define ESC_TELEMETRY_PENDING_CHUNKS 16U
#define ESC_TELEMETRY_OBSERVED_SAMPLE_QUEUE_LEN 8U

#define ESC_TELEMETRY_CONTEXT_PWM_MASK 0x00000FFFUL
#define ESC_TELEMETRY_CONTEXT_RC_ACTIVE_MASK 0x00001000UL
#define ESC_TELEMETRY_CONTEXT_SOURCE_SHIFT 13U
#define ESC_TELEMETRY_CONTEXT_SOURCE_MASK 0xFFFFE000UL

typedef enum
{
    ESC_TELEMETRY_RX_EVENT_IDLE = 0,
    ESC_TELEMETRY_RX_EVENT_HALF_TRANSFER,
    ESC_TELEMETRY_RX_EVENT_TRANSFER_COMPLETE,
    ESC_TELEMETRY_RX_EVENT_POLL,
    ESC_TELEMETRY_RX_EVENT_COUNT
} EscTelemetryRxEvent_t;

typedef enum
{
    ESC_TELEMETRY_INVALIDATION_NONE = 0,
    ESC_TELEMETRY_INVALIDATION_RX_ERROR,
    ESC_TELEMETRY_INVALIDATION_PENDING_OVERFLOW,
    ESC_TELEMETRY_INVALIDATION_DMA_AMBIGUOUS,
    ESC_TELEMETRY_INVALIDATION_RECEIVER_RESTART
} EscTelemetryInvalidationReason_t;

typedef struct
{
    uint8_t has_sample;
    uint32_t publish_sequence;
    uint32_t receive_epoch;
    EscFe32Sample_t sample;
} EscTelemetrySnapshot_t;

typedef struct
{
    EscFe32Sample_t sample;
    uint32_t receive_epoch;
    uint32_t delivery_epoch;
    uint32_t output_context;
} EscTelemetryObservedSample_t;

typedef struct
{
    uint32_t receive_epoch;
    uint32_t delivery_epoch;
    uint32_t dma_events[ESC_TELEMETRY_RX_EVENT_COUNT];
    uint32_t duplicate_dma_events;
    uint32_t ambiguous_dma_events;
    uint32_t invalid_dma_positions;
    uint32_t dma_wrap_events;
    uint32_t bytes_copied_from_dma;
    uint32_t bytes_copied_from_receiver;
    uint32_t pending_chunks_pushed;
    uint32_t pending_chunks_dropped;
    uint32_t pending_bytes_dropped;
    uint32_t stale_chunks_discarded;
    uint32_t stale_samples_suppressed;
    uint32_t receive_invalidations;
    uint32_t last_invalidation_tick_ms;
    uint32_t last_invalidation_reason;
    uint32_t samples_published;
    uint32_t rx_error_count;
    uint32_t last_rx_error_tick_ms;
    uint32_t last_rx_error_flags;
    uint32_t parser_decoded_frames;
    uint32_t parser_crc_failed_frames;
    uint32_t parser_prefix_rejected_candidates;
    uint32_t parser_discarded_bytes;
    uint32_t parser_output_overrun_frames;
    uint32_t observed_samples_queued;
    uint32_t observed_samples_consumed;
    uint32_t observed_samples_discarded;
    uint32_t observed_queue_overflows;
    uint32_t observed_queue_depth_peak;
    uint32_t observed_context_rejected;
    uint32_t observed_generation_boundaries;
    uint32_t output_context_updates;
    uint32_t output_context_generation;
} EscTelemetryDiagnostics_t;

typedef struct
{
    uint32_t samples_published;
    uint32_t rx_error_count;
    uint32_t last_rx_error_flags;
} EscTelemetryReceiverHealth_t;

void EscTelemetry_Init(void);
uint8_t *EscTelemetry_GetDmaBuffer(void);
size_t EscTelemetry_GetDmaBufferLength(void);
void EscTelemetry_ResetDmaCursor(void);
void EscTelemetry_ResetReceiveBoundary(
    uint32_t tick_ms,
    EscTelemetryInvalidationReason_t reason);
void EscTelemetry_RecordDmaEvent(size_t dma_write_pos,
                                 uint32_t received_tick_ms,
                                 EscTelemetryRxEvent_t event);
void EscTelemetry_RecordBytes(const uint8_t *data,
                              size_t length,
                              uint32_t received_tick_ms);
void EscTelemetry_RecordRxError(uint32_t tick_ms, uint32_t error_flags);
uint8_t EscTelemetry_ProcessReceivedByte(uint8_t byte,
                                         uint32_t received_tick_ms,
                                         uint32_t output_context);
void EscTelemetry_ProcessPending(void);
uint8_t EscTelemetry_GetSnapshot(EscTelemetrySnapshot_t *snapshot);
uint8_t EscTelemetry_GetFreshSample(uint32_t now_tick_ms,
                                    uint32_t timeout_ms,
                                    EscFe32Sample_t *sample);
void EscTelemetry_GetDiagnostics(EscTelemetryDiagnostics_t *diagnostics);
void EscTelemetry_GetReceiverHealth(EscTelemetryReceiverHealth_t *health);
uint32_t EscTelemetry_PublishOutputContext(uint16_t pwm_us,
                                           uint8_t rc_active);
uint32_t EscTelemetry_GetOutputContext(void);
uint16_t EscTelemetry_ContextPwm(uint32_t output_context);
uint8_t EscTelemetry_ContextRcActive(uint32_t output_context);
uint32_t EscTelemetry_ContextSource(uint32_t output_context);
uint8_t EscTelemetry_ContextIsValid(uint32_t output_context);
size_t EscTelemetry_PendingSamples(void);
uint8_t EscTelemetry_PopSample(EscTelemetryObservedSample_t *sample);
void EscTelemetry_DiscardSamples(void);
uint32_t EscTelemetry_GetDeliveryEpoch(void);

#ifdef __cplusplus
}
#endif

#endif
