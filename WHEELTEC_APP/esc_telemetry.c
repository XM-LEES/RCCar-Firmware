#include "esc_telemetry.h"

#include <string.h>

#if defined(STM32F407xx)
#include "FreeRTOSConfig.h"
#include "stm32f4xx.h"
typedef uint32_t EscTelemetryIrqState_t;

static EscTelemetryIrqState_t esc_telemetry_enter_critical(void)
{
    EscTelemetryIrqState_t state = __get_BASEPRI();

    __set_BASEPRI_MAX(configMAX_SYSCALL_INTERRUPT_PRIORITY);
    __DSB();
    __ISB();
    return state;
}

static void esc_telemetry_exit_critical(EscTelemetryIrqState_t state)
{
    __set_BASEPRI(state);
    __DSB();
    __ISB();
}
#else
typedef uint8_t EscTelemetryIrqState_t;

static EscTelemetryIrqState_t esc_telemetry_enter_critical(void)
{
    return 0U;
}

static void esc_telemetry_exit_critical(EscTelemetryIrqState_t state)
{
    (void)state;
}
#endif

typedef struct
{
    uint8_t data[ESC_TELEMETRY_PENDING_CHUNK_SIZE];
    size_t length;
    uint32_t received_tick_ms;
    uint32_t receive_epoch;
    uint32_t output_context;
    uint8_t output_context_valid;
} EscTelemetryPendingChunk_t;

typedef struct
{
    uint32_t decoded_frames;
    uint32_t crc_failed_frames;
    uint32_t prefix_rejected_candidates;
    uint32_t discarded_bytes;
    uint32_t output_overrun_frames;
} EscTelemetryParserTotals_t;

static uint8_t s_dma_buffer[ESC_TELEMETRY_DMA_BUFFER_LEN];
static size_t s_last_dma_pos;
static uint8_t s_dma_cursor_valid;
static EscTelemetryPendingChunk_t s_pending_chunks[ESC_TELEMETRY_PENDING_CHUNKS];
static size_t s_pending_head;
static size_t s_pending_tail;
static size_t s_pending_count;
static EscFe32Parser_t s_parser;
static EscTelemetryParserTotals_t s_parser_totals;
static EscTelemetrySnapshot_t s_snapshot;
static EscTelemetryDiagnostics_t s_diagnostics;
static EscTelemetryObservedSample_t
    s_observed_samples[ESC_TELEMETRY_OBSERVED_SAMPLE_QUEUE_LEN];
static size_t s_observed_head;
static size_t s_observed_tail;
static size_t s_observed_count;
static uint32_t s_delivery_epoch;
static uint8_t s_last_observed_source_valid;
static uint32_t s_last_observed_source;
static uint32_t s_recent_output_context[ESC_FE32_FRAME_LEN];
static size_t s_recent_byte_meta_next;
static size_t s_recent_byte_meta_count;
static volatile uint32_t s_current_output_context;
static uint32_t s_output_context_generation;
static uint8_t s_output_context_generation_valid;
static uint8_t s_output_context_rc_active;
static uint32_t s_receive_epoch;
static uint8_t s_parser_reset_pending;

uint16_t EscTelemetry_ContextPwm(uint32_t output_context)
{
    return (uint16_t)(output_context & ESC_TELEMETRY_CONTEXT_PWM_MASK);
}

uint8_t EscTelemetry_ContextRcActive(uint32_t output_context)
{
    return ((output_context & ESC_TELEMETRY_CONTEXT_RC_ACTIVE_MASK) != 0UL) ?
        1U : 0U;
}

uint32_t EscTelemetry_ContextSource(uint32_t output_context)
{
    return (output_context & ESC_TELEMETRY_CONTEXT_SOURCE_MASK) >>
        ESC_TELEMETRY_CONTEXT_SOURCE_SHIFT;
}

uint8_t EscTelemetry_ContextIsValid(uint32_t output_context)
{
    return (EscTelemetry_ContextSource(output_context) != 0UL) ? 1U : 0U;
}

static void esc_telemetry_sync_parser_diagnostics(void)
{
    s_diagnostics.parser_decoded_frames =
        s_parser_totals.decoded_frames + s_parser.decoded_frames;
    s_diagnostics.parser_crc_failed_frames =
        s_parser_totals.crc_failed_frames + s_parser.crc_failed_frames;
    s_diagnostics.parser_prefix_rejected_candidates =
        s_parser_totals.prefix_rejected_candidates +
        s_parser.prefix_rejected_candidates;
    s_diagnostics.parser_discarded_bytes =
        s_parser_totals.discarded_bytes + s_parser.discarded_bytes;
    s_diagnostics.parser_output_overrun_frames =
        s_parser_totals.output_overrun_frames +
        s_parser.output_overrun_frames;
    s_diagnostics.receive_epoch = s_receive_epoch;
    s_diagnostics.delivery_epoch = s_delivery_epoch;
    s_diagnostics.output_context_generation = s_output_context_generation;
}

static void esc_telemetry_clear_recent_byte_meta(void)
{
    memset(s_recent_output_context, 0, sizeof(s_recent_output_context));
    s_recent_byte_meta_next = 0U;
    s_recent_byte_meta_count = 0U;
}

static void esc_telemetry_advance_delivery_epoch_locked(void)
{
    s_delivery_epoch++;
    if (s_delivery_epoch == 0UL)
    {
        s_delivery_epoch = 1UL;
    }
    s_last_observed_source_valid = 0U;
    s_last_observed_source = 0UL;
    s_diagnostics.delivery_epoch = s_delivery_epoch;
}

static void esc_telemetry_clear_observed_locked(void)
{
    s_observed_head = 0U;
    s_observed_tail = 0U;
    s_observed_count = 0U;
}

static void esc_telemetry_discard_observed_locked(void)
{
    if (s_observed_count != 0U)
    {
        s_diagnostics.observed_samples_discarded +=
            (uint32_t)s_observed_count;
        esc_telemetry_clear_observed_locked();
    }
}

static void esc_telemetry_accumulate_parser_diagnostics(void)
{
    s_parser_totals.decoded_frames += s_parser.decoded_frames;
    s_parser_totals.crc_failed_frames += s_parser.crc_failed_frames;
    s_parser_totals.prefix_rejected_candidates +=
        s_parser.prefix_rejected_candidates;
    s_parser_totals.discarded_bytes += s_parser.discarded_bytes;
    s_parser_totals.output_overrun_frames += s_parser.output_overrun_frames;
}

static void esc_telemetry_reset_parser_preserve_sample_ids(void)
{
    uint32_t next_sample_id = s_parser.next_sample_id;

    if (next_sample_id == 0U)
    {
        next_sample_id = 1U;
    }

    esc_telemetry_accumulate_parser_diagnostics();
    EscFe32Parser_Init(&s_parser);
    s_parser.next_sample_id = next_sample_id;
    esc_telemetry_clear_recent_byte_meta();
}

static void esc_telemetry_advance_epoch(void)
{
    s_receive_epoch++;
    if (s_receive_epoch == 0U)
    {
        s_receive_epoch = 1U;
    }
}

static void esc_telemetry_clear_pending(void)
{
    s_pending_head = 0U;
    s_pending_tail = 0U;
    s_pending_count = 0U;
}

static void esc_telemetry_invalidate_locked(
    uint32_t tick_ms,
    EscTelemetryInvalidationReason_t reason)
{
    esc_telemetry_advance_epoch();
    esc_telemetry_clear_pending();
    esc_telemetry_discard_observed_locked();
    esc_telemetry_advance_delivery_epoch_locked();
    esc_telemetry_clear_recent_byte_meta();
    s_dma_cursor_valid = 0U;
    s_parser_reset_pending = 1U;
    if (s_snapshot.has_sample != 0U)
    {
        s_snapshot.has_sample = 0U;
        s_snapshot.publish_sequence++;
    }
    s_snapshot.receive_epoch = s_receive_epoch;
    s_diagnostics.receive_invalidations++;
    s_diagnostics.last_invalidation_tick_ms = tick_ms;
    s_diagnostics.last_invalidation_reason = (uint32_t)reason;
    s_diagnostics.receive_epoch = s_receive_epoch;
}

static void esc_telemetry_reset_parser_if_requested(void)
{
    uint8_t should_reset = 0U;
    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    if (s_parser_reset_pending != 0U)
    {
        s_parser_reset_pending = 0U;
        should_reset = 1U;
    }
    esc_telemetry_exit_critical(irq_state);

    if (should_reset != 0U)
    {
        esc_telemetry_reset_parser_preserve_sample_ids();
    }
}

static uint32_t esc_telemetry_get_epoch(void)
{
    uint32_t epoch;
    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    epoch = s_receive_epoch;
    esc_telemetry_exit_critical(irq_state);
    return epoch;
}

void EscTelemetry_Init(void)
{
    memset(s_dma_buffer, 0, sizeof(s_dma_buffer));
    memset(s_pending_chunks, 0, sizeof(s_pending_chunks));
    memset(&s_parser_totals, 0, sizeof(s_parser_totals));
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(&s_diagnostics, 0, sizeof(s_diagnostics));
    memset(s_observed_samples, 0, sizeof(s_observed_samples));
    s_pending_head = 0U;
    s_pending_tail = 0U;
    s_pending_count = 0U;
    s_observed_head = 0U;
    s_observed_tail = 0U;
    s_observed_count = 0U;
    s_delivery_epoch = 1U;
    s_last_observed_source_valid = 0U;
    s_last_observed_source = 0UL;
    esc_telemetry_clear_recent_byte_meta();
    s_current_output_context = 0UL;
    s_output_context_generation = 0UL;
    s_output_context_generation_valid = 0U;
    s_output_context_rc_active = 0U;
    s_last_dma_pos = 0U;
    s_dma_cursor_valid = 1U;
    s_receive_epoch = 1U;
    s_parser_reset_pending = 0U;
    EscFe32Parser_Init(&s_parser);
    s_snapshot.receive_epoch = s_receive_epoch;
    s_diagnostics.receive_epoch = s_receive_epoch;
    s_diagnostics.delivery_epoch = s_delivery_epoch;
}

uint8_t *EscTelemetry_GetDmaBuffer(void)
{
    return s_dma_buffer;
}

size_t EscTelemetry_GetDmaBufferLength(void)
{
    return sizeof(s_dma_buffer);
}

void EscTelemetry_ResetDmaCursor(void)
{
    s_last_dma_pos = 0U;
    s_dma_cursor_valid = 1U;
}

void EscTelemetry_ResetReceiveBoundary(
    uint32_t tick_ms,
    EscTelemetryInvalidationReason_t reason)
{
    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    esc_telemetry_invalidate_locked(tick_ms, reason);
    esc_telemetry_exit_critical(irq_state);
}

static uint8_t esc_telemetry_push_pending(const uint8_t *data,
                                          size_t length,
                                          uint32_t tick_ms,
                                          uint32_t output_context,
                                          uint8_t output_context_valid)
{
    size_t offset = 0U;

    while (offset < length)
    {
        const size_t remaining = length - offset;
        const size_t chunk_len =
            (remaining > ESC_TELEMETRY_PENDING_CHUNK_SIZE) ?
            ESC_TELEMETRY_PENDING_CHUNK_SIZE : remaining;

        if (s_pending_count >= ESC_TELEMETRY_PENDING_CHUNKS)
        {
            s_diagnostics.pending_chunks_dropped++;
            s_diagnostics.pending_bytes_dropped += (uint32_t)(length - offset);
            return 0U;
        }

        memcpy(s_pending_chunks[s_pending_tail].data, &data[offset], chunk_len);
        s_pending_chunks[s_pending_tail].length = chunk_len;
        s_pending_chunks[s_pending_tail].received_tick_ms = tick_ms;
        s_pending_chunks[s_pending_tail].receive_epoch = s_receive_epoch;
        s_pending_chunks[s_pending_tail].output_context = output_context;
        s_pending_chunks[s_pending_tail].output_context_valid =
            (output_context_valid != 0U &&
             EscTelemetry_ContextIsValid(output_context) != 0U) ? 1U : 0U;
        s_pending_tail = (s_pending_tail + 1U) % ESC_TELEMETRY_PENDING_CHUNKS;
        s_pending_count++;
        s_diagnostics.pending_chunks_pushed++;
        offset += chunk_len;
    }

    return 1U;
}

static uint8_t esc_telemetry_copy_dma_range(size_t start,
                                            size_t end,
                                            uint32_t tick_ms)
{
    const size_t length = end - start;
    const uint32_t output_context = EscTelemetry_GetOutputContext();

    if (length == 0U)
    {
        return 1U;
    }

    if (esc_telemetry_push_pending(&s_dma_buffer[start],
                                   length,
                                   tick_ms,
                                   output_context,
                                   EscTelemetry_ContextIsValid(
                                       output_context)) == 0U)
    {
        return 0U;
    }
    s_diagnostics.bytes_copied_from_dma += (uint32_t)length;
    return 1U;
}

static uint8_t esc_telemetry_same_position_is_ambiguous(
    EscTelemetryRxEvent_t event)
{
    /*
     * A repeated IDLE at the same NDTR position is a common no-new-byte
     * duplicate after HT/TC/IDLE handling. A repeated HT or TC at the same
     * sampled position means either a duplicate IRQ or an entire circular
     * buffer lap was missed; the data boundary is then unknowable, so force
     * resynchronization instead of silently splicing bytes across the gap.
     * Pure cursor math still cannot prove every possible full-lap loss.
     */
    return (event == ESC_TELEMETRY_RX_EVENT_HALF_TRANSFER ||
            event == ESC_TELEMETRY_RX_EVENT_TRANSFER_COMPLETE) ? 1U : 0U;
}

void EscTelemetry_RecordDmaEvent(size_t dma_write_pos,
                                 uint32_t received_tick_ms,
                                 EscTelemetryRxEvent_t event)
{
    if ((uint32_t)event < (uint32_t)ESC_TELEMETRY_RX_EVENT_COUNT)
    {
        s_diagnostics.dma_events[event]++;
    }

    if (dma_write_pos > sizeof(s_dma_buffer))
    {
        s_diagnostics.invalid_dma_positions++;
        esc_telemetry_invalidate_locked(
            received_tick_ms,
            ESC_TELEMETRY_INVALIDATION_DMA_AMBIGUOUS);
        return;
    }

    if (s_dma_cursor_valid == 0U)
    {
        s_last_dma_pos = dma_write_pos;
        s_dma_cursor_valid = 1U;
        return;
    }

    if (dma_write_pos == s_last_dma_pos)
    {
        if (esc_telemetry_same_position_is_ambiguous(event) != 0U)
        {
            s_diagnostics.ambiguous_dma_events++;
            esc_telemetry_invalidate_locked(
                received_tick_ms,
                ESC_TELEMETRY_INVALIDATION_DMA_AMBIGUOUS);
            s_last_dma_pos = dma_write_pos;
            s_dma_cursor_valid = 1U;
        }
        else
        {
            s_diagnostics.duplicate_dma_events++;
        }
        return;
    }

    if (dma_write_pos > s_last_dma_pos)
    {
        if (esc_telemetry_copy_dma_range(s_last_dma_pos,
                                         dma_write_pos,
                                         received_tick_ms) == 0U)
        {
            esc_telemetry_invalidate_locked(
                received_tick_ms,
                ESC_TELEMETRY_INVALIDATION_PENDING_OVERFLOW);
            s_last_dma_pos = dma_write_pos;
            s_dma_cursor_valid = 1U;
            return;
        }
    }
    else
    {
        if (esc_telemetry_copy_dma_range(s_last_dma_pos,
                                         sizeof(s_dma_buffer),
                                         received_tick_ms) == 0U ||
            esc_telemetry_copy_dma_range(0U,
                                         dma_write_pos,
                                         received_tick_ms) == 0U)
        {
            esc_telemetry_invalidate_locked(
                received_tick_ms,
                ESC_TELEMETRY_INVALIDATION_PENDING_OVERFLOW);
            s_last_dma_pos = dma_write_pos;
            s_dma_cursor_valid = 1U;
            return;
        }
        s_diagnostics.dma_wrap_events++;
    }

    s_last_dma_pos = dma_write_pos;
}

void EscTelemetry_RecordBytes(const uint8_t *data,
                              size_t length,
                              uint32_t received_tick_ms)
{
    EscTelemetryIrqState_t irq_state;
    const uint32_t output_context = EscTelemetry_GetOutputContext();

    if (data == NULL || length == 0U)
    {
        return;
    }

    irq_state = esc_telemetry_enter_critical();
    if (esc_telemetry_push_pending(data,
                                   length,
                                   received_tick_ms,
                                   output_context,
                                   EscTelemetry_ContextIsValid(
                                       output_context)) == 0U)
    {
        esc_telemetry_invalidate_locked(
            received_tick_ms,
            ESC_TELEMETRY_INVALIDATION_PENDING_OVERFLOW);
        esc_telemetry_exit_critical(irq_state);
        return;
    }

    s_diagnostics.bytes_copied_from_receiver += (uint32_t)length;
    esc_telemetry_exit_critical(irq_state);
}

void EscTelemetry_RecordRxError(uint32_t tick_ms, uint32_t error_flags)
{
    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    s_diagnostics.rx_error_count++;
    s_diagnostics.last_rx_error_tick_ms = tick_ms;
    s_diagnostics.last_rx_error_flags = error_flags;
    esc_telemetry_invalidate_locked(
        tick_ms,
        ESC_TELEMETRY_INVALIDATION_RX_ERROR);
    esc_telemetry_exit_critical(irq_state);
}

static uint8_t esc_telemetry_pop_pending(EscTelemetryPendingChunk_t *chunk)
{
    uint8_t popped = 0U;

    if (chunk == NULL)
    {
        return 0U;
    }

    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    if (s_pending_count != 0U)
    {
        *chunk = s_pending_chunks[s_pending_head];
        s_pending_head = (s_pending_head + 1U) % ESC_TELEMETRY_PENDING_CHUNKS;
        s_pending_count--;
        popped = 1U;
    }
    esc_telemetry_exit_critical(irq_state);

    return popped;
}

static void esc_telemetry_record_byte_meta(uint32_t tick_ms,
                                           uint32_t output_context,
                                           uint8_t context_valid)
{
    (void)tick_ms;

    s_recent_output_context[s_recent_byte_meta_next] =
        (context_valid != 0U) ? output_context : 0UL;
    s_recent_byte_meta_next =
        (s_recent_byte_meta_next + 1U) % ESC_FE32_FRAME_LEN;
    if (s_recent_byte_meta_count < ESC_FE32_FRAME_LEN)
    {
        s_recent_byte_meta_count++;
    }
}

static uint8_t esc_telemetry_collect_frame_context(uint32_t *output_context)
{
    uint32_t source = 0UL;
    size_t index;

    if (output_context != NULL)
    {
        *output_context = 0UL;
    }
    if (s_recent_byte_meta_count < ESC_FE32_FRAME_LEN)
    {
        return 0U;
    }

    for (index = 0U; index < ESC_FE32_FRAME_LEN; ++index)
    {
        const uint32_t context = s_recent_output_context[index];
        const uint32_t context_source = EscTelemetry_ContextSource(context);

        if (context_source == 0UL)
        {
            return 0U;
        }
        if (source == 0UL)
        {
            source = context_source;
        }
        else if (source != context_source)
        {
            return 0U;
        }
    }

    if (output_context != NULL)
    {
        *output_context = s_recent_output_context[s_recent_byte_meta_next];
    }
    return 1U;
}

static void esc_telemetry_enqueue_observed_locked(
    const EscFe32Sample_t *sample,
    uint32_t receive_epoch,
    uint32_t output_context)
{
    EscTelemetryObservedSample_t *observed;
    size_t next_depth;

    if (sample == NULL)
    {
        return;
    }

    if (s_observed_count >= ESC_TELEMETRY_OBSERVED_SAMPLE_QUEUE_LEN)
    {
        s_diagnostics.observed_queue_overflows++;
        s_diagnostics.observed_samples_discarded +=
            (uint32_t)s_observed_count;
        esc_telemetry_clear_observed_locked();
        esc_telemetry_advance_delivery_epoch_locked();
    }

    next_depth = s_observed_count + 1U;
    observed = &s_observed_samples[s_observed_tail];
    observed->sample = *sample;
    observed->receive_epoch = receive_epoch;
    observed->delivery_epoch = s_delivery_epoch;
    observed->output_context = output_context;
    s_observed_tail =
        (s_observed_tail + 1U) % ESC_TELEMETRY_OBSERVED_SAMPLE_QUEUE_LEN;
    s_observed_count++;
    s_diagnostics.observed_samples_queued++;
    if (next_depth > s_diagnostics.observed_queue_depth_peak)
    {
        s_diagnostics.observed_queue_depth_peak = (uint32_t)next_depth;
    }
}

static void esc_telemetry_account_observed_context_locked(
    const EscFe32Sample_t *sample,
    uint32_t receive_epoch,
    uint32_t output_context,
    uint8_t context_valid)
{
    const uint32_t source = EscTelemetry_ContextSource(output_context);

    if (context_valid == 0U || source == 0UL)
    {
        s_diagnostics.observed_context_rejected++;
        esc_telemetry_discard_observed_locked();
        esc_telemetry_advance_delivery_epoch_locked();
        return;
    }

    if (s_last_observed_source_valid == 0U)
    {
        s_last_observed_source = source;
        s_last_observed_source_valid = 1U;
    }
    else if (s_last_observed_source != source)
    {
        s_diagnostics.observed_generation_boundaries++;
        esc_telemetry_discard_observed_locked();
        esc_telemetry_advance_delivery_epoch_locked();
        s_last_observed_source = source;
        s_last_observed_source_valid = 1U;
    }

    if (EscTelemetry_ContextRcActive(output_context) == 0U)
    {
        s_diagnostics.observed_samples_discarded++;
        return;
    }

    esc_telemetry_enqueue_observed_locked(sample,
                                          receive_epoch,
                                          output_context);
}

static void esc_telemetry_publish_sample(const EscFe32Sample_t *sample,
                                         uint32_t receive_epoch,
                                         uint32_t output_context,
                                         uint8_t context_valid)
{
    if (sample == NULL)
    {
        return;
    }

    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    if (receive_epoch == s_receive_epoch)
    {
        s_snapshot.sample = *sample;
        s_snapshot.has_sample = 1U;
        s_snapshot.receive_epoch = receive_epoch;
        s_snapshot.publish_sequence++;
        s_diagnostics.samples_published++;
        esc_telemetry_account_observed_context_locked(sample,
                                                      receive_epoch,
                                                      output_context,
                                                      context_valid);
    }
    else
    {
        s_diagnostics.stale_samples_suppressed++;
    }
    esc_telemetry_exit_critical(irq_state);
}

static uint8_t esc_telemetry_process_received_byte(uint8_t byte,
                                                   uint32_t received_tick_ms,
                                                   uint32_t output_context,
                                                   uint8_t context_valid)
{
    EscFe32Sample_t sample;
    size_t emitted;
    uint32_t epoch_before;
    uint32_t epoch_after;
    uint32_t frame_context = 0UL;
    uint8_t frame_context_valid;

    esc_telemetry_reset_parser_if_requested();
    epoch_before = esc_telemetry_get_epoch();
    esc_telemetry_record_byte_meta(received_tick_ms,
                                   output_context,
                                   context_valid);
    emitted = EscFe32Parser_PushBytes(&s_parser,
                                      &byte,
                                      1U,
                                      received_tick_ms,
                                      &sample,
                                      1U);
    epoch_after = esc_telemetry_get_epoch();
    if (epoch_after != epoch_before)
    {
        EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

        s_diagnostics.stale_samples_suppressed += (uint32_t)emitted;
        esc_telemetry_exit_critical(irq_state);
        esc_telemetry_reset_parser_if_requested();
        return 1U;
    }

    if (emitted == 0U)
    {
        return 0U;
    }

    frame_context_valid =
        esc_telemetry_collect_frame_context(&frame_context);
    esc_telemetry_publish_sample(&sample,
                                 epoch_after,
                                 frame_context,
                                 frame_context_valid);
    return 1U;
}

uint8_t EscTelemetry_ProcessReceivedByte(uint8_t byte,
                                         uint32_t received_tick_ms,
                                         uint32_t output_context)
{
    const uint8_t context_valid =
        EscTelemetry_ContextIsValid(output_context);

    return esc_telemetry_process_received_byte(byte,
                                               received_tick_ms,
                                               output_context,
                                               context_valid);
}

void EscTelemetry_ProcessPending(void)
{
    EscTelemetryPendingChunk_t chunk;

    esc_telemetry_reset_parser_if_requested();

    while (esc_telemetry_pop_pending(&chunk) != 0U)
    {
        size_t index;
        uint32_t epoch_before;
        uint32_t epoch_after;

        esc_telemetry_reset_parser_if_requested();
        epoch_before = esc_telemetry_get_epoch();
        if (chunk.receive_epoch != epoch_before)
        {
            EscTelemetryIrqState_t irq_state =
                esc_telemetry_enter_critical();

            s_diagnostics.stale_chunks_discarded++;
            esc_telemetry_exit_critical(irq_state);
            continue;
        }

        for (index = 0U; index < chunk.length; ++index)
        {
            (void)esc_telemetry_process_received_byte(
                chunk.data[index],
                chunk.received_tick_ms,
                chunk.output_context,
                chunk.output_context_valid);
            epoch_after = esc_telemetry_get_epoch();
            if (epoch_after != epoch_before ||
                chunk.receive_epoch != epoch_after)
            {
                EscTelemetryIrqState_t irq_state =
                    esc_telemetry_enter_critical();

                s_diagnostics.stale_chunks_discarded++;
                esc_telemetry_exit_critical(irq_state);
                esc_telemetry_reset_parser_if_requested();
                break;
            }
        }
    }

    esc_telemetry_reset_parser_if_requested();

    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    esc_telemetry_sync_parser_diagnostics();
    esc_telemetry_exit_critical(irq_state);
}

uint32_t EscTelemetry_PublishOutputContext(uint16_t pwm_us,
                                           uint8_t rc_active)
{
    uint32_t generation;
    uint32_t context;
    const uint8_t rc_flag = (rc_active != 0U) ? 1U : 0U;
    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    if (s_output_context_generation_valid == 0U)
    {
        s_output_context_generation = 1UL;
        s_output_context_generation_valid = 1U;
    }
    else if (s_output_context_rc_active != rc_flag)
    {
        s_output_context_generation++;
        if (s_output_context_generation >=
            (1UL << (32U - ESC_TELEMETRY_CONTEXT_SOURCE_SHIFT)))
        {
            s_output_context_generation = 1UL;
        }
    }

    s_output_context_rc_active = rc_flag;
    generation = s_output_context_generation;
    context = ((uint32_t)pwm_us & ESC_TELEMETRY_CONTEXT_PWM_MASK) |
        (rc_flag != 0U ? ESC_TELEMETRY_CONTEXT_RC_ACTIVE_MASK : 0UL) |
        (generation << ESC_TELEMETRY_CONTEXT_SOURCE_SHIFT);
    s_current_output_context = context;
    s_diagnostics.output_context_updates++;
    s_diagnostics.output_context_generation = generation;
    esc_telemetry_exit_critical(irq_state);
    return context;
}

uint32_t EscTelemetry_GetOutputContext(void)
{
    return s_current_output_context;
}

size_t EscTelemetry_PendingSamples(void)
{
    size_t pending;
    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    pending = s_observed_count;
    esc_telemetry_exit_critical(irq_state);
    return pending;
}

uint8_t EscTelemetry_PopSample(EscTelemetryObservedSample_t *sample)
{
    uint8_t popped = 0U;
    EscTelemetryIrqState_t irq_state;

    if (sample == NULL)
    {
        return 0U;
    }

    irq_state = esc_telemetry_enter_critical();
    if (s_observed_count != 0U)
    {
        *sample = s_observed_samples[s_observed_head];
        s_observed_head =
            (s_observed_head + 1U) % ESC_TELEMETRY_OBSERVED_SAMPLE_QUEUE_LEN;
        s_observed_count--;
        s_diagnostics.observed_samples_consumed++;
        popped = 1U;
    }
    esc_telemetry_exit_critical(irq_state);
    return popped;
}

void EscTelemetry_DiscardSamples(void)
{
    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    if (s_observed_count != 0U)
    {
        esc_telemetry_discard_observed_locked();
        esc_telemetry_advance_delivery_epoch_locked();
    }
    esc_telemetry_exit_critical(irq_state);
}

uint32_t EscTelemetry_GetDeliveryEpoch(void)
{
    uint32_t epoch;
    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    epoch = s_delivery_epoch;
    esc_telemetry_exit_critical(irq_state);
    return epoch;
}

uint8_t EscTelemetry_GetSnapshot(EscTelemetrySnapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return 0U;
    }

    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    *snapshot = s_snapshot;
    esc_telemetry_exit_critical(irq_state);

    return snapshot->has_sample;
}

uint8_t EscTelemetry_GetFreshSample(uint32_t now_tick_ms,
                                    uint32_t timeout_ms,
                                    EscFe32Sample_t *sample)
{
    EscTelemetrySnapshot_t snapshot;
    uint32_t age_ms;

    if (timeout_ms == 0U || EscTelemetry_GetSnapshot(&snapshot) == 0U)
    {
        return 0U;
    }

    age_ms = now_tick_ms - snapshot.sample.received_tick_ms;
    if (age_ms > timeout_ms)
    {
        return 0U;
    }

    if (sample != NULL)
    {
        *sample = snapshot.sample;
    }
    return 1U;
}

void EscTelemetry_GetDiagnostics(EscTelemetryDiagnostics_t *diagnostics)
{
    if (diagnostics == NULL)
    {
        return;
    }

    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    esc_telemetry_sync_parser_diagnostics();
    *diagnostics = s_diagnostics;
    esc_telemetry_exit_critical(irq_state);
}

void EscTelemetry_GetReceiverHealth(EscTelemetryReceiverHealth_t *health)
{
    EscTelemetryIrqState_t irq_state;

    if (health == NULL)
    {
        return;
    }

    irq_state = esc_telemetry_enter_critical();
    health->samples_published = s_diagnostics.samples_published;
    health->rx_error_count = s_diagnostics.rx_error_count;
    health->last_rx_error_flags = s_diagnostics.last_rx_error_flags;
    esc_telemetry_exit_critical(irq_state);
}
