#include "esc_telemetry.h"

#include <string.h>

#if defined(STM32F407xx)
#include "stm32f4xx.h"
typedef uint32_t EscTelemetryIrqState_t;

static EscTelemetryIrqState_t esc_telemetry_enter_critical(void)
{
    EscTelemetryIrqState_t state = __get_PRIMASK();

    __disable_irq();
    return state;
}

static void esc_telemetry_exit_critical(EscTelemetryIrqState_t state)
{
    __set_PRIMASK(state);
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
static uint32_t s_receive_epoch;
static uint8_t s_parser_reset_pending;

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
    s_pending_head = 0U;
    s_pending_tail = 0U;
    s_pending_count = 0U;
    s_last_dma_pos = 0U;
    s_dma_cursor_valid = 1U;
    s_receive_epoch = 1U;
    s_parser_reset_pending = 0U;
    EscFe32Parser_Init(&s_parser);
    s_snapshot.receive_epoch = s_receive_epoch;
    s_diagnostics.receive_epoch = s_receive_epoch;
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
                                          uint32_t tick_ms)
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

    if (length == 0U)
    {
        return 1U;
    }

    if (esc_telemetry_push_pending(&s_dma_buffer[start],
                                   length,
                                   tick_ms) == 0U)
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

    if (data == NULL || length == 0U)
    {
        return;
    }

    irq_state = esc_telemetry_enter_critical();
    if (esc_telemetry_push_pending(data, length, received_tick_ms) == 0U)
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

static void esc_telemetry_publish_sample(const EscFe32Sample_t *sample,
                                         uint32_t receive_epoch)
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
    }
    else
    {
        s_diagnostics.stale_samples_suppressed++;
    }
    esc_telemetry_exit_critical(irq_state);
}

void EscTelemetry_ProcessPending(void)
{
    EscTelemetryPendingChunk_t chunk;

    esc_telemetry_reset_parser_if_requested();

    while (esc_telemetry_pop_pending(&chunk) != 0U)
    {
        EscFe32Sample_t samples[4];
        size_t emitted;
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

        emitted = EscFe32Parser_PushBytes(&s_parser,
                                          chunk.data,
                                          chunk.length,
                                          chunk.received_tick_ms,
                                          samples,
                                          sizeof(samples) / sizeof(samples[0]));
        epoch_after = esc_telemetry_get_epoch();
        if (epoch_after != epoch_before || chunk.receive_epoch != epoch_after)
        {
            EscTelemetryIrqState_t irq_state =
                esc_telemetry_enter_critical();

            s_diagnostics.stale_chunks_discarded++;
            s_diagnostics.stale_samples_suppressed += (uint32_t)emitted;
            esc_telemetry_exit_critical(irq_state);
            esc_telemetry_reset_parser_if_requested();
            continue;
        }

        for (index = 0U; index < emitted; ++index)
        {
            esc_telemetry_publish_sample(&samples[index], epoch_after);
        }
    }

    esc_telemetry_reset_parser_if_requested();

    EscTelemetryIrqState_t irq_state = esc_telemetry_enter_critical();

    esc_telemetry_sync_parser_diagnostics();
    esc_telemetry_exit_critical(irq_state);
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
