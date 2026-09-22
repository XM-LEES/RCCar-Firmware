#include "esc_telemetry.h"
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

static int feed_dma_bytes(size_t *pos,
                          const uint8_t *data,
                          size_t length,
                          uint32_t tick_ms,
                          EscTelemetryRxEvent_t event)
{
    uint8_t *dma = EscTelemetry_GetDmaBuffer();
    size_t dma_len = EscTelemetry_GetDmaBufferLength();
    size_t index;

    for (index = 0U; index < length; ++index)
    {
        dma[*pos] = data[index];
        *pos = (*pos + 1U) % dma_len;
    }

    EscTelemetry_RecordDmaEvent(*pos, tick_ms, event);
    return 0;
}

static int test_dma_event_dedup_and_snapshot_time(void)
{
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;
    size_t pos = 0U;

    EscTelemetry_Init();
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_PEAK_DYN02, 10U, 100U,
                   ESC_TELEMETRY_RX_EVENT_HALF_TRANSFER);
    EscTelemetry_RecordDmaEvent(pos, 101U, ESC_TELEMETRY_RX_EVENT_IDLE);
    feed_dma_bytes(&pos, &ESC_FE32_FIXTURE_PEAK_DYN02[10],
                   ESC_FE32_FRAME_LEN - 10U, 110U,
                   ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.publish_sequence == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 1U);
    EXPECT_TRUE(snapshot.sample.received_tick_ms == 110U);
    EXPECT_TRUE(snapshot.sample.rpm_raw == 4204U);
    EXPECT_TRUE(snapshot.sample.state_candidate ==
                ESC_FE32_STATE_CANDIDATE_DRIVE_AMBIGUOUS);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.duplicate_dma_events == 1U);
    EXPECT_TRUE(diagnostics.dma_events[ESC_TELEMETRY_RX_EVENT_IDLE] == 2U);
    EXPECT_TRUE(diagnostics.dma_events[ESC_TELEMETRY_RX_EVENT_HALF_TRANSFER] == 1U);
    EXPECT_TRUE(diagnostics.parser_decoded_frames == 1U);
    EXPECT_TRUE(diagnostics.samples_published == 1U);

    return 0;
}

static int test_dma_wrap_and_tick_wrap_freshness(void)
{
    EscFe32Sample_t fresh;
    EscTelemetryDiagnostics_t diagnostics;
    size_t pos = 0U;
    const size_t dma_len = EscTelemetry_GetDmaBufferLength();
    static const uint8_t filler[120] = {0U};

    EscTelemetry_Init();
    feed_dma_bytes(&pos, filler, dma_len - 8U, 50U,
                   ESC_TELEMETRY_RX_EVENT_POLL);
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_BRAKE_DYN02, ESC_FE32_FRAME_LEN,
                   0xFFFFFFF0UL, ESC_TELEMETRY_RX_EVENT_TRANSFER_COMPLETE);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetFreshSample(0x00000010UL, 0x30U, &fresh) == 1U);
    EXPECT_TRUE(fresh.sample_id == 1U);
    EXPECT_TRUE(fresh.received_tick_ms == 0xFFFFFFF0UL);
    EXPECT_TRUE(fresh.state_candidate == ESC_FE32_STATE_CANDIDATE_BRAKE);
    EXPECT_TRUE(EscTelemetry_GetFreshSample(0x00000010UL, 0U, &fresh) == 0U);
    EXPECT_TRUE(EscTelemetry_GetFreshSample(0x00000040UL, 0x20U, &fresh) == 0U);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.dma_wrap_events == 1U);
    EXPECT_TRUE(diagnostics.parser_decoded_frames == 1U);

    return 0;
}

static int test_pending_overflow_and_rx_error_diagnostics(void)
{
    EscTelemetryDiagnostics_t diagnostics;
    uint8_t payload[ESC_TELEMETRY_PENDING_CHUNK_SIZE];
    size_t pos = 0U;
    size_t index;

    memset(payload, 0x55, sizeof(payload));
    EscTelemetry_Init();

    for (index = 0U; index < ESC_TELEMETRY_PENDING_CHUNKS + 2U; ++index)
    {
        feed_dma_bytes(&pos, payload, sizeof(payload), (uint32_t)(200U + index),
                       ESC_TELEMETRY_RX_EVENT_TRANSFER_COMPLETE);
    }
    EscTelemetry_RecordRxError(333U, 0x12UL);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.pending_chunks_dropped != 0U);
    EXPECT_TRUE(diagnostics.pending_bytes_dropped != 0U);
    EXPECT_TRUE(diagnostics.rx_error_count == 1U);
    EXPECT_TRUE(diagnostics.last_rx_error_tick_ms == 333U);
    EXPECT_TRUE(diagnostics.last_rx_error_flags == 0x12UL);

    return 0;
}

static int test_error_boundary_blocks_cross_frame_reassembly(void)
{
    EscFe32Sample_t fresh;
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;
    size_t pos = 0U;

    EscTelemetry_Init();
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_PEAK_DYN02, 16U, 100U,
                   ESC_TELEMETRY_RX_EVENT_HALF_TRANSFER);
    EscTelemetry_RecordRxError(105U, 0x01UL);
    feed_dma_bytes(&pos, &ESC_FE32_FIXTURE_PEAK_DYN02[16],
                   ESC_FE32_FRAME_LEN - 16U, 110U,
                   ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 0U);
    EXPECT_TRUE(EscTelemetry_GetFreshSample(120U, 50U, &fresh) == 0U);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.rx_error_count == 1U);
    EXPECT_TRUE(diagnostics.receive_invalidations == 1U);
    EXPECT_TRUE(diagnostics.last_invalidation_reason ==
                ESC_TELEMETRY_INVALIDATION_RX_ERROR);
    EXPECT_TRUE(diagnostics.parser_decoded_frames == 0U);

    EscTelemetry_ResetDmaCursor();
    pos = 0U;
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_BRAKE_DYN02, ESC_FE32_FRAME_LEN,
                   130U, ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 1U);
    EXPECT_TRUE(snapshot.sample.received_tick_ms == 130U);
    EXPECT_TRUE(snapshot.sample.state_candidate == ESC_FE32_STATE_CANDIDATE_BRAKE);

    return 0;
}

static int test_error_resets_consumed_partial_parser_state(void)
{
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;
    size_t pos = 0U;

    EscTelemetry_Init();
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_PEAK_DYN02, 16U, 300U,
                   ESC_TELEMETRY_RX_EVENT_HALF_TRANSFER);
    EscTelemetry_ProcessPending();
    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 0U);

    EscTelemetry_RecordRxError(305U, 0x02UL);
    feed_dma_bytes(&pos, &ESC_FE32_FIXTURE_PEAK_DYN02[16],
                   ESC_FE32_FRAME_LEN - 16U, 310U,
                   ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 0U);
    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.parser_decoded_frames == 0U);
    EXPECT_TRUE(diagnostics.rx_error_count == 1U);

    EscTelemetry_ResetDmaCursor();
    pos = 0U;
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_BRAKE_DYN02, ESC_FE32_FRAME_LEN,
                   330U, ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 1U);
    EXPECT_TRUE(snapshot.sample.received_tick_ms == 330U);

    return 0;
}

static int test_queued_frame_does_not_publish_after_error(void)
{
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;
    size_t pos = 0U;

    EscTelemetry_Init();
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_PEAK_DYN02, ESC_FE32_FRAME_LEN,
                   200U, ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_RecordRxError(201U, 0x22UL);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 0U);
    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.samples_published == 0U);
    EXPECT_TRUE(diagnostics.parser_decoded_frames == 0U);

    EscTelemetry_ResetDmaCursor();
    pos = 0U;
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_BRAKE_DYN02, ESC_FE32_FRAME_LEN,
                   220U, ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 1U);
    EXPECT_TRUE(snapshot.sample.received_tick_ms == 220U);

    return 0;
}

static int test_overflow_invalidates_snapshot_and_resynchronizes(void)
{
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;
    uint8_t payload[ESC_TELEMETRY_PENDING_CHUNK_SIZE];
    size_t pos = 0U;
    size_t index;

    memset(payload, 0x55, sizeof(payload));
    EscTelemetry_Init();

    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_PEAK_DYN02, ESC_FE32_FRAME_LEN,
                   10U, ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();
    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 1U);

    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_BRAKE_DYN02, 16U, 20U,
                   ESC_TELEMETRY_RX_EVENT_HALF_TRANSFER);
    for (index = 0U; index < ESC_TELEMETRY_PENDING_CHUNKS + 1U; ++index)
    {
        feed_dma_bytes(&pos, payload, sizeof(payload), (uint32_t)(30U + index),
                       ESC_TELEMETRY_RX_EVENT_TRANSFER_COMPLETE);
    }
    feed_dma_bytes(&pos, &ESC_FE32_FIXTURE_BRAKE_DYN02[16],
                   ESC_FE32_FRAME_LEN - 16U, 99U,
                   ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 0U);
    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.pending_chunks_dropped != 0U);
    EXPECT_TRUE(diagnostics.pending_bytes_dropped != 0U);
    EXPECT_TRUE(diagnostics.last_invalidation_reason ==
                ESC_TELEMETRY_INVALIDATION_PENDING_OVERFLOW);

    EscTelemetry_ResetDmaCursor();
    pos = 0U;
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_BRAKE_DYN02, ESC_FE32_FRAME_LEN,
                   130U, ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 2U);
    EXPECT_TRUE(snapshot.sample.received_tick_ms == 130U);

    return 0;
}

static int test_same_position_ht_tc_ambiguity_invalidates(void)
{
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;
    size_t pos = 0U;

    EscTelemetry_Init();
    feed_dma_bytes(&pos, ESC_FE32_FIXTURE_PEAK_DYN02, ESC_FE32_FRAME_LEN,
                   10U, ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_ProcessPending();
    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);

    EscTelemetry_RecordDmaEvent(pos, 20U,
                                ESC_TELEMETRY_RX_EVENT_TRANSFER_COMPLETE);
    EscTelemetry_ProcessPending();
    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 0U);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.ambiguous_dma_events == 1U);
    EXPECT_TRUE(diagnostics.last_invalidation_reason ==
                ESC_TELEMETRY_INVALIDATION_DMA_AMBIGUOUS);

    return 0;
}

static int test_invalid_dma_position_is_counted(void)
{
    EscTelemetryDiagnostics_t diagnostics;

    EscTelemetry_Init();
    EscTelemetry_RecordDmaEvent(EscTelemetry_GetDmaBufferLength() + 1U,
                                500U,
                                ESC_TELEMETRY_RX_EVENT_IDLE);
    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.invalid_dma_positions == 1U);

    return 0;
}

static void feed_direct_bytes(const uint8_t *data,
                              size_t length,
                              uint32_t first_tick_ms,
                              uint32_t output_context)
{
    size_t index;

    for (index = 0U; index < length; ++index)
    {
        (void)EscTelemetry_ProcessReceivedByte(
            data[index],
            first_tick_ms + (uint32_t)index,
            output_context);
    }
}

static int test_rc_observed_sample_keeps_ordered_context(void)
{
    EscTelemetryObservedSample_t observed;
    EscTelemetryDiagnostics_t diagnostics;

    EscTelemetry_Init();
    (void)EscTelemetry_PublishOutputContext(1600U, 1U);
    feed_direct_bytes(ESC_FE32_FIXTURE_BRAKE_DYN02,
                      ESC_FE32_FRAME_LEN,
                      700U,
                      EscTelemetry_GetOutputContext());

    EXPECT_TRUE(EscTelemetry_PendingSamples() == 1U);
    EXPECT_TRUE(EscTelemetry_PopSample(&observed) == 1U);
    EXPECT_TRUE(observed.sample.sample_id == 1U);
    EXPECT_TRUE(observed.sample.received_tick_ms ==
                700U + ESC_FE32_FRAME_LEN - 1U);
    EXPECT_TRUE(EscTelemetry_ContextPwm(observed.output_context) == 1600U);
    EXPECT_TRUE(EscTelemetry_ContextRcActive(observed.output_context) != 0U);
    EXPECT_TRUE(EscTelemetry_ContextSource(observed.output_context) ==
                EscTelemetry_ContextSource(EscTelemetry_GetOutputContext()));

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.observed_samples_queued == 1U);
    EXPECT_TRUE(diagnostics.observed_samples_consumed == 1U);
    EXPECT_TRUE(diagnostics.observed_queue_depth_peak == 1U);

    return 0;
}

static int test_pwm_change_with_same_source_uses_first_byte_context(void)
{
    EscTelemetryObservedSample_t observed;
    uint32_t context;
    size_t index;

    EscTelemetry_Init();
    context = EscTelemetry_PublishOutputContext(1600U, 1U);
    for (index = 0U; index < ESC_FE32_FRAME_LEN / 2U; ++index)
    {
        (void)EscTelemetry_ProcessReceivedByte(
            ESC_FE32_FIXTURE_PEAK_DYN02[index],
            800U + (uint32_t)index,
            context);
    }

    context = EscTelemetry_PublishOutputContext(1700U, 1U);
    for (; index < ESC_FE32_FRAME_LEN; ++index)
    {
        (void)EscTelemetry_ProcessReceivedByte(
            ESC_FE32_FIXTURE_PEAK_DYN02[index],
            800U + (uint32_t)index,
            context);
    }

    EXPECT_TRUE(EscTelemetry_PendingSamples() == 1U);
    EXPECT_TRUE(EscTelemetry_PopSample(&observed) == 1U);
    EXPECT_TRUE(EscTelemetry_ContextPwm(observed.output_context) == 1600U);
    EXPECT_TRUE(EscTelemetry_ContextRcActive(observed.output_context) != 0U);

    return 0;
}

static int test_cross_side_pwm_change_same_source_keeps_first_byte_context(void)
{
    EscTelemetryObservedSample_t observed;
    EscTelemetryDiagnostics_t diagnostics;
    uint32_t initial_epoch;
    uint32_t context;
    size_t index;

    EscTelemetry_Init();
    initial_epoch = EscTelemetry_GetDeliveryEpoch();
    context = EscTelemetry_PublishOutputContext(1600U, 1U);
    for (index = 0U; index < ESC_FE32_FRAME_LEN / 2U; ++index)
    {
        (void)EscTelemetry_ProcessReceivedByte(
            ESC_FE32_FIXTURE_BRAKE_DYN02[index],
            840U + (uint32_t)index,
            context);
    }

    context = EscTelemetry_PublishOutputContext(1400U, 1U);
    for (; index < ESC_FE32_FRAME_LEN; ++index)
    {
        (void)EscTelemetry_ProcessReceivedByte(
            ESC_FE32_FIXTURE_BRAKE_DYN02[index],
            840U + (uint32_t)index,
            context);
    }

    EXPECT_TRUE(EscTelemetry_GetDeliveryEpoch() == initial_epoch);
    EXPECT_TRUE(EscTelemetry_PendingSamples() == 1U);
    EXPECT_TRUE(EscTelemetry_PopSample(&observed) == 1U);
    EXPECT_TRUE(EscTelemetry_ContextPwm(observed.output_context) == 1600U);
    EXPECT_TRUE(EscTelemetry_ContextRcActive(observed.output_context) != 0U);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.observed_context_rejected == 0U);
    EXPECT_TRUE(diagnostics.observed_generation_boundaries == 0U);

    return 0;
}

static int test_source_change_inside_frame_rejects_observed_context(void)
{
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;
    uint32_t context;
    size_t index;

    EscTelemetry_Init();
    context = EscTelemetry_PublishOutputContext(1600U, 1U);
    for (index = 0U; index < ESC_FE32_FRAME_LEN / 2U; ++index)
    {
        (void)EscTelemetry_ProcessReceivedByte(
            ESC_FE32_FIXTURE_PEAK_DYN02[index],
            900U + (uint32_t)index,
            context);
    }

    context = EscTelemetry_PublishOutputContext(1500U, 0U);
    for (; index < ESC_FE32_FRAME_LEN; ++index)
    {
        (void)EscTelemetry_ProcessReceivedByte(
            ESC_FE32_FIXTURE_PEAK_DYN02[index],
            900U + (uint32_t)index,
            context);
    }

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 1U);
    EXPECT_TRUE(EscTelemetry_PendingSamples() == 0U);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.observed_context_rejected == 1U);
    EXPECT_TRUE(diagnostics.samples_published == 1U);

    return 0;
}

static int test_non_rc_frames_do_not_enter_observed_queue(void)
{
    EscTelemetrySnapshot_t snapshot;
    EscTelemetryDiagnostics_t diagnostics;

    EscTelemetry_Init();
    (void)EscTelemetry_PublishOutputContext(1500U, 0U);
    EscTelemetry_RecordBytes(ESC_FE32_FIXTURE_BRAKE_DYN02,
                             ESC_FE32_FRAME_LEN,
                             1000U);
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetSnapshot(&snapshot) == 1U);
    EXPECT_TRUE(snapshot.sample.sample_id == 1U);
    EXPECT_TRUE(EscTelemetry_PendingSamples() == 0U);

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.observed_samples_discarded == 1U);

    return 0;
}

static int test_observed_queue_overflow_starts_new_delivery_epoch(void)
{
    EscTelemetryObservedSample_t observed;
    EscTelemetryDiagnostics_t diagnostics;
    uint32_t initial_epoch;
    size_t index;

    EscTelemetry_Init();
    (void)EscTelemetry_PublishOutputContext(1600U, 1U);
    initial_epoch = EscTelemetry_GetDeliveryEpoch();
    for (index = 0U; index < ESC_TELEMETRY_OBSERVED_SAMPLE_QUEUE_LEN + 1U;
         ++index)
    {
        EscTelemetry_RecordBytes(ESC_FE32_FIXTURE_PEAK_DYN02,
                                 ESC_FE32_FRAME_LEN,
                                 1100U + (uint32_t)(index * 20U));
    }
    EscTelemetry_ProcessPending();

    EXPECT_TRUE(EscTelemetry_GetDeliveryEpoch() != initial_epoch);
    EXPECT_TRUE(EscTelemetry_PendingSamples() == 1U);
    EXPECT_TRUE(EscTelemetry_PopSample(&observed) == 1U);
    EXPECT_TRUE(observed.sample.sample_id ==
                ESC_TELEMETRY_OBSERVED_SAMPLE_QUEUE_LEN + 1U);
    EXPECT_TRUE(observed.delivery_epoch == EscTelemetry_GetDeliveryEpoch());

    EscTelemetry_GetDiagnostics(&diagnostics);
    EXPECT_TRUE(diagnostics.observed_queue_overflows == 1U);
    EXPECT_TRUE(diagnostics.observed_samples_discarded ==
                ESC_TELEMETRY_OBSERVED_SAMPLE_QUEUE_LEN);

    return 0;
}

int main(void)
{
    if (test_dma_event_dedup_and_snapshot_time() != 0)
    {
        return 1;
    }
    if (test_dma_wrap_and_tick_wrap_freshness() != 0)
    {
        return 1;
    }
    if (test_pending_overflow_and_rx_error_diagnostics() != 0)
    {
        return 1;
    }
    if (test_error_boundary_blocks_cross_frame_reassembly() != 0)
    {
        return 1;
    }
    if (test_error_resets_consumed_partial_parser_state() != 0)
    {
        return 1;
    }
    if (test_queued_frame_does_not_publish_after_error() != 0)
    {
        return 1;
    }
    if (test_overflow_invalidates_snapshot_and_resynchronizes() != 0)
    {
        return 1;
    }
    if (test_same_position_ht_tc_ambiguity_invalidates() != 0)
    {
        return 1;
    }
    if (test_invalid_dma_position_is_counted() != 0)
    {
        return 1;
    }
    if (test_rc_observed_sample_keeps_ordered_context() != 0)
    {
        return 1;
    }
    if (test_pwm_change_with_same_source_uses_first_byte_context() != 0)
    {
        return 1;
    }
    if (test_cross_side_pwm_change_same_source_keeps_first_byte_context() != 0)
    {
        return 1;
    }
    if (test_source_change_inside_frame_rejects_observed_context() != 0)
    {
        return 1;
    }
    if (test_non_rc_frames_do_not_enter_observed_queue() != 0)
    {
        return 1;
    }
    if (test_observed_queue_overflow_starts_new_delivery_epoch() != 0)
    {
        return 1;
    }

    return 0;
}
