#include "esc_fe32_parser.h"
#include "esc_soft_uart_rx.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#define EXPECT_TRUE(condition) \
    do { \
        if (!(condition)) { \
            fprintf(stderr, "%s:%d: expectation failed: %s\n", \
                    __FILE__, __LINE__, #condition); \
            return 1; \
        } \
    } while (0)

static unsigned long parse_ulong(const char *text)
{
    char *end = NULL;
    const unsigned long value = strtoul(text, &end, 10);
    if (text == NULL || *text == '\0' || end == NULL || *end != '\0')
    {
        return 0UL;
    }
    return value;
}

static uint8_t decode_uart_byte(uint8_t raw_byte, uint8_t *decoded_byte)
{
    EscSoftUartRxCore_t core;
    EscSoftUartRxSampleResult_t result;
    uint8_t index;

    if (decoded_byte == NULL)
    {
        return 0U;
    }

    EscSoftUartRxCore_Init(&core);
    EscSoftUartRxCore_Start(&core);
    result = EscSoftUartRxCore_Sample(&core, 0U);
    if (result.status != ESC_SOFT_UART_RX_SAMPLE_CONTINUE)
    {
        return 0U;
    }

    for (index = 0U; index < 8U; index++)
    {
        const uint8_t level =
            ((raw_byte & (uint8_t)(1U << index)) != 0U) ? 1U : 0U;
        result = EscSoftUartRxCore_Sample(&core, level);
        if (result.status != ESC_SOFT_UART_RX_SAMPLE_CONTINUE)
        {
            return 0U;
        }
    }

    result = EscSoftUartRxCore_Sample(&core, 1U);
    if (result.status != ESC_SOFT_UART_RX_SAMPLE_BYTE)
    {
        return 0U;
    }

    *decoded_byte = result.byte;
    return 1U;
}

int main(int argc, char **argv)
{
    EscFe32Parser_t parser;
    EscFe32Sample_t samples[8];
    FILE *stream;
    unsigned long expected_frames;
    unsigned long expected_crc_failures;
    unsigned long expected_prefix_rejections;
    unsigned long expected_bytes;
    unsigned long total_bytes = 0UL;
    unsigned long total_frames = 0UL;
    uint32_t tick_ms = 0U;

    if (argc != 6)
    {
        fprintf(stderr,
                "usage: %s raw.bin expected_bytes expected_frames expected_crc_failures expected_prefix_rejections\n",
                argv[0]);
        return 2;
    }

    expected_bytes = parse_ulong(argv[2]);
    expected_frames = parse_ulong(argv[3]);
    expected_crc_failures = parse_ulong(argv[4]);
    expected_prefix_rejections = parse_ulong(argv[5]);
    stream = fopen(argv[1], "rb");
    if (stream == NULL)
    {
        fprintf(stderr, "failed to open %s: errno=%d\n", argv[1], errno);
        return 2;
    }

    EscFe32Parser_Init(&parser);
    for (;;)
    {
        uint8_t raw_byte;
        uint8_t decoded_byte;
        size_t emitted;
        const int value = fgetc(stream);

        if (value == EOF)
        {
            break;
        }
        raw_byte = (uint8_t)value;
        EXPECT_TRUE(decode_uart_byte(raw_byte, &decoded_byte) != 0U);
        EXPECT_TRUE(decoded_byte == raw_byte);

        tick_ms++;
        emitted = EscFe32Parser_PushBytes(&parser, &decoded_byte, 1U, tick_ms,
                                          samples,
                                          sizeof(samples) / sizeof(samples[0]));
        total_frames += (unsigned long)emitted;
        total_bytes++;
    }

    fclose(stream);

    EXPECT_TRUE(total_bytes == expected_bytes);
    EXPECT_TRUE(total_frames == expected_frames);
    EXPECT_TRUE(parser.decoded_frames == expected_frames);
    EXPECT_TRUE(parser.crc_failed_frames == expected_crc_failures);
    EXPECT_TRUE(parser.prefix_rejected_candidates == expected_prefix_rejections);
    EXPECT_TRUE(parser.output_overrun_frames == 0U);
    EXPECT_TRUE(parser.next_sample_id == (uint32_t)(expected_frames + 1UL));

    printf("soft-uart replayed %lu bytes, decoded %lu FE32 frames, crc_failures=%lu, prefix_rejections=%lu\n",
           total_bytes, total_frames, (unsigned long)parser.crc_failed_frames,
           (unsigned long)parser.prefix_rejected_candidates);
    return 0;
}
