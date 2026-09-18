#include "esc_fe32_parser.h"

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

int main(int argc, char **argv)
{
    static const size_t chunk_pattern[] = {1U, 7U, 31U, 32U, 33U, 5U, 64U, 3U};
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
    size_t pattern_index = 0U;

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
        uint8_t buffer[64];
        const size_t want = chunk_pattern[pattern_index %
            (sizeof(chunk_pattern) / sizeof(chunk_pattern[0]))];
        const size_t got = fread(buffer, 1U, want, stream);
        size_t emitted;

        if (got == 0U)
        {
            break;
        }

        tick_ms += 10U;
        emitted = EscFe32Parser_PushBytes(&parser, buffer, got, tick_ms,
                                          samples,
                                          sizeof(samples) / sizeof(samples[0]));
        total_frames += (unsigned long)emitted;
        total_bytes += (unsigned long)got;
        pattern_index++;
    }

    fclose(stream);

    EXPECT_TRUE(total_bytes == expected_bytes);
    EXPECT_TRUE(total_frames == expected_frames);
    EXPECT_TRUE(parser.decoded_frames == expected_frames);
    EXPECT_TRUE(parser.crc_failed_frames == expected_crc_failures);
    EXPECT_TRUE(parser.prefix_rejected_candidates == expected_prefix_rejections);
    EXPECT_TRUE(parser.output_overrun_frames == 0U);
    EXPECT_TRUE(parser.next_sample_id == (uint32_t)(expected_frames + 1UL));

    printf("replayed %lu bytes, decoded %lu FE32 frames, crc_failures=%lu, prefix_rejections=%lu\n",
           total_bytes, total_frames, (unsigned long)parser.crc_failed_frames,
           (unsigned long)parser.prefix_rejected_candidates);
    return 0;
}
