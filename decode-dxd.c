#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "decoder.h"

static int read_24bit_samples (FILE *file, int32_t *buffer, int nchans, int samples_to_read);

#define BUFFER_SAMPLES  16384

int main (int argc, char **argv)
{
    int64_t total_dsd_samples = 0, total_pcm_samples = 0;
    unsigned char *destin_buffer;
    int nchans = 2, level = 3;
    int32_t *source_buffer;
    Decoder *decoder;

    if (argc == 1) {
        fprintf (stderr, "Convert raw 24-bit DXD-e to raw DSD via DSD verification/extraction/modulation\n");
        fprintf (stderr, "Usage: decode-dsd <nchans> [level] < 24bit-dxd.raw > 1bit-dsd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required), <level> = 1 to 5 (default = 3)\n");
        fprintf (stderr, " Note: areas with missing or corrupted embedded DSD get newly modulated DSD\n");
        return 0;
    }

    if (argc > 1) {
        nchans = atoi (argv [1]);

        if (nchans < 1 || nchans > 16) {
            fprintf (stderr, "must be 1 to 16 channels!\n");
            return 1;
        }
    }

    if (argc > 2) {
        level = atoi (argv [2]);

        if (level < 1 || level > 5) {
            fprintf (stderr, "level must be 1 to 5!\n");
            return 1;
        }
    }

    decoder = decodeInit (nchans, level);
    source_buffer = calloc (sizeof (int32_t), BUFFER_SAMPLES * nchans);
    destin_buffer = calloc (1, BUFFER_SAMPLES * nchans);

    uint32_t random = 0x31415926;

    while (1) {
        int buffer_samples = BUFFER_SAMPLES;

        do {
            buffer_samples = random >> 18;
            random = ((random << 4) - random) ^ 1;
            random = ((random << 4) - random) ^ 1;
            random = ((random << 4) - random) ^ 1;
        } while (buffer_samples < 10 || buffer_samples > BUFFER_SAMPLES);

        int samples_read = read_24bit_samples (stdin, source_buffer, nchans, buffer_samples);
        int samples_generated = decodeProcess (decoder, source_buffer, samples_read ? samples_read : -1, destin_buffer, buffer_samples);

        total_pcm_samples += samples_read;
        total_dsd_samples += samples_generated;

        fwrite (destin_buffer, nchans, samples_generated, stdout);

        if (!samples_read && !samples_generated)
            break;

    }

    fprintf (stderr, "%ld total PCM samples, %ld total DSD samples, %ld were embedded (all chans, %.1f%%)\n",
        total_pcm_samples, total_dsd_samples, decodeTotalEmbeddedSamples (decoder),
        decodeTotalEmbeddedSamples (decoder) * 100.0 / total_dsd_samples / nchans);

    dsd_transition_dumpstats (stderr);

    decodeFree (decoder);
    free (source_buffer);
    free (destin_buffer);

    return 0;
}

static int read_24bit_samples (FILE *file, int32_t *buffer, int nchans, int samples_to_read)
{
    int samples_read = 0;

    while (samples_to_read--) {
        unsigned char raw_buffer [nchans * 3], *raw_ptr = raw_buffer;
        int t;

        if ((t = fread (raw_buffer, 3, nchans, file)) != nchans) {
            return samples_read;
        }

        for (int c = 0; c < nchans; ++c) {
            int32_t sample24 = *raw_ptr++ << 8;

            sample24 += *raw_ptr++ << 16;
            sample24 += *raw_ptr++ << 24;
            *buffer++ = sample24 >> 8;
        }

        samples_read++;
    }

    return samples_read;
}
