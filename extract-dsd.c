#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "dsd-utils.h"

#define BUFSAMPLES  4704

static int read_24bit_samples (FILE *file, int32_t *buffer, int nchans, int samples_to_read);

int main (int argc, char **argv)
{
    int total_dsd_samples = 0, total_pcm_samples = 0, valid_dsd_samples = 0;
    PilotDetect *pilot_detector;
    int nchans = 2, force = 0;
    unsigned char *dst_buffer;
    int32_t *src_buffer;

    if (argc == 1) {
        fprintf (stderr, "Convert raw 24-bit DXD-e to raw DSD via DSD verification/extraction\n");
        fprintf (stderr, "Usage: extract-dsd <nchans> [F|f] < 24bit-dxd.raw > 1bit-dsd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required)\n");
        fprintf (stderr, "       [F|f] to force extract even without pilot signal (for testing)\n");
        fprintf (stderr, " Note: areas with missing or corrupted DSD get silence (0x69)\n");
        return 0;
    }

    if (argc > 1) {
        nchans = atoi (argv [1]);

        if (nchans < 1 || nchans > 16) {
            fprintf (stderr, "must be 1 to 16 channels!\n");
            return 1;
        }
    }

    if (argc > 2)
        for (int argi = 2; argi < argc; ++argi) {
            if (strlen (argv [argi]) == 1 && (*argv [argi] == 'f' || *argv [argi] == 'F'))
                force = 1;
            else {
                fprintf (stderr, "unknown argument: %s\n", argv [argi]);
                return 1;
            }
        }

    src_buffer = calloc (sizeof (int32_t), BUFSAMPLES * nchans);
    dst_buffer = calloc (sizeof (char), BUFSAMPLES * nchans);
    pilot_detector = pilotDetectInit (nchans);

    while (1) {
        int samples_read = read_24bit_samples (stdin, src_buffer, nchans, BUFSAMPLES);

        if (!samples_read)
            break;

        total_pcm_samples += samples_read * nchans;

        for (int c = 0; c < nchans; ++c) {
            int dsd_valid = force ? 1 : pilotDetectChannelRun (pilot_detector, src_buffer, c, samples_read);

            for (int i = 0; i < samples_read; ++i)
                dst_buffer [(i * nchans) + c] = dsd_valid ? src_buffer [(i * nchans) + c] & 0xff : 0x69;

            if (dsd_valid)
                valid_dsd_samples += samples_read;
        }

        fwrite (dst_buffer, nchans, samples_read, stdout);
        total_dsd_samples += samples_read * nchans;
    }

    fprintf (stderr, "%d total PCM samples, %d total DSD samples, %d were%svalid\n",
        total_pcm_samples, total_dsd_samples, valid_dsd_samples, force ? " forced " : " ");

    pilotDetectDestroy (pilot_detector);
    free (src_buffer);
    free (dst_buffer);

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
