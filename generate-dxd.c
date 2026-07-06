#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sys/random.h>

#include "dsd-utils.h"

#define BUFSAMPLES  65536

int main (int argc, char **argv)
{
    int64_t total_dsd_samples = 0, total_pcm_samples = 0;
    int source_head = 0, decimate_tail = 0, embed_tail = 0;
    int nchans = 2, embed_dsd = 0, flags = 0, filter = 0;
    EmbedContext *dsd_embedder = NULL;
    void *dsd_decimator;
    unsigned char *src_buffer;
    int32_t *dst_buffer;

    if (argc == 1) {
        fprintf (stderr, "Convert raw DSD to raw 24-bit DXD via 8x decimation, embed source DSD\n");
        fprintf (stderr, "Usage: generate-dxd <nchans> [E|e] [P|p] [S|s] [F|f] < 1bit-dsd.raw > 24bit-dxd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required)\n");
        fprintf (stderr, "       [E|e] to embed source DSD\n");
        fprintf (stderr, "       [P|p] to add pilot signal (unique every run for production)\n");
        fprintf (stderr, "       [S|s] to add pilot signal (same every run for testing)\n");
        fprintf (stderr, "       [F|f] for lowpass filter\n");
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
            if (strlen (argv [argi]) == 1 && (*argv [argi] == 'e' || *argv [argi] == 'E'))
                embed_dsd = 1;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'p' || *argv [argi] == 'P'))
                flags |= EMBED_PILOT_SIGNAL | EMBED_PILOT_UNIQUE;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 's' || *argv [argi] == 'S')) {
                flags |= EMBED_PILOT_SIGNAL;
                flags &= ~EMBED_PILOT_UNIQUE;
            }
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'f' || *argv [argi] == 'F'))
                filter = 1;
            else {
                fprintf (stderr, "unknown argument: %s\n", argv [argi]);
                return 1;
            }
        }

    if ((flags & EMBED_PILOT_SIGNAL) && !embed_dsd) {
        fprintf (stderr, "doesn't make sense to add pilot signal without also embedding DSD!!\n");
        return 1;
    }

    src_buffer = calloc (sizeof (unsigned char), BUFSAMPLES * nchans);
    dst_buffer = calloc (sizeof (int32_t), BUFSAMPLES * nchans);
    dsd_decimator = decimate_dsd_init (nchans, filter ? DECIMATE_LOWPASS : 0);

    if (embed_dsd)
        dsd_embedder = dsd_embed_init (nchans, flags);

    while (1) {
        int samples_read = fread (src_buffer + source_head * nchans, nchans, BUFSAMPLES - source_head, stdin);
        int decimated_samples;

        if (samples_read < BUFSAMPLES - source_head)
            fprintf (stderr, "read only %d samples\n", samples_read);

        total_dsd_samples += samples_read;
        source_head += samples_read;

        if (samples_read) {
            decimated_samples = decimate_dsd_run (dsd_decimator, src_buffer + decimate_tail * nchans, source_head - decimate_tail, dst_buffer);
            decimate_tail = source_head;
        }
        else
            decimated_samples = decimate_dsd_run (dsd_decimator, NULL, -1, dst_buffer);

        if (dsd_embedder) {
            dsd_embed_run (dsd_embedder, dst_buffer, src_buffer, decimated_samples);
            embed_tail += decimated_samples;
        }
        else
            embed_tail = source_head;

        for (int i = 0; i < decimated_samples * nchans; ++i) {
            putchar (dst_buffer [i] & 0xff);
            putchar ((dst_buffer [i] >> 8) & 0xff);
            putchar ((dst_buffer [i] >> 16) & 0xff);
        }

        total_pcm_samples += decimated_samples;

        if (!samples_read)
            break;

        if (embed_tail < source_head)
            memmove (src_buffer, src_buffer + embed_tail * nchans, (source_head - embed_tail) * nchans);

        source_head -= embed_tail;
        decimate_tail -= embed_tail;
        embed_tail -= embed_tail;
    }

    if (dsd_embedder)
        fprintf (stderr, "%ld total DSD samples, %ld total PCM samples with embedded %s\n",
            total_dsd_samples, total_pcm_samples, (flags & EMBED_PILOT_SIGNAL) ? "DSD and pilot signal" : "raw DSD only");
    else
        fprintf (stderr, "%ld total DSD samples, %ld total PCM samples (without embedded DSD)\n", total_dsd_samples, total_pcm_samples);

    if (dsd_embedder)
        dsd_embed_destroy (dsd_embedder);

    decimate_dsd_destroy (dsd_decimator);
    free (src_buffer);
    free (dst_buffer);

    return 0;
}
