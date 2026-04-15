#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define PILOT_SEQUENCE 0xf123456789abcde0

#define BUFSAMPLES  1000

static int dsd_extract_run (void *extract_context, int32_t *dst_buffer, int32_t *src_buffer, int nsamples);
static void dsd_extract_destroy (void *extract_context);
static void *dsd_extract_init (int nchans);

int main (int argc, char **argv)
{
    int total_dsd_samples = 0, total_pcm_samples = 0, valid_dsd_samples = 0;
    int32_t *src_buffer, *dst_buffer;
    int index = 0, nchans = 2, ch;
    void *dsd_extractor;

    if (argc == 1) {
        fprintf (stderr, "Convert raw 24-bit DXD-e to raw DSD via DSD extraction\n");
        fprintf (stderr, "Usage: extract-dsd <nchans> < 24bit-dxd.raw > 1bit-dsd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required)\n");
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

    src_buffer = calloc (sizeof (int32_t), BUFSAMPLES * nchans);
    dst_buffer = calloc (sizeof (int32_t), BUFSAMPLES * nchans);
    dsd_extractor = dsd_extract_init (nchans);

    while ((ch = getchar()) != EOF) {
        src_buffer [index] = ch & 0xff;
        src_buffer [index] |= (getchar() & 0xff) << 8;
        src_buffer [index] |= (getchar() & 0xff) << 16;
        src_buffer [index] = (src_buffer [index] << 8) >> 8;
        total_pcm_samples++;
        index++;

        if (index == BUFSAMPLES * nchans) {
            memcpy (dst_buffer, src_buffer, index * sizeof (dst_buffer [0]));
            valid_dsd_samples += dsd_extract_run (dsd_extractor, dst_buffer, src_buffer, index / nchans);

            for (int i = 0; i < index; ++i) {
                putchar (dst_buffer [i]);
                total_dsd_samples++;
            }

            index = 0;
        }
    }

    if (index) {
        memcpy (dst_buffer, src_buffer, index * sizeof (dst_buffer [0]));
        valid_dsd_samples += dsd_extract_run (dsd_extractor, dst_buffer, src_buffer, index / nchans);

        for (int i = 0; i < index; ++i) {
            putchar (dst_buffer [i]);
            total_dsd_samples++;
        }
    }

    fprintf (stderr, "%d total PCM samples, %d total DSD samples, %d were valid\n", total_pcm_samples, total_dsd_samples, valid_dsd_samples);

    dsd_extract_destroy (dsd_extractor);
    free (src_buffer);
    free (dst_buffer);

    return 0;
}

/*------------------------------------------------------------------------------------------------------------------------*/

typedef struct {
    uint64_t parity_masks [64];
    int nchans, *parity_index;
} ExtractContext;

static void *dsd_extract_init (int nchans)
{
    ExtractContext *context = (ExtractContext *) calloc (1, sizeof (ExtractContext));
    uint64_t shifter = PILOT_SEQUENCE;

    context->nchans = nchans;
    context->parity_index = calloc (sizeof (int), nchans);

    for (int i = 0; i < nchans; ++i)
        context->parity_index [i] = -1;

    for (int i = 0; i < 64; ++i) {
        context->parity_masks [i] = shifter;
        shifter = (shifter << 1) | ((shifter >> 63) & 1);
    }

    if (shifter != PILOT_SEQUENCE) {
        fprintf (stderr, "something's wrong with the pilot sequence!\n");
        exit (1);
    }

    return context;
}

static uint64_t generate_parities (int32_t *src_buffer, int nsamples, int stride);

static int dsd_extract_run (void *extract_context, int32_t *dst_buffer, int32_t *src_buffer, int nsamples)
{
    ExtractContext *context = (ExtractContext *) extract_context;
    int valid_dsd_samples = 0;

    for (int index = 0; index < nsamples; index += 64) {
        uint64_t mask = 0xffffffffffffffff;
        int bsamples = 64;

        if (index + bsamples > nsamples) {
            bsamples = nsamples - index;
            mask <<= 64 - bsamples;
        }

        for (int chan = 0; chan < context->nchans; ++chan) {
            uint64_t parities = generate_parities (src_buffer + chan, bsamples, context->nchans);

            if (context->parity_index [chan] < 0) {
                for (int i = 0; i < 64; ++i)
                    if (parities == (context->parity_masks [i] & mask)) {   // we really don't want to do this if less than 64 bits...
                        fprintf (stderr, "chan %d, index %d, matched mask %d\n", chan, index, i);
                        context->parity_index [chan] = i;
                        break;
                    }
            }
            else {
                uint64_t errors = (parities ^ context->parity_masks [context->parity_index [chan]]) & mask;

                if (errors) {
                    int leading_good_samples = __builtin_clzll (errors);
                    int trial;

                    // fprintf (stderr, "\nchan %d, index %d, mask = %d, got errors: %016lx\n", chan, index, context->parity_index [chan],  errors);

                    for (trial = 0; trial < 64; ++trial)
                        if (trial != context->parity_index [chan]) {
                            uint64_t next_errors = (parities ^ context->parity_masks [trial]) & mask;

                            if (!next_errors) {
                                context->parity_index [chan] = trial;
                                break;
                            }

                            int trailing_good_samples = __builtin_ctzll (next_errors);

                            if (leading_good_samples + trailing_good_samples >= 64) {
                                // fprintf (stderr, "chan %d, index %d, new mask = %d, errors: %016lx\n", chan, index, trial, next_errors);
                                context->parity_index [chan] = trial;
                                break;
                            }
                        }

                    if (trial == 64) {
                        fprintf (stderr, "chan %d, index %d, failure to continue\n", chan, index);
                        context->parity_index [chan] = -1;
                    }
                }
            }

            if (0 || context->parity_index [chan] >= 0)
                for (int i = chan; i < bsamples * context->nchans + chan; i += context->nchans) {
                    dst_buffer [i] = src_buffer [i] & 0xff;
                    valid_dsd_samples++;
                }
            else
                for (int i = chan; i < bsamples * context->nchans + chan; i += context->nchans)
                    dst_buffer [i] = 0x69;
        }

        src_buffer += context->nchans * bsamples;
        dst_buffer += context->nchans * bsamples;
    }

    return valid_dsd_samples;
}

static uint64_t generate_parities (int32_t *src_buffer, int nsamples, int stride)
{
    uint64_t parities = 0;

    for (int bc = nsamples; bc--; src_buffer += stride)
        parities = (parities << 1) | (__builtin_parity (*src_buffer) & 1);

    if (nsamples < 64)
        parities <<= 64 - nsamples;

    return parities;
}

static void dsd_extract_destroy (void *extract_context)
{
    free (extract_context);
}
