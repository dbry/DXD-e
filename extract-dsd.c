#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define PILOT_SEQUENCE 0xf123456789abcde0
#define HONOR_PILOT

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
    uint64_t channel_shifter, start_index, stop_index;
    uint32_t parity_shifter;
    char locked;
} ExtractChannel;

typedef struct {
    uint64_t parity_masks [64], sample_index;
    ExtractChannel *chans;
    int nchans;
} ExtractContext;

static void *dsd_extract_init (int nchans)
{
    ExtractContext *context = (ExtractContext *) calloc (1, sizeof (ExtractContext));
    uint64_t shifter = PILOT_SEQUENCE;

    context->nchans = nchans;

    for (int i = 0; i < 64; ++i) {
        context->parity_masks [i] = shifter;
        shifter = (shifter << 1) | ((shifter >> 63) & 1);
    }

    context->chans = calloc (sizeof (ExtractChannel), nchans);

    return context;
}

static int dsd_extract_run (void *extract_context, int32_t *dst_buffer, int32_t *src_buffer, int nsamples)
{
    ExtractContext *context = (ExtractContext *) extract_context;
    int64_t base_index = context->sample_index;
    int valid_dsd_samples = 0;

    for (int index = 0; index < nsamples * context->nchans; ++index)
        dst_buffer [index] = 0x69;

    for (int index = 0; index < nsamples; ++index) {
        for (int c = 0; c < context->nchans; ++c) {
            ExtractChannel *chan = context->chans + c;

            chan->parity_shifter = (chan->parity_shifter << 1) | __builtin_parity (src_buffer [(index * context->nchans) + c]);
            chan->channel_shifter = (chan->channel_shifter << 1) | __builtin_parity (chan->parity_shifter & 0x80002001);

            if (chan->locked) {
                chan->locked = ((chan->locked + 1) & 0x3f) | 0x40;
                if (chan->channel_shifter != context->parity_masks [chan->locked & 0x3f]) {
                    chan->stop_index = context->sample_index - 32;
                    fprintf (stderr, "%d: embedded: %.3f - %.3f\n", c, chan->start_index / 352800.0, chan->stop_index / 352800.0);

                    if (chan->stop_index > base_index) {
                        int start_index = chan->start_index < base_index ? 0 : chan->start_index - base_index;
                        int stop_index = chan->stop_index - base_index;

                        for (int i = start_index; i < stop_index; i++) {
                            dst_buffer [(i * context->nchans) + c] = src_buffer [(i * context->nchans) + c] & 0xff;
                            valid_dsd_samples++;
                        }
                    }

                    chan->locked = 0;
                }
            }
            else
                for (int i = 0; i < 64; ++i)
                    if (chan->channel_shifter == context->parity_masks [i]) {
                        if (context->sample_index > 94)
                            chan->start_index = context->sample_index - 32;
                        else
                            chan->start_index = 0;

                        fprintf (stderr, "%d:   locked: %.3f\n", c, context->sample_index / 352800.0);
                        chan->locked = i | 0x40;
                        break;
                    }
        }

        context->sample_index++;
    }

    for (int c = 0; c < context->nchans; ++c) {
        ExtractChannel *chan = context->chans + c;

        if (chan->locked) {
            int start_index = chan->start_index < base_index ? 0 : chan->start_index - base_index, stop_index = nsamples;

            for (int i = start_index; i < stop_index; i++) {
                dst_buffer [(i * context->nchans) + c] = src_buffer [(i * context->nchans) + c] & 0xff;
                valid_dsd_samples++;
            }
        }
    }

    return valid_dsd_samples;
}

static void dsd_extract_destroy (void *extract_context)
{
    ExtractContext *context = (ExtractContext *) extract_context;

    free (context->chans);
    free (context);
}
