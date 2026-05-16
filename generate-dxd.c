#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <sys/random.h>

#include "decimator.h"
#include "biquad.h"

#define PILOT_SEQUENCE 0xf123456789abcde0

static void dsd_embed_run (void *embed_context, int32_t *dst_buffer, unsigned char *src_buffer, int nsamples);
static void dsd_embed_destroy (void *embed_context);
static void *dsd_embed_init (int nchans);

#define BUFSAMPLES  65536

int main (int argc, char **argv)
{
    int64_t total_dsd_samples = 0, total_pcm_samples = 0;
    int source_head = 0, decimate_tail = 0, embed_tail = 0;
    void *dsd_decimator, *dsd_embedder = NULL;
    int nchans = 2, embed_dsd = 0, filter = 0;
    unsigned char *src_buffer;
    int32_t *dst_buffer;

    if (argc == 1) {
        fprintf (stderr, "Convert raw DSD to raw 24-bit DXD via 8x decimation, embed source DSD\n");
        fprintf (stderr, "Usage: generate-dxd <nchans> [E|e] [F|f] < 1bit-dsd.raw > 24bit-dxd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required)\n");
        fprintf (stderr, "       [E|e] to embed source DSD\n");
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
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'f' || *argv [argi] == 'F'))
                filter = 1;
            else {
                fprintf (stderr, "unknown argument: %s\n", argv [argi]);
                return 1;
            }
        }

    src_buffer = calloc (sizeof (unsigned char), BUFSAMPLES * nchans);
    dst_buffer = calloc (sizeof (int32_t), BUFSAMPLES * nchans);
    dsd_decimator = decimate_dsd_init (nchans, filter ? DECIMATE_LOWPASS : 0);

    if (embed_dsd)
        dsd_embedder = dsd_embed_init (nchans);

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
        fprintf (stderr, "%ld total DSD samples, %ld total PCM samples (with embedded DSD)\n", total_dsd_samples, total_pcm_samples);
    else
        fprintf (stderr, "%ld total DSD samples, %ld total PCM samples (without embedded DSD)\n", total_dsd_samples, total_pcm_samples);

    if (dsd_embedder)
        dsd_embed_destroy (dsd_embedder);

    decimate_dsd_destroy (dsd_decimator);
    free (src_buffer);
    free (dst_buffer);

    return 0;
}

/*------------------------------------------------------------------------------------------------------------------------*/

typedef struct {
    uint32_t *parity_shifters;
    Biquad *noise_shapers;
    float *noise_feedback;
    int64_t sample_index;
    int nchans;
} EmbedContext;

static void shaper_init (Biquad *f, double a0, double a1, double a2, double a3, double a4, double b1, double b2, double b3, double b4);

static void *dsd_embed_init (int nchans)
{
    EmbedContext *context = (EmbedContext *) calloc (1, sizeof (EmbedContext));

    context->nchans = nchans;
    context->parity_shifters = calloc (nchans, sizeof (uint32_t));
    context->noise_feedback = calloc (nchans, sizeof (float));
    context->noise_shapers = calloc (nchans, sizeof (Biquad));

// TRANSFER FUNCTION:
// nominator:
// 	+1.0 * z^{0}
// 	-3.265716689706981 * z^{-1}
// 	+4.267791696608121 * z^{-2}
// 	-2.6428195890912085 * z^{-3}
// 	+0.6509425172718173 * z^{-4}
// denominator:
// 	+1.0 * z^{0}
// 	+0.013993225356330928 * z^{-1}
// 	+1.64215031498716 * z^{-2}
// 	+0.03615562742615169 * z^{-3}
// 	+0.744295543451128 * z^{-4}

    for (int i = 0; i < nchans; ++i) {
        shaper_init (context->noise_shapers + i,
            1.0, -3.265716689706981, +4.267791696608121, -2.6428195890912085, +0.6509425172718173,
            +0.013993225356330928, +1.64215031498716, +0.03615562742615169, +0.744295543451128); // version 4
        if (getrandom (context->parity_shifters + i, 4, 0) != 4)
            fprintf (stderr, "getrandom() not working!\n");
    }

    return context;
}

// src_buffer[] is 1-bit DSD (8 bits per integer)
// dst_buffer[] is 24-bit raw DXD data (decimated DSD)
// DSD data is embedded into DXD data with parity bit and noise-shaping into dst_buffer[]

static void dsd_embed_run (void *embed_context, int32_t *dst_buffer, unsigned char *src_buffer, int nsamples)
{
    EmbedContext *context = (EmbedContext *) embed_context;
    int chan = 0;

    for (int i = 0; i < nsamples * context->nchans; ++i) {
        float codevalue = dst_buffer [i] - context->noise_feedback [chan];
        int32_t outvalue = ((int32_t) codevalue & 0xffffff00) | src_buffer [i];

#ifdef PILOT_SEQUENCE
        int pilot_parity_bit = (PILOT_SEQUENCE >> (63 - (context->sample_index & 0x3f))) & 1;
        pilot_parity_bit ^= __builtin_parity (context->parity_shifters [chan] & 0x40001000);
        context->parity_shifters [chan] = (context->parity_shifters [chan] << 1) | pilot_parity_bit;
        outvalue ^= (__builtin_parity (outvalue) ^ pilot_parity_bit) << 8;
#endif

        context->noise_feedback [chan] = biquad_apply_sample (context->noise_shapers + chan, outvalue - codevalue);
        dst_buffer [i] = outvalue;

        if (++chan == context->nchans) {
            context->sample_index++;
            chan = 0;
        }
    }
}

static void dsd_embed_destroy (void *embed_context)
{
    EmbedContext *context = (EmbedContext *) embed_context;

    free (context->parity_shifters);
    free (context->noise_feedback);
    free (context->noise_shapers);
    free (context);
}

/*------------------------------------------------------------------------------------------------------------------------*/

// Specify H(z) filter indirectly with N(z). Note, this is passed the actual noise-shaping
// transfer function, and so a0 needs to be 1.0. This function translates the filter to the
// H(z) form. The input to the resulting filter is the unfiltered quantization noise and the
// output, delayed by one sample, is subtracted from the [next] value to be quantized.

static void shaper_init (Biquad *f, double a0, double a1, double a2, double a3, double a4, double b1, double b2, double b3, double b4)
{
    BiquadCoefficients coeffs = { 0 };

    if (a0 != 1.0) {
        fprintf (stderr, "shaper_init() error: a0 = %g, should be one!\n", a0);
        exit (1);
    }

    coeffs.a0 = b1 - a1;
    coeffs.a1 = b2 - a2;
    coeffs.a2 = b3 - a3;
    coeffs.a3 = b4 - a4;

    coeffs.b1 = b1;
    coeffs.b2 = b2;
    coeffs.b3 = b3;
    coeffs.b4 = b4;

    biquad_init (f, &coeffs, 1.0);
}
