#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "decimator.h"
#include "biquad.h"

#define PILOT_SEQUENCE 0xf123456789abcde0
// #define RANDOM_SHIFTER_JUMPS
#define INCLUDE_PILOT

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
            if (strlen (argv [argi]) == 1 && (*argv [argi] == 'f' || *argv [argi] == 'F'))
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
    uint64_t *shifters;
    Biquad *shapers;
    float *feedback;
    int nchans;
} EmbedContext;

static void shaper_init (Biquad *f, double a0, double a1, double a2, double a3, double a4, double b1, double b2, double b3, double b4);

static void *dsd_embed_init (int nchans)
{
    EmbedContext *context = (EmbedContext *) calloc (1, sizeof (EmbedContext));

    context->nchans = nchans;
    context->shifters = calloc (nchans, sizeof (uint64_t));
    context->feedback = calloc (nchans, sizeof (float));
    context->shapers = calloc (nchans, sizeof (Biquad));

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
        // shaper_init (context->shapers + i, 1.0, -1.9131052389358465, +0.9195804711121185, 0.0, 0.0, +0.0015535122173491456, +0.8929992057512717, 0.0, 0.0); // version 1
        // shaper_init (context->shapers + i, 1.0, -3.6513540088861856, +5.162094711894004, -3.3528946153001415, +0.8433117656683422, -0.008357070470613955, +0.9052491172813699, 0.0, 0.0); // version 2
        // shaper_init (context->shapers + i,
        //     1.0, -3.2394787809861945, +4.224896866596852, -2.6248646851461928, +0.6507166688029212,
        //     +0.25316696308483166, +1.5779036618342488, +0.2411058793250051, +0.6764611042053907); // version 3
        shaper_init (context->shapers + i,
            1.0, -3.265716689706981, +4.267791696608121, -2.6428195890912085, +0.6509425172718173,
            +0.013993225356330928, +1.64215031498716, +0.03615562742615169, +0.744295543451128); // version 4
        context->shifters [i] = PILOT_SEQUENCE; 
    }

    return context;
}

// src_buffer[] is 1-bit DSD (8 bits per integer)
// dst_buffer[] is 24-bit raw DXD data
// DSD data is embedded into DXD data with parity bit and noise-shaping into dst_buffer[]

static uint32_t seed = 0x31415926;

static void dsd_embed_run (void *embed_context, int32_t *dst_buffer, unsigned char *src_buffer, int nsamples)
{
    EmbedContext *context = (EmbedContext *) embed_context;
    int chan = 0;

    for (int i = 0; i < nsamples * context->nchans; ++i) {
        float codevalue = dst_buffer [i] - context->feedback [chan];
        int32_t outvalue;

        context->shifters [chan] = (context->shifters [chan] << 1) | ((context->shifters [chan] >> 63) & 1);
        outvalue = ((int32_t)codevalue & 0xfffffe00) | src_buffer [i];

#ifdef INCLUDE_PILOT         // include pilot sequence
        outvalue |= ((__builtin_parity (outvalue) ^ (int32_t) context->shifters [chan]) & 1) << 8;
#else                        // don't include pilot sequence (always even parity)
        outvalue |= (__builtin_parity (outvalue) & 1) << 8;
#endif

        context->feedback [chan] = biquad_apply_sample (context->shapers + chan, outvalue - codevalue);
        dst_buffer [i] = outvalue;

        seed = ((seed << 4) - seed) ^ 1;
        seed = ((seed << 4) - seed) ^ 1;
        seed = ((seed << 4) - seed) ^ 1;

        if (!(seed & 0xfff)) {
            seed = ((seed << 4) - seed) ^ 1;
            seed = ((seed << 4) - seed) ^ 1;
            seed = ((seed << 4) - seed) ^ 1;
#ifdef RANDOM_SHIFTER_JUMPS
            int skips = seed >> 26;
            while (skips--)
                context->shifters [chan] = (context->shifters [chan] << 1) | ((context->shifters [chan] >> 63) & 1);
#endif
        }

        if (++chan == context->nchans)
            chan = 0;
    }
}

static void dsd_embed_destroy (void *embed_context)
{
    EmbedContext *context = (EmbedContext *) embed_context;

    free (context->shapers);
    free (context->feedback);
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
