////////////////////////////////////////////////////////////////////////////
//                           **** DSD UTILS ****                          //
//                         Various DSD/DXD Helpers                        //
//                     Copyright (c) 2026 David Bryant                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// dsd-utils.c

#include "dsd-utils.h"

////////////////////////////////////////////////////////////////////////////
// Code to embed DSD audio data into 24-bit PCM (DXD-e) and optionally
// insert the digital pilot signal used later to verify the presence of
// the embedded DSD.
////////////////////////////////////////////////////////////////////////////

static void shaper_init (Biquad *f, double a0, double a1, double a2, double a3, double a4, double b1, double b2, double b3, double b4);

EmbedContext *dsd_embed_init (int nchans, int flags)
{
    EmbedContext *context = (EmbedContext *) calloc (1, sizeof (EmbedContext));
    uint32_t seed = 0x31415926;

    context->flags = flags;
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

        if (flags & EMBED_PILOT_SIGNAL) {
            if (flags & EMBED_PILOT_UNIQUE) {
                if (getrandom (context->parity_shifters + i, 4, 0) != 4)
                    fprintf (stderr, "generate-dxd: getrandom() not working!\n");
            }
            else {
                context->parity_shifters [i] = seed;
                seed = (seed << 1) | ((seed >> 31) & 1);
            }
        }
    }

    return context;
}

// src_buffer[] is 1-bit DSD (8 bits per byte)
// dst_buffer[] is 24-bit raw DXD data (decimated DSD)
// DSD data is embedded into DXD data with parity bit and noise-shaping into dst_buffer[]

void dsd_embed_run (EmbedContext *context, int32_t *dst_buffer, unsigned char *src_buffer, int nsamples)
{
    int chan = 0;

    for (int i = 0; i < nsamples * context->nchans; ++i) {
        float codevalue = dst_buffer [i] - context->noise_feedback [chan];
        int32_t outvalue = ((int32_t) codevalue & 0xffffff00) | src_buffer [i];

        if (context->flags & EMBED_PILOT_SIGNAL) {
            int pilot_parity_bit = (PILOT_SEQUENCE >> (63 - (context->sample_index & 0x3f))) & 1;
            pilot_parity_bit ^= __builtin_parity (context->parity_shifters [chan] & 0x40001000);
            context->parity_shifters [chan] = (context->parity_shifters [chan] << 1) | pilot_parity_bit;
            outvalue ^= (__builtin_parity (outvalue) ^ pilot_parity_bit) << 8;
        }

        context->noise_feedback [chan] = biquad_apply_sample (context->noise_shapers + chan, outvalue - codevalue);
        dst_buffer [i] = outvalue;

        if (++chan == context->nchans) {
            context->sample_index++;
            chan = 0;
        }
    }
}

void dsd_embed_destroy (EmbedContext *context)
{
    free (context->parity_shifters);
    free (context->noise_feedback);
    free (context->noise_shapers);
    free (context);
}

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

////////////////////////////////////////////////////////////////////////////
// Code to detect the digital pilot signal that verifies the presence of
// the embedded DSD.
////////////////////////////////////////////////////////////////////////////

PilotDetect *PilotDetectInit (int nchans)
{
    PilotDetect *context = (PilotDetect *) calloc (1, sizeof (PilotDetect));
    uint64_t shifter = PILOT_SEQUENCE;

    context->nchans = nchans;

    for (int i = 0; i < 64; ++i) {
        context->parity_masks [i] = shifter;
        shifter = (shifter << 1) | ((shifter >> 63) & 1);
    }

    context->chans = calloc (sizeof (PilotDetectChannel), nchans);

    return context;
}

int PilotDetectChannelRun (PilotDetect *context, const int32_t *src_buffer, int chan, int nsamples)
{
    PilotDetectChannel *chanptr = context->chans + chan;
    int retval = chanptr->locked;

    for (int index = 0; index < nsamples; ++index) {
        chanptr->parity_shifter = (chanptr->parity_shifter << 1) | __builtin_parity (src_buffer [(index * context->nchans) + chan]);
        chanptr->channel_shifter = (chanptr->channel_shifter << 1) | __builtin_parity (chanptr->parity_shifter & 0x80002001);

        if (chanptr->locked) {
            chanptr->locked = ((chanptr->locked + 1) & 0x3f) | 0x40;
            if (chanptr->channel_shifter != context->parity_masks [chanptr->locked & 0x3f]) {
                fprintf (stderr, "%d:  unlocked: %.4f (%d/%d)\n", chan, chanptr->sample_index / 352800.0, index, nsamples);
                retval = chanptr->locked = 0;
            }
        }
        else
            for (int i = 0; i < 64; ++i)
                if (chanptr->channel_shifter == context->parity_masks [i]) {
                    if (chanptr->sample_index <= 94) {
                        fprintf (stderr, "%d: prelocked: %.4f (%d/%d)\n", chan, chanptr->sample_index / 352800.0, index, nsamples);
                        retval = 1;
                    }
                    else
                        fprintf (stderr, "%d:    locked: %.4f (%d/%d)\n", chan, chanptr->sample_index / 352800.0, index, nsamples);

                    chanptr->locked = i | 0x40;
                    break;
                }

        chanptr->sample_index++;
    }

    return retval;
}

void PilotDetectDestroy (PilotDetect *context)
{
    free (context->chans);
    free (context);
}

////////////////////////////////////////////////////////////////////////////
// Code to merge two streams of DSD audio, stitching the two segments at
// the best possible location and possibly fudging a few DSD samples around
// the transition point for the smallest possible glitch.
////////////////////////////////////////////////////////////////////////////

static unsigned char xor_images [] = {
    0x00, 0x04, 0x08, 0x0C,
    0x10, 0x18,
    0x20,
    0x30,
};

#define NUM_XORS (sizeof (xor_images) / sizeof (xor_images [0]))
#define SQUARE(x) ((x)*(x))

typedef struct {
    double value, slope;
    double abs_error, rms_error;
    int abs_error_rank, rms_error_rank;
    unsigned char xor_value;
} evalPoint;

#ifdef STATISTICS
static double total_abs_error, total_rms_error;
static int num_transitions;
#endif

void dsd_transition (DecimationContext *decimator, int64_t samples, unsigned char *initial_dsd, const unsigned char *final_dsd, int byte_count)
{
    int num_eval_points = byte_count - 12, num_pcm_values = byte_count - 6, best_eval_point = 0;
    evalPoint *evalPoints = calloc (num_eval_points, sizeof (evalPoint));
    int32_t *initial_pcm = calloc (num_pcm_values, sizeof (int32_t));
    int32_t *final_pcm = calloc (num_pcm_values, sizeof (int32_t));

    for (int i = 0; i < num_pcm_values; ++i) {
        initial_pcm [i] = decimate_single_sample (decimator, initial_dsd + i);
        final_pcm [i] = decimate_single_sample (decimator, final_dsd + i);
    }

    for (int i = 0; i < num_eval_points; ++i) {
        int32_t *initial_pcm_eval = initial_pcm + i, *final_pcm_eval = final_pcm + i, target_pcm [7];
        unsigned char transition_buffer [13];
        double slope = 0.0, value = 0.0;

        for (int j = 0; j < 7; ++j) {
            target_pcm [j] = (initial_pcm_eval [j] + final_pcm_eval [j]) / 2.0;
            value += (initial_pcm_eval [j] + final_pcm_eval [j]) / 14.0;
        }

        for (int j = 0; j <= 2; ++j)
            slope += ((initial_pcm_eval [6 - j] + final_pcm_eval [6 - j]) - (initial_pcm_eval [j] + final_pcm_eval [j])) / 24.0;

        evalPoints [i].value = value;
        evalPoints [i].slope = slope;
        evalPoints [i].abs_error = FLT_MAX;

        memcpy (transition_buffer, initial_dsd + i, 6);
        memcpy (transition_buffer + 7, final_dsd + i + 7, 6);

        for (int x = 0; x < NUM_XORS; ++x) {
            double average = 0.0, rms_error = 0.0;

            transition_buffer [6] = ((initial_dsd [i + 6] & 0xF0) | (final_dsd [i + 6] & 0x0F)) ^ xor_images [x];

            for (int j = 0; j < 7; ++j) {
                double sample = decimate_single_sample (decimator, transition_buffer + j);
                rms_error += SQUARE (sample - target_pcm [j]) / 7.0;
                average += sample / 7.0;
            }

            if (fabs (average - value) < fabs (evalPoints [i].abs_error)) {
                evalPoints [i].abs_error = average - value;
                evalPoints [i].xor_value = xor_images [x];
                evalPoints [i].rms_error = rms_error;
            }
        }
    }

    for (int rank = 1; rank <= num_eval_points; ++rank) {
        double best_rms_error = FLT_MAX, best_abs_error = FLT_MAX;
        int best_abs_error_index = 0, best_rms_error_index = 0;

        for (int i = 0; i < num_eval_points; ++i) {
            if (fabs (evalPoints [i].abs_error) < best_abs_error && !evalPoints [i].abs_error_rank) {
                best_abs_error = fabs (evalPoints [i].abs_error);
                best_abs_error_index = i;
            }

            if (evalPoints [i].rms_error < best_rms_error && !evalPoints [i].rms_error_rank) {
                best_rms_error = evalPoints [i].rms_error;
                best_rms_error_index = i;
            }
        }

        evalPoints [best_abs_error_index].abs_error_rank = rank;
        evalPoints [best_rms_error_index].rms_error_rank = rank;

        if (evalPoints [best_abs_error_index].rms_error_rank) {
            if (evalPoints [best_rms_error_index].abs_error_rank &&
                evalPoints [best_rms_error_index].abs_error_rank < evalPoints [best_abs_error_index].rms_error_rank)
                    best_eval_point = best_rms_error_index;
            else
                best_eval_point = best_abs_error_index;

            break;
        }
        else if (evalPoints [best_rms_error_index].abs_error_rank) {
            best_eval_point = best_rms_error_index;
            break;
        }
    }

#ifdef STATISTICS
    fprintf (stderr, "time = %8.5f (%4d, 0x%02x): y,m = %6.0f,%5.0f, abs error = %7.2f (%3d), rms error = %7.2f (%3d)\n",
        (samples + best_eval_point + 6) / 352800.0, best_eval_point, evalPoints [best_eval_point].xor_value,
        evalPoints [best_eval_point].value / 256.0,
        evalPoints [best_eval_point].slope / 256.0,
        evalPoints [best_eval_point].abs_error / 256.0, evalPoints [best_eval_point].abs_error_rank,
        sqrt (evalPoints [best_eval_point].rms_error) / 256.0, evalPoints [best_eval_point].rms_error_rank),

    total_abs_error += fabs (evalPoints [best_eval_point].abs_error) / 256.0;
    total_rms_error += sqrt (evalPoints [best_eval_point].rms_error) / 256.0;
    num_transitions++;
#endif

    initial_dsd [best_eval_point + 6] &= 0xF0;
    initial_dsd [best_eval_point + 6] |= final_dsd [best_eval_point + 6] & 0x0F;
    initial_dsd [best_eval_point + 6] ^= evalPoints [best_eval_point].xor_value;
    memcpy (initial_dsd + best_eval_point + 7, final_dsd + best_eval_point + 7, byte_count - best_eval_point - 7);

    free (evalPoints);
    free (initial_pcm);
    free (final_pcm);
}

void dsd_transition_dumpstats (FILE *stream)
{
#ifdef STATISTICS
    if (num_transitions)
        fprintf (stream, "%d DSD transitions, average absolute error = %.3f, average RMS error = %.3f\n",
            num_transitions, total_abs_error / num_transitions, total_rms_error / num_transitions);
#endif
}

