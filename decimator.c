////////////////////////////////////////////////////////////////////////////
//                           **** DECIMATOR ****                          //
//                       Simple DSD to PCM Decimator                      //
//                     Copyright (c) 2026 David Bryant                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// decimator.c

#include "decimator.h"

static void extrapolate_pcm (int32_t *samples, int samples_to_extrapolate, int samples_visible, int num_channels);

// Lowpass, Sample rate = 352800, Fc = 44100, Q = 0.875
static BiquadCoefficients lowpass_filter = {
    0.10430216888580293, 0.20860433777160586, 0.10430216888580293, 0.0, 0.0,
    -1.007230842836138, 0.4244395183793498, 0.0, 0.0
};

void *decimate_dsd_init (int num_channels, int flags)
{
    DecimationContext *context = (DecimationContext *)malloc (sizeof (DecimationContext));
    double filter_sum = 0, filter_scale;
    int i, j;

    if (!context)
        return context;

    memset (context, 0, sizeof (*context));
    context->num_channels = num_channels;
    context->flags = flags;

    for (i = 0; i < NUM_FILTER_TERMS; ++i)
        filter_sum += decm_filter [i];

    filter_scale = ((1 << 23) - 1) / filter_sum * 16.0;

    for (i = 0; i < NUM_FILTER_TERMS; ++i) {
        int scaled_term = (int) floor (decm_filter [i] * filter_scale + 0.5);

        if (scaled_term) {
            for (j = 0; j < 256; ++j)
                if (j & (0x80 >> (i & 0x7)))
                    context->conv_tables [i >> 3] [j] += scaled_term;
                else
                    context->conv_tables [i >> 3] [j] -= scaled_term;
        }
    }

    if (num_channels) {
        context->chans = (DecimationChannel *)malloc (num_channels * sizeof (DecimationChannel));

        if (!context->chans) {
            free (context);
            return NULL;
        }
    }

    decimate_dsd_reset (context);

    return context;
}

void decimate_dsd_reset (void *decimate_context)
{
    DecimationContext *context = (DecimationContext *) decimate_context;
    int chan = 0, i;

    if (!context)
        return;

    for (chan = 0; chan < context->num_channels; ++chan) {
        if (context->flags & DECIMATE_LOWPASS)
            biquad_init (&context->chans [chan].lowpass_filter, &lowpass_filter, 1.0);

        for (i = 0; i < HISTORY_BYTES; ++i)
            context->chans [chan].delay [i] = 0x55;
    }

    context->reset = 1;
}

int decimate_dsd_run (void *decimate_context, const unsigned char *in_samples, int numInputFrames, int32_t *out_samples)
{
    DecimationContext *context = (DecimationContext *) decimate_context;
    int chan = 0, scount = numInputFrames;
    int32_t *outsamptr = out_samples;

    if (!context)
        return 0;

    if (numInputFrames < 0) {
        if (context->reset)
            return 0;

        for (int i = 0; i < DELAY_SAMPLES; ++i)
            for (int c = 0; c < context->num_channels; ++c)
                *out_samples++ = context->chans [c].last_sample;

        return DELAY_SAMPLES;
    }

    while (scount) {
        DecimationChannel *sp = context->chans + chan;
        int32_t sum = 0;

#if (HISTORY_BYTES == 7)
        sum += context->conv_tables [0] [sp->delay [0] = sp->delay [1]];
        sum += context->conv_tables [1] [sp->delay [1] = sp->delay [2]];
        sum += context->conv_tables [2] [sp->delay [2] = sp->delay [3]];
        sum += context->conv_tables [3] [sp->delay [3] = sp->delay [4]];
        sum += context->conv_tables [4] [sp->delay [4] = sp->delay [5]];
        sum += context->conv_tables [5] [sp->delay [5] = sp->delay [6]];
        sum += context->conv_tables [6] [sp->delay [6] = *in_samples++];
#else
        int i;

        for (i = 0; i < HISTORY_BYTES-1; ++i)
            sum += context->conv_tables [i] [sp->delay [i] = sp->delay [i+1]];

        sum += context->conv_tables [i] [sp->delay [i] = *in_samples++];
#endif

        if (context->flags & DECIMATE_LOWPASS) {
            float fsample = biquad_apply_sample (&sp->lowpass_filter, sum / 134217728.0);
            *outsamptr++ = sp->last_sample = (int32_t) floor (fsample * 8388608.0);
        }
        else
            *outsamptr++ = sp->last_sample = (sum + 8) >> 4;

        if (++chan == context->num_channels) {
            scount--;
            chan = 0;
        }
    }

    if (context->reset) {
        numInputFrames -= DELAY_SAMPLES;
        memmove (out_samples, out_samples + DELAY_SAMPLES * context->num_channels, numInputFrames * context->num_channels * sizeof (int32_t));
        extrapolate_pcm (out_samples, DELAY_SAMPLES, numInputFrames, context->num_channels);
        context->reset = 0;
    }

    return numInputFrames;
}

int32_t decimate_single_sample (void *decimate_context, const unsigned char in_samples [HISTORY_BYTES])
{
    DecimationContext *context = (DecimationContext *) decimate_context;

#if (HISTORY_BYTES == 7)
    int32_t sum =
        context->conv_tables [0] [in_samples [0]] +
        context->conv_tables [1] [in_samples [1]] +
        context->conv_tables [2] [in_samples [2]] +
        context->conv_tables [3] [in_samples [3]] +
        context->conv_tables [4] [in_samples [4]] +
        context->conv_tables [5] [in_samples [5]] +
        context->conv_tables [6] [in_samples [6]];
#else
    int32_t sum = 0;
    int i;

    for (i = 0; i < HISTORY_BYTES; ++i)
        sum += context->conv_tables [i] in_samples [i];
#endif

    return (sum + 8) >> 4;
}

// This function is used to linearly extrapolate some samples at the beginning of the first
// decoded frame because we don't have the previous DSD data to prefill the decimation filter.
// Currently we only extrapolate at the beginning of the file because we have an implicit
// delay in the decimation. It might be better, but more complicated, to have zero delay in
// the decimation and split the extrapolated samples between the beginning and end of the
// file.

static void extrapolate_pcm (int32_t *samples, int samples_to_extrapolate, int samples_visible, int num_channels)
{
    int scount = num_channels, min_period = 5, max_period = 10;

    if (samples_visible < samples_to_extrapolate + min_period * 2)
        return;

    if (samples_visible < samples_to_extrapolate + max_period * 2)
        max_period = (samples_visible - samples_to_extrapolate) / 2;

    while (scount--) {
        float left_value_ave = 0.0, right_value_ave = 0.0, slope;
        int period, i;

        for (period = min_period; period <= max_period; ++period) {
            float left_ratio = (samples_to_extrapolate + period / 2.0F) / period, right_ratio = (period / 2.0F) / period;
            int32_t *sam1 = samples + samples_to_extrapolate * num_channels, *sam2 = sam1 + period * num_channels;
            float ave1 = 0.0, ave2 = 0.0;

            for (i = 0; i < period; ++i) {
                ave1 += (float) sam1 [i * num_channels] / period;
                ave2 += (float) sam2 [i * num_channels] / period;
            }

            left_value_ave += ave1 + (ave1 - ave2) * left_ratio;
            right_value_ave += ave1 + (ave1 - ave2) * right_ratio;
        }

        right_value_ave /= (max_period - min_period + 1);
        left_value_ave /= (max_period - min_period + 1);
        slope = (right_value_ave - left_value_ave) / (samples_to_extrapolate - 1);

        for (i = 0; i < samples_to_extrapolate; ++i)
            samples [i * num_channels] = (int32_t) (left_value_ave + i * slope + 0.5);

        samples++;
    }
}

void decimate_dsd_destroy (void *decimate_context)
{
    DecimationContext *context = (DecimationContext *) decimate_context;

    if (!context)
        return;

    if (context->chans)
        free (context->chans);

    free (context);
}
