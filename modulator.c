////////////////////////////////////////////////////////////////////////////
//                           **** MODULATOR ****                          //
//                       Simple PCM to DSD Modulator                      //
//                     Copyright (c) 2026 David Bryant                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// modulator.c

#include "modulator.h"

// Higher-order noise shaping filters (i.e., above 2nd-order) become unstable,
// especially with higher levels. Increasing look-ahead helps, but we still need
// to soft-clip and dynamically reduce the order of the noise-shaping filter above
// +3.1 dB SACD. This is the maximum allowed level, although some DSD files go
// beyond this, and of course this code is designed to produce reasonable results
// with arbitrary PCM input.

#define SOFT_CLIP   0.72                // we soft-clip above +3.1 dB SACD level, and also start reducing noise-shaping here
#define HARD_CLIP   0.86                // we hard clip full-scale PCM to here, and also do pure 2nd-order noise-shaping here
#define MIN_ORDER   2.00                // 2nd-order is minimum noise-shaping filter
#define MAX_ORDER   3.00                // 3rd-order is maximum noise-shaping filter

static void init_filter (int numTaps, float *filter, double fraction);
static inline double apply_filter (float *A, float *B, int num_taps);
static double best_sample (const float *samples, double order, float *error, int depth);

Modulate *modulateInit (int numChannels, int depth)
{
    Modulate *cxt = calloc (1, sizeof (Modulate));

    cxt->numChannels = numChannels;
    cxt->depth = depth;

    cxt->order = MIN_ORDER + (depth * 0.09) + 0.28;     // these constants determined empirically

    if (cxt->order > MAX_ORDER)
        cxt->order = MAX_ORDER;

    for (int f = 0; f < NUM_FILTERS; ++f) {
        cxt->upsample_filters [f] = calloc (US_TAPS, sizeof (float));
        init_filter (US_TAPS, cxt->upsample_filters [f], (f + 0.5) / NUM_FILTERS);
    }

    cxt->source_buffers = calloc (numChannels, sizeof (float*));
    cxt->source_buffer_samples = NUM_SAMPLES;
    cxt->source_buffer_head = US_TAPS / 2;
    cxt->source_buffer_tail = 0;

    cxt->upsample_buffers = calloc (numChannels, sizeof (float*));
    cxt->upsample_buffer_samples = NUM_SAMPLES;
    cxt->upsample_buffer_fill = cxt->upsample_buffer_conv = 0;
    cxt->upsample_buffer_tail = 4;

    cxt->error_feedback = calloc (numChannels, sizeof (float*));

    for (int c = 0; c < numChannels; ++c) {
        cxt->upsample_buffers [c] = calloc (cxt->upsample_buffer_samples, sizeof (float));
        cxt->source_buffers [c] = calloc (cxt->source_buffer_samples, sizeof (float));
        cxt->error_feedback [c] = calloc (3, sizeof (float));
    }

    return cxt;
}

ModulateResult modulateProcess (Modulate *cxt, const float *input, int numInputFrames, unsigned char *output, int numOutputFrames)
{
    ModulateResult res = { 0, 0 };

    if (cxt->flags & MODULATOR_FLUSHED) {
        numInputFrames = 0;
    }

    if (numInputFrames < 0) {
        int samples_to_add = US_TAPS / 2 + (cxt->depth + 3) / 8;

        if (cxt->source_buffer_head + samples_to_add >= cxt->source_buffer_samples) {
            int samples_to_move = cxt->source_buffer_head - cxt->source_buffer_tail;

            for (int c = 0; c < cxt->numChannels; ++c)
                memmove (cxt->source_buffers [c], cxt->source_buffers [c] + cxt->source_buffer_tail, samples_to_move * sizeof (float));

            cxt->source_buffer_head -= cxt->source_buffer_tail;
            cxt->source_buffer_tail -= cxt->source_buffer_tail;
        }

        for (int c = 0; c < cxt->numChannels; ++c)
            for (int i = 0; i < samples_to_add; ++i)
                cxt->source_buffers [c] [cxt->source_buffer_head + i] = cxt->source_buffers [c] [cxt->source_buffer_head - 1];

        cxt->source_buffer_head += samples_to_add;
        cxt->flags |= MODULATOR_FLUSHED;
    }

    while (numOutputFrames > 0) {

        if (cxt->source_buffer_tail + US_TAPS > cxt->source_buffer_head) {      // if we don't have enough source data to do anything...
            if (numInputFrames > 0) {                                           // if we have source samples still...
                if (cxt->source_buffer_head == cxt->source_buffer_samples) {    // if the source buffer is full, shift it left
                    int samples_to_move = cxt->source_buffer_head - cxt->source_buffer_tail;

                    for (int c = 0; c < cxt->numChannels; ++c)
                        memmove (cxt->source_buffers [c], cxt->source_buffers [c] + cxt->source_buffer_tail, samples_to_move * sizeof (float));

                    cxt->source_buffer_head -= cxt->source_buffer_tail;
                    cxt->source_buffer_tail -= cxt->source_buffer_tail;
                }

                for (int c = 0; c < cxt->numChannels; ++c)
                    cxt->source_buffers [c] [cxt->source_buffer_head] = *input++;

                cxt->source_buffer_head++;
                numInputFrames--;
                res.input_used++;
            }
            else
                break;
        }
        else {  // we do have enough source data to generate 8 upsamples

            if (!(cxt->flags & MODULATOR_PREFILLED)) {
                for (int c = 0; c < cxt->numChannels; ++c)
                    for (int i = 0; i < US_TAPS / 2; ++i)
                        cxt->source_buffers [c] [i] = cxt->source_buffers [c] [US_TAPS / 2];

                cxt->flags |= MODULATOR_PREFILLED;
            }

            if (cxt->upsample_buffer_fill + NUM_FILTERS >= cxt->upsample_buffer_samples) {  // if upsample buffer is full...
                int samples_to_move = cxt->upsample_buffer_fill - cxt->upsample_buffer_tail;

                for (int c = 0; c < cxt->numChannels; ++c)
                    memmove (cxt->upsample_buffers [c], cxt->upsample_buffers [c] + cxt->upsample_buffer_tail, samples_to_move * sizeof (float));

                cxt->upsample_buffer_fill -= cxt->upsample_buffer_tail;
                cxt->upsample_buffer_conv -= cxt->upsample_buffer_tail;
                cxt->upsample_buffer_tail -= cxt->upsample_buffer_tail;
            }

            // generate 8 upsamples, and soft-clip if over +3.1 dB SACD
            for (int c = 0; c < cxt->numChannels; ++c) {
                float *upsample_ptr = cxt->upsample_buffers [c] + cxt->upsample_buffer_fill;
                float *source_ptr = cxt->source_buffers [c] + cxt->source_buffer_tail;

                for (int f = 0; f < NUM_FILTERS; ++f) {
                    double result = apply_filter (source_ptr, cxt->upsample_filters [f], US_TAPS);

                    if (fabs (result) > SOFT_CLIP) {
                        if (result >= 1.00)
                            *upsample_ptr++ = HARD_CLIP;
                        else if (result > 0.0)
                            *upsample_ptr++ = 1.0 - (0.0784 / (result - 0.44));
                        else if (result <= -1.00)
                            *upsample_ptr++ = -HARD_CLIP;
                        else
                            *upsample_ptr++ = -1.0 - (0.0784 / (result + 0.44));
                    }
                    else
                        *upsample_ptr++ = result;
                }
            }

            cxt->upsample_buffer_fill += NUM_FILTERS;
            cxt->source_buffer_tail++;
        }

        // do the actual SDM here, assuming we have sufficient samples for lookahead depth
        while (cxt->upsample_buffer_fill - cxt->upsample_buffer_conv > cxt->depth) {
            for (int c = 0; c < cxt->numChannels; ++c) {
                float *sample_ptr = cxt->upsample_buffers [c] + cxt->upsample_buffer_conv, sample_max = 0.0;
                float order = cxt->order;

                for (int i = 0; i <= cxt->depth; ++i)
                    sample_max = fmax (sample_max, fabs (sample_ptr [i]));

                if (sample_max > SOFT_CLIP)
                    order -= (sample_max - SOFT_CLIP) / (HARD_CLIP - SOFT_CLIP) * (order - 2.0);

                *sample_ptr = best_sample (sample_ptr, order, cxt->error_feedback [c], cxt->depth);
            }

            cxt->upsample_buffer_conv++;
        }

        // while we have whole bytes of DSD data ready, output it
        while (cxt->upsample_buffer_tail + 8 <= cxt->upsample_buffer_conv) {
            for (int c = 0; c < cxt->numChannels; ++c) {
                unsigned char dsd = 0;

                for (int b = 0; b < 8; ++b)
                    dsd = (dsd << 1) | (cxt->upsample_buffers [c] [cxt->upsample_buffer_tail + b] > 0.0);

                *output++ = dsd;
            }

            cxt->upsample_buffer_tail += 8;
            res.output_generated++;
            numOutputFrames--;
        }
    }

    return res;
}

void modulateFree (Modulate *cxt)
{
    for (int f = 0; f < NUM_FILTERS; ++f)
        free (cxt->upsample_filters [f]);

    for (int c = 0; c < cxt->numChannels; ++c) {
        free (cxt->upsample_buffers [c]);
        free (cxt->source_buffers [c]);
        free (cxt->error_feedback [c]);
    }

    free (cxt->upsample_buffers);
    free (cxt->source_buffers);
    free (cxt->error_feedback);
    free (cxt);
}

#define MIN_RMS_ERROR(x)    ((x)*(x) - fabs (2.0*(x)) + 1.0)
#define SQUARE(x)           ((x)*(x))

static double min_error (const float *samples, const float *filter, const float *error, int depth, double max_min)
{
    double sample = samples [0] + (error [0] * filter [0]) + (error [1] * filter [1]) + (error [2] * filter [2]);

    if (depth == 2) {
        double sample_0 = samples [1] + ((-1.0 - sample) * filter [0]) + (error [0] * filter [1]) + (error [1] * filter [2]);
        double sample_1 = sample_0 + filter [0] * 2.0;

        double sample_0_0 = samples [2] + ((-1.0 - sample_0) * filter [0]) + ((-1.0 - sample) * filter [1]) + (error [0] * filter [2]);
        double sample_0_1 = samples [2] + ((+1.0 - sample_0) * filter [0]) + ((-1.0 - sample) * filter [1]) + (error [0] * filter [2]);
        double sample_1_0 = samples [2] + ((-1.0 - sample_1) * filter [0]) + ((+1.0 - sample) * filter [1]) + (error [0] * filter [2]);
        double sample_1_1 = samples [2] + ((+1.0 - sample_1) * filter [0]) + ((+1.0 - sample) * filter [1]) + (error [0] * filter [2]);

        double error_0_0 = SQUARE (-1.0 - sample) + SQUARE (-1.0 - sample_0) + MIN_RMS_ERROR (sample_0_0);
        double error_0_1 = SQUARE (-1.0 - sample) + SQUARE (+1.0 - sample_0) + MIN_RMS_ERROR (sample_0_1);
        double error_1_0 = SQUARE (+1.0 - sample) + SQUARE (-1.0 - sample_1) + MIN_RMS_ERROR (sample_1_0);
        double error_1_1 = SQUARE (+1.0 - sample) + SQUARE (+1.0 - sample_1) + MIN_RMS_ERROR (sample_1_1);

        return fmin (fmin (error_0_0, error_0_1), fmin (error_1_0, error_1_1));
    }
    else {
        double first_sample = (sample < 0.0) ? -1.0 : 1.0;
        double error_sum = SQUARE (first_sample - sample);

        if (error_sum < max_min) {
            float loc_error [] = { first_sample - sample, error [0], error [1] };
            double alt_error_sum = SQUARE (-first_sample - sample);

            error_sum += min_error (samples + 1, filter, loc_error, depth - 1, max_min - error_sum);

            if (alt_error_sum < error_sum) {
                loc_error [0] = -first_sample - sample;
                alt_error_sum += min_error (samples + 1, filter, loc_error, depth - 1, error_sum - alt_error_sum);

                if (alt_error_sum < error_sum)
                    error_sum = alt_error_sum;
            }
        }

        return error_sum;
    }
}

static double best_sample (const float *samples, double order, float *error, int depth)
{
    float filter [] = { -order, 2.0 * order - 3.0, 2.0 - order };
    double sample = samples [0] + (error [0] * filter [0]) + (error [1] * filter [1]) + (error [2] * filter [2]);
    double best_sample = (sample < 0.0) ? -1.0 : 1.0;

    error [2] = error [1];
    error [1] = error [0];

    if (depth)
        switch (depth) {

        case 1: {
            double sample_0 = samples [1] + ((-1.0 - sample) * filter [0]) + (error [1] * filter [1]) + (error [2] * filter [2]);
            double sample_1 = sample_0 + filter [0] * 2.0;

            best_sample = SQUARE (+1.0 - sample) + MIN_RMS_ERROR (sample_1) < SQUARE (-1.0 - sample) + MIN_RMS_ERROR (sample_0) ? 1.0 : -1.0;
            break;
        }

        case 2: {
            double sample_0 = samples [1] + ((-1.0 - sample) * filter [0]) + (error [1] * filter [1]) + (error [2] * filter [2]);
            double sample_1 = sample_0 + filter [0] * 2.0;

            double sample_0_0 = samples [2] + ((-1.0 - sample_0) * filter [0]) + ((-1.0 - sample) * filter [1]) + (error [1] * filter [2]);
            double sample_0_1 = sample_0_0 + filter [0] * 2.0;
            double sample_1_0 = sample_0_0 + (filter [1] - SQUARE (filter [0])) * 2.0;
            double sample_1_1 = sample_1_0 + filter [0] * 2.0;

            double error_0_0 = (+2.0 * sample) + SQUARE (-1.0 - sample_0) + MIN_RMS_ERROR (sample_0_0);
            double error_0_1 = (+2.0 * sample) + SQUARE (+1.0 - sample_0) + MIN_RMS_ERROR (sample_0_1);
            double error_1_0 = (-2.0 * sample) + SQUARE (-1.0 - sample_1) + MIN_RMS_ERROR (sample_1_0);
            double error_1_1 = (-2.0 * sample) + SQUARE (+1.0 - sample_1) + MIN_RMS_ERROR (sample_1_1);

            best_sample = fmin (error_1_0, error_1_1) < fmin (error_0_0, error_0_1) ? 1.0 : -1.0;
            break;
        }

        default: {
            double alt_error_sum = SQUARE (-best_sample - sample);
            double error_sum = SQUARE (best_sample - sample);

            error [0] = best_sample - sample;
            error_sum += min_error (samples + 1, filter, error, depth - 1, DBL_MAX);

            if (alt_error_sum < error_sum) {
                error [0] = -best_sample - sample;
                alt_error_sum += min_error (samples + 1, filter, error, depth - 1, error_sum - alt_error_sum);

                if (alt_error_sum < error_sum)
                    best_sample = -best_sample;
            }
         }
    }

    error [0] = best_sample - sample;
    return best_sample;
}

#define BLACKMAN_HARRIS

static void init_filter (int numTaps, float *filter, double fraction)
{
    double filter_sum = 0.0, scaler, error;
    double tempFilter [numTaps];
    const double a0 = 0.35875;
    const double a1 = 0.48829;
    const double a2 = 0.14128;
    const double a3 = 0.01168;
    int i;

    // "dist" is the absolute distance from the sinc maximum to the filter tap to be calculated, in radians
    // "ratio" is that distance divided by half the tap count such that it reaches π at the window extremes

    // Note that with this scaling, the odd terms of the Blackman-Harris calculation appear to be negated
    // with respect to the reference formula version.

    for (i = 0; i < numTaps; ++i) {
        double dist = fabs ((numTaps / 2 - 1) + fraction - i) * M_PI;
        double ratio = dist / (numTaps / 2);
        double value;

        if (dist != 0.0) {
            value = sin (dist) / dist;

#ifdef BLACKMAN_HARRIS
                value *= a0 + a1 * cos (ratio) + a2 * cos (2 * ratio) + a3 * cos (3 * ratio);
#else
                value *= 0.5 * (1.0 + cos (ratio));     // Hann window
#endif
        }
        else
            value = 1.0;

        filter_sum += tempFilter [i] = value;
    }

    // filter should have unity DC gain

    scaler = 1.0 / filter_sum;
    error = 0.0;

    for (i = numTaps / 2; i < numTaps; i = numTaps - i - (i >= numTaps / 2)) {
        filter [i] = (tempFilter [i] *= scaler) - error;
        error += filter [i] - tempFilter [i];
    }
}

static double apply_filter (float *A, float *B, int num_taps)
{
    double sum = 0.0;

    do sum += (double) *A++ * *B++;
    while (--num_taps);

    return sum;
}

