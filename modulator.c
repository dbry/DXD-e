////////////////////////////////////////////////////////////////////////////
//                           **** MODULATOR ****                          //
//                       Simple PCM to DSD Modulator                      //
//                     Copyright (c) 2026 David Bryant                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// modulator.c

#include "modulator.h"
#include "decimator.h"

#ifdef NO_PRUNING
#undef NO_PRUNING
#define NO_PRUNING  1
#else
#define NO_PRUNING  0
#endif

// #define ALLOW_DRIFT      // for testing

#define DEPTH_TERM  2       // tree search is terminated at this depth

// Higher-order noise shaping filters (i.e., above 2nd-order) become unstable,
// especially with higher levels. Increasing look-ahead helps, but we still need
// to soft-clip and dynamically reduce the order of the noise-shaping filter above
// +3.1 dB SACD. This is the maximum allowed level, although some DSD files go
// beyond this, and of course this code is designed to produce reasonable results
// with arbitrary PCM input.

#define SOFT_CLIP   0.72F               // we soft-clip above +3.1 dB SACD level
#define HARD_CLIP   0.86F               // we hard clip full-scale PCM to here

static void init_filter (int numTaps, float *filter, double fraction);
static inline double apply_filter (float *A, float *B, int num_taps);

#ifdef STATISTICS
static double best_sample (ModulatorChannel *cxt, const float *samples, double order, float *error, int depth);
#else
static double best_sample (const float *samples, double order, float *error, int depth);
#endif

// This table was determined empirically. When supplied a 1 kHz sine ramping linearly to full scale
// (which gets soft-clipped to 0.86), the maximum run length should be no greater than 18 bits (except in
// the case of depth = 0). Depths of 0 and 1 are NOT recommended (and not that much faster). The minimum
// depth for reasonably high quality is 4-5, with 8 and higher being optimal (especially in cases where
// the level exceeds 0 dB SACD).

static DepthShapingConfig DepthShapingConfigs [] = {
    { 2.40, 0.40, 2.00  },   // depth = 0
    { 2.60, 0.40, 2.00  },   // depth = 1
    { 2.75, 0.40, 2.30  },   // depth = 2
    { 2.85, 0.40, 2.50  },   // depth = 3
    { 3.00, 0.40, 2.60  },   // depth = 4
    { 3.00, 0.50, 2.70  },   // depth = 5
    { 3.00, 0.60, 2.75  },   // depth = 6
    { 3.00, 0.66, 2.80  },   // depth = 7
    { 3.00, 0.72, 2.825 },   // depth = 8
    { 3.00, 0.74, 2.85  },   // depth = 9
    { 3.00, 0.76, 2.875 },   // depth = 10
    { 3.00, 0.78, 2.90  },   // depth = 11
    { 3.00, 0.80, 2.925 },   // depth = 12
    { 3.00, 0.81, 2.925 },   // depth = 13
    { 3.00, 0.82, 2.95  },   // depth = 14
    { 3.00, 0.83, 2.95  },   // depth = 15
    { 3.00, 0.84, 2.975 },   // depth = 16
    { 3.00, 0.85, 2.975 },   // depth = 17
    { 3.00, 0.86, 3.00  },   // depth = 18+ (pure 3rd-order to hard-clip limit)
};

#define NUM_CONFIG_DEPTHS (sizeof (DepthShapingConfigs) / sizeof (DepthShapingConfigs [0]))

Modulate *modulateInit (int numChannels, int depth, int flags)
{
    Modulate *cxt = calloc (1, sizeof (Modulate));
    float **upsample_filters;
    void *decimator = NULL;

    if (flags & MODULATOR_ALIGN_EMBEDDED)
        decimator = decimate_dsd_init (0, 0);

    cxt->numChannels = numChannels;

    for (int d = 0; d < NUM_CONFIG_DEPTHS; ++d) {
        DepthShapingConfig *shaping_config = DepthShapingConfigs + d;

        if (shaping_config->transition_level != HARD_CLIP)
            shaping_config->slope = (shaping_config->final_order - shaping_config->initial_order) /
                (HARD_CLIP - shaping_config->transition_level);
        else
            shaping_config->slope = 0.0;
    }

    upsample_filters = calloc (NUM_FILTERS, sizeof (float*));

    for (int f = 0; f < NUM_FILTERS; ++f) {
        upsample_filters [f] = calloc (US_TAPS, sizeof (float));
        init_filter (US_TAPS, upsample_filters [f], (f + 0.5) / NUM_FILTERS);
    }

    cxt->channels = calloc (numChannels, sizeof (ModulatorChannel));

    for (int c = 0; c < numChannels; ++c) {
        cxt->channels [c].chan = c;

        modulateSetDepth (cxt, c, depth);
        cxt->channels [c].flags = flags;

        cxt->channels [c].upsample_filters = upsample_filters;

        cxt->channels [c].source_buffer = calloc (NUM_SAMPLES, sizeof (float));
        cxt->channels [c].source_buffer_head = US_TAPS / 2;
        cxt->channels [c].source_buffer_tail = 0;

        cxt->channels [c].upsample_buffer = calloc (NUM_SAMPLES, sizeof (float));
        cxt->channels [c].upsample_buffer_fill = cxt->channels [c].upsample_buffer_conv = 0;
        cxt->channels [c].upsample_buffer_tail = 4;

        cxt->channels [c].dsd_buffer = calloc (NUM_SAMPLES, sizeof (unsigned char));
        cxt->channels [c].decimator = decimator;
    }

    if ((flags & MODULATE_MULTITHREADED) && numChannels > 1)
        cxt->workers = workersInit (numChannels - 1);

    return cxt;
}

void modulateSetDepth (Modulate *cxt, int channel_number, int depth)
{
    ModulatorChannel *cptr = cxt->channels + channel_number;

    if (depth > MAX_DEPTH)
        depth = MAX_DEPTH;

    if (channel_number < cxt->numChannels) {
        if (depth < NUM_CONFIG_DEPTHS)
            cptr->shaping_config = DepthShapingConfigs + depth;
        else
            cptr->shaping_config = DepthShapingConfigs + NUM_CONFIG_DEPTHS - 1;

        cptr->depth = depth;
    }
}

void modulateSetAlignment (Modulate *cxt, int channel_number, int enable)
{
    ModulatorChannel *cptr = cxt->channels + channel_number;

    if (channel_number < cxt->numChannels && cptr->decimator && enable == !(cptr->flags & MODULATOR_ALIGN_EMBEDDED)) {
        cptr->phase_locked = cptr->unlock_count = 0;
        cptr->flags ^= MODULATOR_ALIGN_EMBEDDED;
    }
}

static int modulateProcessChannelJob (void *ptr, void *sync_not_used);

int modulateProcess (Modulate *cxt, const float *input, int numInputFrames, unsigned char *mod_output, unsigned char *emb_output)
{
    for (int c = 0; c < cxt->numChannels; ++c) {
        cxt->channels [c].input = input + c;
        cxt->channels [c].numInputFrames = numInputFrames;
        if (mod_output) cxt->channels [c].mod_output = mod_output + c;
        if (emb_output) cxt->channels [c].emb_output = emb_output + c;
        cxt->channels [c].stride = cxt->numChannels;

        if (cxt->workers)
            workersEnqueueJob (cxt->workers, modulateProcessChannelJob, cxt->channels + c,
                c < cxt->numChannels - 1 ? WaitForAvailableWorkerThread : DontUseWorkerThread);
        else
            modulateProcessChannelJob (cxt->channels + c, NULL);
    }

    if (cxt->workers)
        workersWaitAllJobs (cxt->workers);

    return cxt->channels [0].numOutputFrames;
}

static unsigned char xor_images [] = {
    0x00, 0x04, 0x08, 0x0C,
    0x10, 0x18,
    0x20,
    0x30,
};

#define NUM_XORS (sizeof (xor_images) / sizeof (xor_images [0]))
#define SLOW_RATIO   16384.0
#define FAST_RATIO   256.0

static int modulateProcessChannelJob (void *ptr, void *sync_not_used)
{
    ModulatorChannel *cxt = ptr;
    cxt->numOutputFrames = 0;

    if (cxt->flags & MODULATOR_FLUSHED)
        cxt->numInputFrames = 0;

    if (cxt->numInputFrames < 0) {
        int samples_to_add = US_TAPS / 2 + (MAX_DEPTH + 3) / 8 + cxt->delayed_samples;

        if (cxt->source_buffer_head + samples_to_add >= NUM_SAMPLES) {
            int samples_to_move = cxt->source_buffer_head - cxt->source_buffer_tail;

            memmove (cxt->source_buffer, cxt->source_buffer + cxt->source_buffer_tail, samples_to_move * sizeof (float));
            cxt->source_buffer_head -= cxt->source_buffer_tail;
            cxt->source_buffer_tail -= cxt->source_buffer_tail;
        }

        for (int i = 0; i < samples_to_add; ++i)
            cxt->source_buffer [cxt->source_buffer_head + i] = cxt->source_buffer [cxt->source_buffer_head - 1];

        cxt->source_buffer_head += samples_to_add;
        cxt->flags |= MODULATOR_FLUSHED;
    }

    while (1) {
        if (cxt->source_buffer_tail + US_TAPS > cxt->source_buffer_head) {      // if we don't have enough source data to do anything...
            if (cxt->numInputFrames > 0) {                                      // if we have source samples still...
                if (cxt->source_buffer_head == NUM_SAMPLES) {                   // if the source buffer is full, shift it left
                    int samples_to_move = cxt->source_buffer_head - cxt->source_buffer_tail;

                    memmove (cxt->source_buffer, cxt->source_buffer + cxt->source_buffer_tail, samples_to_move * sizeof (float));
                    cxt->source_buffer_head -= cxt->source_buffer_tail;
                    cxt->source_buffer_tail -= cxt->source_buffer_tail;
                }

#ifdef STATISTICS
                if (*cxt->input < cxt->sample_min) cxt->sample_min = *cxt->input;
                if (*cxt->input > cxt->sample_max) cxt->sample_max = *cxt->input;
#endif

                cxt->source_buffer [cxt->source_buffer_head] = *cxt->input;
                cxt->input += cxt->stride;
                cxt->source_buffer_head++;
                cxt->numInputFrames--;
            }
            else
                break;
        }
        else {  // else we do have enough source data to generate 8 upsamples

            if (!(cxt->flags & MODULATOR_PREFILLED)) {
                for (int i = 0; i < US_TAPS / 2; ++i)
                    cxt->source_buffer [i] = cxt->source_buffer [US_TAPS / 2];

                cxt->flags |= MODULATOR_PREFILLED;
            }

            if (cxt->upsample_buffer_fill + NUM_FILTERS >= NUM_SAMPLES) {       // if upsample buffer is full...
                int samples_to_move = cxt->upsample_buffer_fill - cxt->upsample_buffer_tail;

                memmove (cxt->upsample_buffer, cxt->upsample_buffer + cxt->upsample_buffer_tail, samples_to_move * sizeof (float));
                memmove (cxt->dsd_buffer, cxt->dsd_buffer + cxt->upsample_buffer_tail, samples_to_move * sizeof (unsigned char));
                cxt->upsample_buffer_fill -= cxt->upsample_buffer_tail;
                cxt->upsample_buffer_conv -= cxt->upsample_buffer_tail;
                cxt->upsample_buffer_tail -= cxt->upsample_buffer_tail;
            }

            // generate 8 upsamples, and soft-clip if over +3.1 dB SACD
            float *upsample_ptr = cxt->upsample_buffer + cxt->upsample_buffer_fill;
            unsigned char *dsd_ptr = cxt->dsd_buffer + cxt->upsample_buffer_fill;
            float *source_ptr = cxt->source_buffer + cxt->source_buffer_tail;
            int dsd_data = (((int32_t)(source_ptr [7] * 8388608.0) & 0xff) << 8) |
                ((int32_t)(source_ptr [8] * 8388608.0) & 0xff);

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

                *dsd_ptr++ = (dsd_data >> (11 - f)) & 0x1;      // b.0 of dsd_buffer is embedded DSD

#ifdef STATISTICS
                if (upsample_ptr [-1] < cxt->upsample_min) cxt->upsample_min = upsample_ptr [-1];
                if (upsample_ptr [-1] > cxt->upsample_max) cxt->upsample_max = upsample_ptr [-1];
#endif
            }

            cxt->upsample_buffer_fill += NUM_FILTERS;
            cxt->source_buffer_tail++;
        }

        // do the actual SDM here, assuming we have sufficient samples for lookahead depth
        while (cxt->upsample_buffer_fill - cxt->upsample_buffer_conv > MAX_DEPTH) {
            float *sample_ptr = cxt->upsample_buffer + cxt->upsample_buffer_conv, sample_max = 0.0;
            unsigned char *dsd_ptr = cxt->dsd_buffer + cxt->upsample_buffer_conv++;
            float order = cxt->shaping_config->initial_order;

            for (int i = 0; i <= cxt->depth; ++i)
                sample_max = fmax (sample_max, fabs (sample_ptr [i]));

            if (sample_max > cxt->shaping_config->transition_level)
                order += (sample_max - cxt->shaping_config->transition_level) * cxt->shaping_config->slope;

#ifdef STATISTICS
            float outsample = best_sample (cxt, sample_ptr, order, cxt->error_feedback, cxt->depth);
            double filtered_error = outsample - *sample_ptr, unfiltered_error = cxt->error_feedback [0];

            *dsd_ptr |= ((outsample > 0.0) & 1) << 1;           // b.1 of dsd_buffer is calculated DSD sample

            if (cxt->min_order == 0.0 || order < cxt->min_order) cxt->min_order = order;
            if (order > cxt->max_order) cxt->max_order = order;

            // test synch by periodically resetting DSD encoding
            // double seconds = cxt->total_samples / 2822400.0;
            // if ((seconds / 2.0) - floor (seconds / 2.0) == (double) cxt->chan / cxt->stride)
            //     cxt->error_feedback [0] = cxt->error_feedback [1] = cxt->error_feedback [2] = 0.0;

            if (outsample == cxt->last_sample) {
                if (++(cxt->run_count) == 100) {
                    fprintf (stderr, "\n*** ch %d: value %2g, terminating run of %d, ending at %.6f secs, peak value = %g ***\n\n",
                        cxt->chan, outsample, cxt->run_count, cxt->total_samples / 2822400.0, cxt->last_sample_peak);
                    exit (1);
                }

                if (sample_max > cxt->last_sample_peak)
                    cxt->last_sample_peak = sample_max;
            }
            else {
                if (cxt->run_count > cxt->max_run_count) {
                    cxt->max_run_count = cxt->run_count;
                    if (cxt->run_count >= 20)
                        fprintf (stderr, "ch %d: value %2g, terminating run of %d, ending at %.6f secs, peak value = %g\n",
                            cxt->chan, outsample, cxt->run_count, cxt->total_samples / 2822400.0, cxt->last_sample_peak);
                }

                cxt->last_sample_peak = sample_max;
                cxt->last_sample = outsample;
                cxt->run_count = 1;
            }

            cxt->rms_filtered_error += filtered_error * filtered_error;
            cxt->rms_unfiltered_error += unfiltered_error * unfiltered_error;
            if (unfiltered_error > cxt->max_unfiltered_error) cxt->max_unfiltered_error = unfiltered_error;
            if (unfiltered_error < cxt->min_unfiltered_error) cxt->min_unfiltered_error = unfiltered_error;
            if (filtered_error > cxt->max_filtered_error) cxt->max_filtered_error = filtered_error;
            if (filtered_error < cxt->min_filtered_error) cxt->min_filtered_error = filtered_error;
            cxt->total_samples++;
#else
            cxt->last_sample = best_sample (sample_ptr, order, cxt->error_feedback, cxt->depth);
            *dsd_ptr |= ((cxt->last_sample > 0.0) & 1) << 1;           // b.1 of dsd_buffer is calculated DSD sample
#endif
        }

        // while we have whole bytes of DSD data ready, output it
        while (cxt->upsample_buffer_tail + 8 <= cxt->upsample_buffer_conv) {
            unsigned char dsd_embedded = 0, dsd_calculated = 0;
            int bcount = 8;

            while (bcount--) {
                dsd_calculated = (dsd_calculated << 1) | ((cxt->dsd_buffer [cxt->upsample_buffer_tail] & 0x2) >> 1);
                dsd_embedded = (dsd_embedded << 1) | (cxt->dsd_buffer [cxt->upsample_buffer_tail++] & 0x1);
            }

            cxt->dsd_calculated_buffer [cxt->delayed_samples] = dsd_calculated;
            cxt->dsd_embedded_buffer [cxt->delayed_samples++] = dsd_embedded;

            if (cxt->delayed_samples == DSD_DELAY) {
                if (cxt->mod_output) {
                    *cxt->mod_output = cxt->dsd_calculated_buffer [0];
                    cxt->mod_output += cxt->stride;
                }

                if (cxt->emb_output) {
                    *cxt->emb_output = cxt->dsd_embedded_buffer [0];
                    cxt->emb_output += cxt->stride;
                }

#ifndef WRITE_ERROR_CHAN
                cxt->numOutputFrames++;
#endif

                if (cxt->flags & MODULATOR_ALIGN_EMBEDDED) {
                    int32_t calculated_sum = 0, embedded_sum = 0, average_sum, closest_average;
                    unsigned char dsd_merged_buffer [DSD_DELAY];
                    int transition_byte = (DSD_DELAY - 1) / 2;

                    for (int i = 0; i < 7; ++i) {
                        calculated_sum += decimate_single_sample (cxt->decimator, cxt->dsd_calculated_buffer + i);
                        embedded_sum += decimate_single_sample (cxt->decimator, cxt->dsd_embedded_buffer + i);
                    }

                    average_sum = (calculated_sum + embedded_sum + 7) / 14;
                    memcpy (dsd_merged_buffer, cxt->dsd_calculated_buffer, transition_byte);
                    memcpy (dsd_merged_buffer + transition_byte + 1, cxt->dsd_embedded_buffer + transition_byte + 1, transition_byte);

                    for (int xor_index = 0; xor_index < NUM_XORS; xor_index++) {
                        int32_t merged_sum = 0, merged_average;

                        dsd_merged_buffer [transition_byte] = (cxt->dsd_calculated_buffer [transition_byte] & 0xF0) | (cxt->dsd_embedded_buffer [transition_byte] & 0x0F);
                        dsd_merged_buffer [transition_byte] ^= xor_images [xor_index];

                        for (int i = 0; i < 7; ++i)
                            merged_sum += decimate_single_sample (cxt->decimator, dsd_merged_buffer + i);

                        merged_average = (merged_sum + 3) / 7;

                        if (!xor_index || abs (merged_average - average_sum) < abs (closest_average - average_sum))
                            closest_average = merged_average;
                    }

                    if (closest_average < average_sum)
                        cxt->minus_error_count++;
                    else if (closest_average > average_sum)
                        cxt->plus_error_count++;

                    if (abs (closest_average - average_sum) > 83656)
                        cxt->large_error_count++;

                    if (cxt->plus_error_count + cxt->minus_error_count >= 64 && cxt->plus_error_count != cxt->minus_error_count &&
                        (cxt->plus_error_count > cxt->minus_error_count) == (cxt->last_sample > 0.0)) {
                            double ratio;

                            if (cxt->large_error_count > 16 || !cxt->plus_error_count || !cxt->minus_error_count) {
                                if (cxt->phase_locked && (cxt->unlock_count++ >= 16 || cxt->large_error_count > 16))
                                    cxt->unlock_count = cxt->phase_locked = 0;
                            }
                            else if (!cxt->phase_locked && cxt->plus_error_count > 26 && cxt->minus_error_count > 26 && cxt->large_error_count < 4)
#ifdef STATISTICS
                                cxt->align_lock_count +=
#endif
                                cxt->phase_locked = 1;

                            ratio = cxt->phase_locked ? SLOW_RATIO : FAST_RATIO;
                            cxt->plus_error_count = cxt->minus_error_count = cxt->large_error_count = 0;
#ifndef ALLOW_DRIFT
                            cxt->error_feedback [0] = cxt->error_feedback [0] - cxt->error_feedback [0] / ratio + cxt->error_feedback [1] / ratio;
                            cxt->error_feedback [1] = cxt->error_feedback [1] - cxt->error_feedback [1] / ratio + cxt->error_feedback [2] / ratio;
                            cxt->error_feedback [2] = cxt->error_feedback [2] - cxt->error_feedback [2] / ratio;
#endif
                        }

#ifdef WRITE_ERROR_CHAN
                    if (cxt->chan == WRITE_ERROR_CHAN) {
                        int32_t error = closest_average - average_sum;
                        putchar (error & 0xff);
                        putchar ((error >> 8) & 0xff);
                        putchar ((error >> 16) & 0xff);
                    }
#endif
                }

                memmove (cxt->dsd_calculated_buffer, cxt->dsd_calculated_buffer + 1, DSD_DELAY - 1);
                memmove (cxt->dsd_embedded_buffer, cxt->dsd_embedded_buffer + 1, DSD_DELAY - 1);
                cxt->delayed_samples--;
            }
        }
    }

    return 0;
}

void modulateFree (Modulate *cxt)
{
#ifdef STATISTICS
    fprintf (stderr, "%ld total samples, %.2f seconds\n\n", cxt->channels [0].total_samples, cxt->channels [0].total_samples / 2822400.0);

    for (int c = 0; c < cxt->numChannels; ++c) {
        ModulatorChannel *chan = cxt->channels + c;
        fprintf (stderr, "chan %d: RMS noise = %.2f dB unfiltered, %.2f dB filtered, max run %d, order %.3f to %.3f\n", c,
            log10 (chan->rms_unfiltered_error / chan->total_samples * 2.0) * 10.0,
            log10 (chan->rms_filtered_error / chan->total_samples * 2.0) * 10.0,
            chan->max_run_count, chan->min_order, chan->max_order);
        if (chan->called_best_sample)
            fprintf (stderr, "        alt checked %.1f%%, alt used %.1f%%, %.1f ave leaves\n",
                chan->checked_alt_sample * 100.0 / chan->called_best_sample,
                chan->used_alt_sample * 100.0 / chan->called_best_sample,
                (double) chan->leaves / chan->called_best_sample);
        fprintf (stderr, "        PCM input (unclipped) range = %.3f to %.3f\n", chan->sample_min, chan->sample_max);
        fprintf (stderr, "          upsampled (clipped) range = %.3f to %.3f\n", chan->upsample_min, chan->upsample_max);
        fprintf (stderr, "             unfiltered error range = %.3f to %.3f\n", chan->min_unfiltered_error, chan->max_unfiltered_error);
        fprintf (stderr, "               filtered error range = %.3f to %.3f\n", chan->min_filtered_error, chan->max_filtered_error);
        if (chan->flags & MODULATOR_ALIGN_EMBEDDED)
            fprintf (stderr, "                   alignment locked = %d times\n\n", chan->align_lock_count);
    }
#endif

    if (cxt->workers)
        workersDeinit (cxt->workers);

    for (int c = 0; c < cxt->numChannels; ++c) {
        free (cxt->channels [c].dsd_buffer);
        free (cxt->channels [c].source_buffer);
        free (cxt->channels [c].upsample_buffer);
    }

    for (int f = 0; f < NUM_FILTERS; ++f)
        free (cxt->channels [0].upsample_filters [f]);

    decimate_dsd_destroy (cxt->channels [0].decimator);
    free (cxt->channels [0].upsample_filters);
    free (cxt->channels);
    free (cxt);
}

#define MIN_RMS_ERROR(x)    ((x)*(x) - fabs (2.0*(x)) + 1.0)
#define SQUARE(x)           ((x)*(x))

#ifdef STATISTICS
static double min_error (ModulatorChannel *cxt, const float *samples, const float *filter, const float *error, int depth, double max_min)
#else
static double min_error (const float *samples, const float *filter, const float *error, int depth, double max_min)
#endif
{
    double sample = samples [0] + (error [0] * filter [0]) + (error [1] * filter [1]) + (error [2] * filter [2]);

#if (DEPTH_TERM == 0)
    if (depth == 0) {
#ifdef STATISTICS
        cxt->leaves++;
#endif
        return MIN_RMS_ERROR (sample);
    }
#endif

#if (DEPTH_TERM == 1)
    if (depth == 1) {
        double sample_0 = samples [1] + ((-1.0 - sample) * filter [0]) + (error [0] * filter [1]) + (error [1] * filter [2]);
        double sample_1 = sample_0 + filter [0] * 2.0;

#ifdef STATISTICS
        cxt->leaves += 2;
#endif
        return fmin (SQUARE (-1.0 - sample) + MIN_RMS_ERROR (sample_0), SQUARE (1.0 - sample) + MIN_RMS_ERROR (sample_1));
    }
#endif

#if (DEPTH_TERM == 2)
    if (depth == 2) {
        double sample_0 = samples [1] + ((-1.0 - sample) * filter [0]) + (error [0] * filter [1]) + (error [1] * filter [2]);
        double sample_1 = sample_0 + filter [0] * 2.0;

        double sample_0_0 = samples [2] + ((-1.0 - sample_0) * filter [0]) + ((-1.0 - sample) * filter [1]) + (error [0] * filter [2]);
        double sample_0_1 = sample_0_0 + filter [0] * 2.0;
        double sample_1_0 = samples [2] + ((-1.0 - sample_1) * filter [0]) + ((+1.0 - sample) * filter [1]) + (error [0] * filter [2]);
        double sample_1_1 = sample_1_0 + filter [0] * 2.0;

        double error_0_0 = SQUARE (-1.0 - sample) + SQUARE (-1.0 - sample_0) + MIN_RMS_ERROR (sample_0_0);
        double error_0_1 = SQUARE (-1.0 - sample) + SQUARE (+1.0 - sample_0) + MIN_RMS_ERROR (sample_0_1);
        double error_1_0 = SQUARE (+1.0 - sample) + SQUARE (-1.0 - sample_1) + MIN_RMS_ERROR (sample_1_0);
        double error_1_1 = SQUARE (+1.0 - sample) + SQUARE (+1.0 - sample_1) + MIN_RMS_ERROR (sample_1_1);

#ifdef STATISTICS
        cxt->leaves += 4;
#endif

        return fmin (fmin (error_0_0, error_0_1), fmin (error_1_0, error_1_1));
    }
#endif

    {
        double first_sample = (sample < 0.0) ? -1.0 : 1.0;
        double error_sum = SQUARE (first_sample - sample);

        if (NO_PRUNING || error_sum < max_min) {
            float loc_error [] = { first_sample - sample, error [0], error [1] };
            double alt_error_sum = SQUARE (-first_sample - sample);

#ifdef STATISTICS
            error_sum += min_error (cxt, samples + 1, filter, loc_error, depth - 1, max_min - error_sum);
#else
            error_sum += min_error (samples + 1, filter, loc_error, depth - 1, max_min - error_sum);
#endif
            if (NO_PRUNING || alt_error_sum < error_sum) {
                loc_error [0] = -first_sample - sample;
#ifdef STATISTICS
                alt_error_sum += min_error (cxt, samples + 1, filter, loc_error, depth - 1, error_sum - alt_error_sum);
#else
                alt_error_sum += min_error (samples + 1, filter, loc_error, depth - 1, error_sum - alt_error_sum);
#endif
                if (alt_error_sum < error_sum)
                    error_sum = alt_error_sum;
            }
        }

        return error_sum;
    }
}

#ifdef STATISTICS
static double best_sample (ModulatorChannel *cxt, const float *samples, double order, float *error, int depth)
#else
static double best_sample (const float *samples, double order, float *error, int depth)
#endif
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
#ifdef STATISTICS
            error_sum += min_error (cxt, samples + 1, filter, error, depth - 1, DBL_MAX);
            cxt->called_best_sample++;
#else
            error_sum += min_error (samples + 1, filter, error, depth - 1, DBL_MAX);
#endif
            if (NO_PRUNING || alt_error_sum < error_sum) {
                error [0] = -best_sample - sample;
#ifdef STATISTICS
                alt_error_sum += min_error (cxt, samples + 1, filter, error, depth - 1, error_sum - alt_error_sum);
                cxt->checked_alt_sample++;
#else
                alt_error_sum += min_error (samples + 1, filter, error, depth - 1, error_sum - alt_error_sum);
#endif
                if (alt_error_sum < error_sum) {
                    best_sample = -best_sample;
#ifdef STATISTICS
                    cxt->used_alt_sample++;
#endif
                }
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

