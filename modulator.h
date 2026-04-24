////////////////////////////////////////////////////////////////////////////
//                           **** MODULATOR ****                          //
//                       Simple PCM to DSD Modulator                      //
//                    Float to Integer Audio Decimation                   //
//                     Copyright (c) 2026 David Bryant                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// modulator.h

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <math.h>

#include "workers.h"

#define NUM_FILTERS 8                   // upsample ratio
#define US_TAPS     16                  // upsample filter number of taps
#define NS_TAPS     3                   // taps in noise-shaping filter
#define NUM_SAMPLES 1024

#define MODULATE_MULTITHREADED  0x1
#define MODULATOR_PREFILLED     0x2
#define MODULATOR_FLUSHED       0x4

typedef struct {
    float initial_order, transition_level, final_order, slope;
} DepthShapingConfig;

typedef struct {
    int flags;
    int upsample_buffer_fill, upsample_buffer_conv, upsample_buffer_tail;
    int source_buffer_head, source_buffer_tail;
    float *source_buffer, *upsample_buffer, error_feedback [NS_TAPS];
    unsigned char *dsd_buffer;
    DepthShapingConfig *shaping_config;
    float **upsample_filters;

    const float *input;
    int numOutputFrames, numInputFrames, stride, depth;
    unsigned char *output;

#ifdef STATISTICS
    int64_t called_best_sample, checked_alt_sample, used_alt_sample, leaves;
    float sample_min, sample_max, upsample_min, upsample_max;
    double rms_filtered_error, rms_unfiltered_error;
    double max_filtered_error, max_unfiltered_error;
    double min_filtered_error, min_unfiltered_error;
    double last_sample, last_sample_peak;
    int max_run_count, run_count, chan;
    int64_t total_samples;
#endif
} ModulatorChannel;

typedef struct {
    int numChannels;
    Workers *workers;
    ModulatorChannel *channels;
} Modulate;

Modulate *modulateInit (int numChannels, int depth, int flags);
int modulateProcess (Modulate *cxt, const float *input, int numInputFrames, unsigned char *output);
void modulateFree (Modulate *cxt);

