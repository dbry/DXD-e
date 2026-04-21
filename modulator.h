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

#define NUM_FILTERS 8                   // upsample ratio
#define US_TAPS     16                  // upsample filter number of taps
#define NS_TAPS     3                   // taps in noise-shaping filter
#define NUM_SAMPLES 1024

#define MODULATOR_PREFILLED 0x1
#define MODULATOR_FLUSHED   0x2

typedef struct {
    float initial_order, transition_level, final_order, slope;
} DepthShapingConfig;

typedef struct {
    unsigned int input_used, output_generated;
} ModulateResult;

typedef struct {
    int numChannels, depth, flags;
    int upsample_buffer_samples, upsample_buffer_fill, upsample_buffer_conv, upsample_buffer_tail;
    int source_buffer_samples, source_buffer_head, source_buffer_tail;
    float *upsample_filters [NUM_FILTERS];
    DepthShapingConfig *shaping_config;
    uint32_t *tpdf_generators;

    float **source_buffers, **upsample_buffers, **error_feedback;

#ifdef STATISTICS
    double *rms_filtered_error, *rms_unfiltered_error;
    double *max_filtered_error, *max_unfiltered_error;
    double *min_filtered_error, *min_unfiltered_error;
    double *last_samples, *last_samples_peak;
    int *max_run_counts, *run_counts;
    int64_t total_samples;
#endif
} Modulate;

Modulate *modulateInit (int numChannels, int depth);
ModulateResult modulateProcess (Modulate *cxt, const float *input, int numInputFrames, unsigned char *output, int numOutputFrames);
void modulateFree (Modulate *cxt);

