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
#define NS_TAPS     4                   // taps in noise-shaping filter
#define NUM_SAMPLES 1024
#define DSD_DELAY   13                  // DSD byte delay line for analysis
#define MAX_DEPTH   24

#define MODULATE_MULTITHREADED      0x1
#define MODULATOR_PREFILLED         0x2     // do not set, internal use only
#define MODULATOR_FLUSHED           0x4     // do not set, internal use only
#define MODULATOR_ALIGN_EMBEDDED    0x10

typedef struct {
    int flags;
    int upsample_buffer_fill, upsample_buffer_conv, upsample_buffer_tail;
    int source_buffer_head, source_buffer_tail;
    float *source_buffer, *upsample_buffer;
    float last_sample, min_order, max_order;
    double error_feedback [NS_TAPS];
    unsigned char *dsd_buffer;
    float **upsample_filters;
    uint32_t tpdf_generator;
    void *decimator;

    const float *input;
    int numOutputFrames, numInputFrames, stride, depth, chan;
    unsigned char *mod_output, *emb_output;

    unsigned char dsd_embedded_buffer [DSD_DELAY], dsd_calculated_buffer [DSD_DELAY];
    int delayed_samples, plus_error_count, minus_error_count, large_error_count, phase_locked, unlock_count;
    int64_t total_samples;

#ifdef STATISTICS
    int64_t called_best_sample, checked_alt_sample, used_alt_sample, leaves;
    int max_run_count, run_count, align_lock_count, max_depth_seen;
    float sample_min, sample_max, upsample_min, upsample_max;
    double rms_filtered_error, rms_unfiltered_error;
    double max_filtered_error, max_unfiltered_error;
    double min_filtered_error, min_unfiltered_error;
    double last_sample_peak;
#endif
} ModulatorChannel;

typedef struct {
    int numChannels;
    Workers *workers;
    ModulatorChannel *channels;
} Modulate;

Modulate *modulateInit (int numChannels, int depth, int flags);
void modulateSetDepth (Modulate *cxt, int channel_number, int depth);
void modulateSetAlignment (Modulate *cxt, int channel_number, int enable);
int modulateProcess (Modulate *cxt, const float *input, int numInputFrames, unsigned char *mod_output, unsigned char *emb_output);
void modulateFree (Modulate *cxt);

