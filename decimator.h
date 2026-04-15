////////////////////////////////////////////////////////////////////////////
//                           **** DECIMATOR ****                          //
//                       Simple DSD to PCM Decimator                      //
//                     Copyright (c) 2026 David Bryant                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// decimator.h

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

// 56 term decimation filter
// < 0.5 dB down at 20 kHz
// > 100 dB stopband attenuation (fs/12)

static const int32_t decm_filter [] = {
    4, 17, 56, 147, 336, 692, 1315, 2337,
    3926, 6281, 9631, 14216, 20275, 28021, 37619, 49155,
    62616, 77870, 94649, 112551, 131049, 149507, 167220, 183448,
    197472, 208636, 216402, 220385, 220385, 216402, 208636, 197472,
    183448, 167220, 149507, 131049, 112551, 94649, 77870, 62616,
    49155, 37619, 28021, 20275, 14216, 9631, 6281, 3926,
    2337, 1315, 692, 336, 147, 56, 17, 4,
};

#define NUM_FILTER_TERMS 56
#define HISTORY_BYTES ((NUM_FILTER_TERMS+7)/8)
#define DELAY_SAMPLES ((HISTORY_BYTES - 1) / 2)

typedef struct {
    unsigned char delay [HISTORY_BYTES];
    int32_t last_sample;
} DecimationChannel;

typedef struct {
    int32_t conv_tables [HISTORY_BYTES] [256];
    DecimationChannel *chans;
    int num_channels, reset;
} DecimationContext;

void *decimate_dsd_init (int num_channels);
void decimate_dsd_reset (void *decimate_context);
// void decimate_dsd_run (void *decimate_context, int32_t *out_samples, unsigned char *in_samples, int num_samples);
int decimate_dsd_run (void *decimate_context, const unsigned char *in_samples, int numInputFrames, int32_t *out_samples);
void decimate_dsd_destroy (void *decimate_context);
