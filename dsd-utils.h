////////////////////////////////////////////////////////////////////////////
//                           **** DSD UTILS ****                          //
//                         Various DSD/DXD Helpers                        //
//                     Copyright (c) 2026 David Bryant                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// dsd-utils.h

#ifndef DSD_UTILS_H
#define DSD_UTILS_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <sys/random.h>

#include "biquad.h"

// ******************** DSD Decimator ********************

#define DECIMATE_LOWPASS    0x1

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
    Biquad lowpass_filter;
    int32_t last_sample;
} DecimateDSDchannel;

typedef struct {
    int32_t conv_tables [HISTORY_BYTES] [256];
    int flags, num_channels, reset;
    DecimateDSDchannel *chans;
} DecimateDSD;

#ifdef __cplusplus
extern "C" {
#endif

DecimateDSD *decimateDSDinit (int num_channels, int flags);
void decimateDSDreset (DecimateDSD *cxt);
int decimateDSDrun (DecimateDSD *cxt, const unsigned char *in_samples, int numInputFrames, int32_t *out_samples);
int32_t decimateSingleDSDsample (DecimateDSD *cxt, const unsigned char in_samples [HISTORY_BYTES]);
void decimateDSDdestroy (DecimateDSD *cxt);

#ifdef __cplusplus
}
#endif

// ******************** DSD Embedding & Detection ********************

#define PILOT_SEQUENCE 0xf123456789abcde0

#define EMBED_PILOT_SIGNAL  0x1
#define EMBED_PILOT_UNIQUE  0x2

typedef struct {
    uint32_t *parity_shifters;
    Biquad *noise_shapers;
    float *noise_feedback;
    int64_t sample_index;
    int nchans, flags;
} EmbedContext;

typedef struct {
    uint64_t channel_shifter, sample_index;
    uint32_t parity_shifter;
    int samples_to_skip;
    char locked;
} PilotDetectChannel;

typedef struct {
    uint64_t parity_masks [64];
    PilotDetectChannel *chans;
    int nchans;
} PilotDetect;

#ifdef __cplusplus
extern "C" {
#endif

EmbedContext *dsd_embed_init (int nchans, int flags);
void dsd_embed_run (EmbedContext *embed_context, int32_t *dst_buffer, unsigned char *src_buffer, int nsamples);
void dsd_embed_destroy (EmbedContext *embed_context);

PilotDetect *PilotDetectInit (int nchans);
int PilotDetectChannelRun (PilotDetect *context, const int32_t *src_buffer, int chan, int nsamples);
void PilotDetectDestroy (PilotDetect *context);

#ifdef __cplusplus
}
#endif

// ******************** DSD Transitioning ********************

#ifdef __cplusplus
extern "C" {
#endif

void dsd_transition (DecimateDSD *decimator, int64_t samples, unsigned char *initial_dsd, const unsigned char *final_dsd, int byte_count);
void dsd_transition_dumpstats (FILE *stream);

#ifdef __cplusplus
}
#endif

#endif
