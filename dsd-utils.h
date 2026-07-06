////////////////////////////////////////////////////////////////////////////
//                           **** DSD UTILS ****                          //
//                         Various DSD/DXD Helpers                        //
//                     Copyright (c) 2026 David Bryant                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// dsd-utils.h

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <sys/random.h>

#include "decimator.h"

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

EmbedContext *dsd_embed_init (int nchans, int flags);
void dsd_embed_run (EmbedContext *embed_context, int32_t *dst_buffer, unsigned char *src_buffer, int nsamples);
void dsd_embed_destroy (EmbedContext *embed_context);

typedef struct {
    uint64_t channel_shifter, sample_index;
    uint32_t parity_shifter;
    char locked;
} PilotDetectChannel;

typedef struct {
    uint64_t parity_masks [64];
    PilotDetectChannel *chans;
    int nchans;
} PilotDetect;

PilotDetect *PilotDetectInit (int nchans);
int PilotDetectChannelRun (PilotDetect *context, const int32_t *src_buffer, int chan, int nsamples);
void PilotDetectDestroy (PilotDetect *context);

void dsd_transition (DecimationContext *decimator, int64_t samples, unsigned char *initial_dsd, const unsigned char *final_dsd, int byte_count);
void dsd_transition_dumpstats (FILE *stream);

