////////////////////////////////////////////////////////////////////////////
//                          **** GENERATE-DSD ****                        //
//                       Simple PCM to DSD Modulator                      //
//                     Copyright (c) 2026 David Bryant                    //
//                          All Rights Reserved.                          //
//      Distributed under the BSD Software License (see license.txt)      //
////////////////////////////////////////////////////////////////////////////

// generate-dsd.c

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <math.h>

#include "modulator.h"

int read_24bit_samples (FILE *file, float *buffer, int nchans, int samples_to_read)
{
    int samples_read = 0;

    while (samples_to_read--) {
        unsigned char raw_buffer [nchans * 3], *raw_ptr = raw_buffer;
        int t;

        if ((t = fread (raw_buffer, 3, nchans, file)) != nchans) {
            return samples_read;
        }

        for (int c = 0; c < nchans; ++c) {
            int32_t sample24 = *raw_ptr++ << 8;

            sample24 += *raw_ptr++ << 16;
            sample24 += *raw_ptr++ << 24;
            *buffer++ = (sample24 >> 8) / 8388608.0;
        }

        samples_read++;
    }

    return samples_read;
}

#define OUTPUT_SAMPLES  256
#define INPUT_SAMPLES   256

int main (int argc, char **argv)
{
    int nchans = 2, depth = 4;

    if (argc == 1) {
        fprintf (stderr, "Convert raw 24-bit PCM to raw DSD via 8x upsampling + delta-sigma modulation\n");
        fprintf (stderr, "Usage: generate-dsd <nchans> [<depth>] < 24bit-pcm.raw > 1bit-dsd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required), <depth> = 2 to 24 (default = 4)\n");
        return 0;
    }

    if (argc > 1) {
        nchans = atoi (argv [1]);

        if (nchans < 1 || nchans > 16) {
            fprintf (stderr, "must be 1 to 16 channels!\n");
            return 1;
        }
    }

    if (argc > 2) {
        depth = atoi (argv [2]);

        if (depth < 0 || depth > 24) {
            fprintf (stderr, "depth must be 0 to 24!\n");
            return 1;
        }
    }

    Modulate *modulator = modulateInit (nchans, depth);
    int64_t total_input_samples = 0, total_output_samples = 0;
    unsigned char output_buffer [OUTPUT_SAMPLES * nchans];
    float input_buffer [INPUT_SAMPLES * nchans];
    int input_exhausted = 0;

    while (!input_exhausted) {
        int samples_ready = read_24bit_samples (stdin, input_buffer, nchans, INPUT_SAMPLES);
        float *input_ptr = input_buffer;

        input_exhausted = !samples_ready;

        while (1) {
            ModulateResult res = modulateProcess (modulator, input_ptr, input_exhausted ? -1 : samples_ready, output_buffer, OUTPUT_SAMPLES);

            if (!res.input_used && !res.output_generated)
                break;

            total_output_samples += res.output_generated;
            total_input_samples += res.input_used;
            input_ptr += res.input_used * nchans;
            samples_ready -= res.input_used;

            fwrite (output_buffer, 1, res.output_generated * nchans, stdout);

            if (!samples_ready && res.output_generated < OUTPUT_SAMPLES)
                break;
        }
    }

    fprintf (stderr, "total samples: %ld read, %ld written\n", total_input_samples, total_output_samples);
    modulateFree (modulator);
    return 0;
}
