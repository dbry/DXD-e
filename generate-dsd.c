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
#include "dsd-utils.h"

#define BUFFER_SAMPLES  4704

static int read_24bit_samples (FILE *file, float *buffer, int nchans, int samples_to_read);

int main (int argc, char **argv)
{
    int nchans = 2, level = 3, embedded = 0, toggle = 0;
    int flags = MODULATE_MULTITHREADED;

    if (argc == 1) {
        fprintf (stderr, "Convert raw 24-bit PCM to raw DSD via 8x upsampling + delta-sigma modulation\n");
        fprintf (stderr, "Usage: generate-dsd <nchans> [<level> [e|c] [s|m] [d|a]] < 24bit-pcm.raw > 1bit-dsd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required), <level> = 1 to 5 (default = 3)\n");
        fprintf (stderr, "       e = output embedded DSD, c = output calculated DSD (default), t = toggle\n");
        fprintf (stderr, "       d = drift (default allows streams to drift), a = align streams\n");
        fprintf (stderr, "       s = single-threaded, m = multi-threaded (default)\n");
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
        level = atoi (argv [2]);

        if (level < 1 || level > 5) {
            fprintf (stderr, "level must be 1 to 5!\n");
            return 1;
        }

        for (int argi = 3; argi < argc; ++argi) {
            if (strlen (argv [argi]) == 1 && (*argv [argi] == 's' || *argv [argi] == 'S'))
                flags &= ~MODULATE_MULTITHREADED;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'm' || *argv [argi] == 'M'))
                flags |= MODULATE_MULTITHREADED;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'e' || *argv [argi] == 'E'))
                embedded = 1;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'c' || *argv [argi] == 'C'))
                embedded = 0;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 't' || *argv [argi] == 'T'))
                toggle = 1;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'a' || *argv [argi] == 'A'))
                flags |= MODULATOR_ALIGN_EMBEDDED;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'd' || *argv [argi] == 'D'))
                flags &= ~MODULATOR_ALIGN_EMBEDDED;
            else {
                fprintf (stderr, "unknown argument: %s\n", argv [argi]);
                return 1;
            }
        }
    }

    DecimateDSD *decimator = decimateDSDinit (0, 0);
    Modulate *modulator = modulateInit (nchans, level, flags);
    int64_t total_input_samples = 0, total_output_samples = 0;
    unsigned char emb_output_buffer [BUFFER_SAMPLES * nchans];
    unsigned char mod_output_buffer [BUFFER_SAMPLES * nchans];
    float input_buffer [BUFFER_SAMPLES * nchans];
    int buffer_count = 0, latency_samples = 0;
    unsigned char embedded_selected [nchans];

    memset (embedded_selected, embedded & 0x1, nchans);

    while (1) {
        int samples_read = read_24bit_samples (stdin, input_buffer, nchans, BUFFER_SAMPLES);
        int output_generated = modulateProcess (modulator, input_buffer, samples_read ? samples_read : -1, mod_output_buffer, emb_output_buffer);

        if (total_output_samples == 0 && output_generated) {
            latency_samples = BUFFER_SAMPLES - output_generated;
            fprintf (stderr, "initial modulator call generated %d samples, latency = %d samples\n", output_generated, latency_samples);
        }

        for (int c = 0; c < nchans; ++c) {
            unsigned char *initial_ptr = (embedded_selected [c] ? emb_output_buffer : mod_output_buffer) + c;
            unsigned char *final_ptr = (embedded_selected [c] ? mod_output_buffer : emb_output_buffer) + c;
            unsigned char initial_buf [output_generated], final_buf [output_generated];
            unsigned char *output_ptr = emb_output_buffer + c;

            for (int i = 0; i < output_generated; ++i) {
                initial_buf [i] = initial_ptr [i * nchans];
                final_buf [i] = final_ptr [i * nchans];
            }

            if (toggle && buffer_count && output_generated == BUFFER_SAMPLES && buffer_count % nchans == c) {
                dsd_transition (decimator, total_output_samples, initial_buf, final_buf, output_generated);
                embedded_selected [c] ^= 1;
            }

            for (int i = 0; i < output_generated; ++i)
                output_ptr [i * nchans] = initial_buf [i];
        }

        total_output_samples += output_generated;
        total_input_samples += samples_read;
        buffer_count++;

        fwrite (emb_output_buffer, 1, output_generated * nchans, stdout);

        if (!samples_read)
            break;
    }

    fprintf (stderr, "total samples: %ld read, %ld written\n", total_input_samples, total_output_samples);
    dsd_transition_dumpstats (stderr);
    decimateDSDdestroy (decimator);
    modulateFree (modulator);

    return 0;
}

static int read_24bit_samples (FILE *file, float *buffer, int nchans, int samples_to_read)
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
