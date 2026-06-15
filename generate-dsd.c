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
#include "decimator.h"

#define BUFFER_SAMPLES  4704

#ifdef STATISTICS
static double total_abs_error, total_rms_error;
static int num_transitions;
#endif

static DecimationContext *decimator;

static void dsd_transition (int64_t samples, unsigned char *initial_dsd, const unsigned char *final_dsd, int byte_count);
static int read_24bit_samples (FILE *file, float *buffer, int nchans, int samples_to_read);

int main (int argc, char **argv)
{
    int nchans = 2, depth = 4, embedded = 0, toggle = 0;
    int flags = MODULATE_MULTITHREADED;

    if (argc == 1) {
        fprintf (stderr, "Convert raw 24-bit PCM to raw DSD via 8x upsampling + delta-sigma modulation\n");
        fprintf (stderr, "Usage: generate-dsd <nchans> [<depth> [e|c] [s|m] [d|a]] < 24bit-pcm.raw > 1bit-dsd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required), <depth> = 2 to 24 (default = 4)\n");
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
        depth = atoi (argv [2]);

        if (depth < 0 || depth > 24) {
            fprintf (stderr, "depth must be 0 to 24!\n");
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

    decimator = decimate_dsd_init (0, 0);

    Modulate *modulator = modulateInit (nchans, depth, flags);
    int64_t total_input_samples = 0, total_output_samples = 0;
    unsigned char emb_output_buffer [BUFFER_SAMPLES * nchans];
    unsigned char mod_output_buffer [BUFFER_SAMPLES * nchans];
    float input_buffer [BUFFER_SAMPLES * nchans];
    int buffer_count = 0, latency_samples = 0;
    unsigned char embedded_selected [nchans];
    int depth_selected [nchans];

    memset (embedded_selected, embedded & 0x1, nchans);

    for (int c = 0; c < nchans; ++c)
        depth_selected [c] = depth;

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

#if 1       // toggle between embedded and calculated DSD every buffer
            if (toggle && buffer_count && output_generated == BUFFER_SAMPLES && buffer_count % nchans == c) {
                dsd_transition (total_output_samples, initial_buf, final_buf, output_generated);
                embedded_selected [c] ^= 1;
            }
#else       // toggle specified depth with 2 every 16 buffers
            if (toggle && buffer_count && !(buffer_count & 0xf) && ((buffer_count >> 4) % nchans) == c) {
                modulateSetDepth (modulator, c, depth_selected [c] ^= depth ^ 2);
                fprintf (stderr, "set channel %d depth to %d\n", c, depth_selected [c]);
            }
#endif

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

#ifdef STATISTICS
    if (num_transitions)
        fprintf (stderr, "%d DSD transitions, average absolute error = %.3f, average RMS error = %.3f\n",
            num_transitions, total_abs_error / num_transitions, total_rms_error / num_transitions);
#endif

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

static unsigned char xor_images [] = {
    0x00, 0x04, 0x08, 0x0C,
    0x10, 0x18,
    0x20,
    0x30,
};

#define NUM_XORS (sizeof (xor_images) / sizeof (xor_images [0]))
#define SQUARE(x) ((x)*(x))

typedef struct {
    double value, slope;
    double abs_error, rms_error;
    int abs_error_rank, rms_error_rank;
    unsigned char xor_value;
} evalPoint;

static void dsd_transition (int64_t samples, unsigned char *initial_dsd, const unsigned char *final_dsd, int byte_count)
{
    int num_eval_points = byte_count - 12, num_pcm_values = byte_count - 6, best_eval_point = 0;
    evalPoint *evalPoints = calloc (num_eval_points, sizeof (evalPoint));
    int32_t *initial_pcm = calloc (num_pcm_values, sizeof (int32_t));
    int32_t *final_pcm = calloc (num_pcm_values, sizeof (int32_t));

    for (int i = 0; i < num_pcm_values; ++i) {
        initial_pcm [i] = decimate_single_sample (decimator, initial_dsd + i);
        final_pcm [i] = decimate_single_sample (decimator, final_dsd + i);
    }

    for (int i = 0; i < num_eval_points; ++i) {
        int32_t *initial_pcm_eval = initial_pcm + i, *final_pcm_eval = final_pcm + i, target_pcm [7];
        unsigned char transition_buffer [13];
        double slope = 0.0, value = 0.0;

        for (int j = 0; j < 7; ++j) {
            target_pcm [j] = (initial_pcm_eval [j] + final_pcm_eval [j]) / 2.0;
            value += (initial_pcm_eval [j] + final_pcm_eval [j]) / 14.0;
        }

        for (int j = 0; j <= 2; ++j)
            slope += ((initial_pcm_eval [6 - j] + final_pcm_eval [6 - j]) - (initial_pcm_eval [j] + final_pcm_eval [j])) / 24.0;

        evalPoints [i].value = value;
        evalPoints [i].slope = slope;
        evalPoints [i].abs_error = FLT_MAX;

        memcpy (transition_buffer, initial_dsd + i, 6);
        memcpy (transition_buffer + 7, final_dsd + i + 7, 6);

        for (int x = 0; x < NUM_XORS; ++x) {
            double average = 0.0, rms_error = 0.0;

            transition_buffer [6] = ((initial_dsd [i + 6] & 0xF0) | (final_dsd [i + 6] & 0x0F)) ^ xor_images [x];

            for (int j = 0; j < 7; ++j) {
                double sample = decimate_single_sample (decimator, transition_buffer + j);
                rms_error += SQUARE (sample - target_pcm [j]) / 7.0;
                average += sample / 7.0;
            }

            if (fabs (average - value) < fabs (evalPoints [i].abs_error)) {
                evalPoints [i].abs_error = average - value;
                evalPoints [i].xor_value = xor_images [x];
                evalPoints [i].rms_error = rms_error;
            }
        }
    }

    for (int rank = 1; rank <= num_eval_points; ++rank) {
        double best_rms_error = FLT_MAX, best_abs_error = FLT_MAX;
        int best_abs_error_index = 0, best_rms_error_index = 0;

        for (int i = 0; i < num_eval_points; ++i) {
            if (fabs (evalPoints [i].abs_error) < best_abs_error && !evalPoints [i].abs_error_rank) {
                best_abs_error = fabs (evalPoints [i].abs_error);
                best_abs_error_index = i;
            }

            if (evalPoints [i].rms_error < best_rms_error && !evalPoints [i].rms_error_rank) {
                best_rms_error = evalPoints [i].rms_error;
                best_rms_error_index = i;
            }
        }

        evalPoints [best_abs_error_index].abs_error_rank = rank;
        evalPoints [best_rms_error_index].rms_error_rank = rank;

        if (evalPoints [best_abs_error_index].rms_error_rank) {
            if (evalPoints [best_rms_error_index].abs_error_rank &&
                evalPoints [best_rms_error_index].abs_error_rank < evalPoints [best_abs_error_index].rms_error_rank)
                    best_eval_point = best_rms_error_index;
            else
                best_eval_point = best_abs_error_index;

            break;
        }
        else if (evalPoints [best_rms_error_index].abs_error_rank) {
            best_eval_point = best_rms_error_index;
            break;
        }
    }

#ifdef STATISTICS
    fprintf (stderr, "time = %8.5f (%4d, 0x%02x): y,m = %6.0f,%5.0f, abs error = %7.2f (%3d), rms error = %7.2f (%3d)\n",
        (samples + best_eval_point + 6) / 352800.0, best_eval_point, evalPoints [best_eval_point].xor_value,
        evalPoints [best_eval_point].value / 256.0,
        evalPoints [best_eval_point].slope / 256.0,
        evalPoints [best_eval_point].abs_error / 256.0, evalPoints [best_eval_point].abs_error_rank,
        sqrt (evalPoints [best_eval_point].rms_error) / 256.0, evalPoints [best_eval_point].rms_error_rank),

    total_abs_error += fabs (evalPoints [best_eval_point].abs_error) / 256.0;
    total_rms_error += sqrt (evalPoints [best_eval_point].rms_error) / 256.0;
    num_transitions++;
#endif

    initial_dsd [best_eval_point + 6] &= 0xF0;
    initial_dsd [best_eval_point + 6] |= final_dsd [best_eval_point + 6] & 0x0F;
    initial_dsd [best_eval_point + 6] ^= evalPoints [best_eval_point].xor_value;
    memcpy (initial_dsd + best_eval_point + 7, final_dsd + best_eval_point + 7, byte_count - best_eval_point - 7);

    free (evalPoints);
    free (initial_pcm);
    free (final_pcm);
}
