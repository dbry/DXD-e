#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "modulator.h"
#include "decimator.h"

#define PILOT_SEQUENCE 0xf123456789abcde0
#define BUFSAMPLES  4704
#define NUM_BUFFERS 3

#define IDLE_DEPTH  2
#define GEN_DEPTH   8

typedef enum { Init, Embedding, Generating, Syncing } DecoderState;

typedef struct {
    char dsd_pilot_valid [NUM_BUFFERS];
    DecoderState state, next_state;
} DecoderChannel;

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

static PilotDetect *PilotDetectInit (int nchans);
static void PilotDetectDestroy (PilotDetect *context);
static int PilotDetectChannelRun (PilotDetect *context, const int32_t *src_buffer, int chan, int nsamples);
static int read_24bit_samples (FILE *file, int32_t *buffer, int nchans, int samples_to_read);
static void dsd_transition (int64_t samples, unsigned char *initial_dsd, const unsigned char *final_dsd, int byte_count);

#ifdef STATISTICS
static double total_abs_error, total_rms_error;
static int num_transitions;
#endif

int main (int argc, char **argv)
{
    int total_dsd_samples = 0, total_pcm_samples = 0, valid_dsd_samples = 0, source_samples = 0;
    unsigned char *embedded_buffer, *modulated_buffer, *composite_buffer;
    PilotDetect *pilot_detector;
    int32_t *source_buffer;
    Modulate *modulator;
    float *float_buffer;
    int nchans = 2;

    if (argc == 1) {
        fprintf (stderr, "Convert raw 24-bit DXD-e to raw DSD via DSD verification/extraction/modulation\n");
        fprintf (stderr, "Usage: decode-dsd <nchans> < 24bit-dxd.raw > 1bit-dsd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required)\n");
        fprintf (stderr, " Note: areas with missing or corrupted embedded DSD get newly modulated DSD\n");
        return 0;
    }

    if (argc > 1) {
        nchans = atoi (argv [1]);

        if (nchans < 1 || nchans > 16) {
            fprintf (stderr, "must be 1 to 16 channels!\n");
            return 1;
        }
    }

    source_buffer = calloc (sizeof (int32_t), NUM_BUFFERS * BUFSAMPLES * nchans);
    float_buffer = calloc (sizeof (float), BUFSAMPLES * nchans);
    embedded_buffer = calloc (sizeof (char), BUFSAMPLES * nchans);
    modulated_buffer = calloc (sizeof (char), BUFSAMPLES * nchans);
    composite_buffer = calloc (sizeof (char), BUFSAMPLES * nchans);
    modulator = modulateInit (nchans, IDLE_DEPTH, MODULATE_MULTITHREADED | MODULATOR_ALIGN_EMBEDDED);
    pilot_detector = PilotDetectInit (nchans);

    DecoderChannel channels [nchans] = {};

    while (1) {
        int samples_to_read = BUFSAMPLES * NUM_BUFFERS - source_samples, samples_to_convert;
        int samples_read = read_24bit_samples (stdin, source_buffer + source_samples * nchans, nchans, samples_to_read);
        int samples_generated;

        total_pcm_samples += samples_read;
        source_samples += samples_read;

        if (source_samples) {
            for (int c = 0; c < nchans; ++c) {
                DecoderChannel *chan = channels + c;

                switch (chan->state) {
                    case Init:
                        chan->next_state = chan->state = Embedding;     // default to embedding until we see an invalid frame

                        if (samples_read >= BUFSAMPLES) {
                            chan->dsd_pilot_valid [0] = PilotDetectChannelRun (pilot_detector, source_buffer, c, BUFSAMPLES);

                            if (!chan->dsd_pilot_valid [0])
                                chan->next_state = chan->state = Generating;
                        }

                        if (samples_read >= BUFSAMPLES * 2) {
                            chan->dsd_pilot_valid [1] = PilotDetectChannelRun (pilot_detector, source_buffer + BUFSAMPLES * nchans, c, BUFSAMPLES);

                            if (!chan->dsd_pilot_valid [1])
                                chan->next_state = chan->state = Generating;
                        }

                        if (samples_read >= BUFSAMPLES * 3) {
                            chan->dsd_pilot_valid [2] = PilotDetectChannelRun (pilot_detector, source_buffer + BUFSAMPLES * 2 * nchans, c, BUFSAMPLES);

                            if (!chan->dsd_pilot_valid [2])
                                chan->next_state = chan->state = Generating;
                        }

                        if (chan->state == Generating)
                            modulateSetDepth (modulator, c, GEN_DEPTH);

                        break;

                    case Generating:
                        if (samples_read) {
                            chan->dsd_pilot_valid [2] = PilotDetectChannelRun (pilot_detector, source_buffer + BUFSAMPLES * 2 * nchans, c, samples_read);

                            if (chan->dsd_pilot_valid [0] && chan->dsd_pilot_valid [1] && chan->dsd_pilot_valid [2]) {
                                modulateSetAlignment (modulator, c, 1);
                                chan->next_state = Syncing;
                            }
                        }

                        break;

                    case Syncing:
                        if (samples_read) {
                            chan->dsd_pilot_valid [2] = PilotDetectChannelRun (pilot_detector, source_buffer + BUFSAMPLES * 2 * nchans, c, samples_read);

                            if (chan->dsd_pilot_valid [2])
                                chan->next_state = Embedding;
                            else {
                                modulateSetAlignment (modulator, c, 0);
                                chan->next_state = Generating;
                            }
                        }

                        break;

                    case Embedding:
                        if (samples_read) {
                            chan->dsd_pilot_valid [2] = PilotDetectChannelRun (pilot_detector, source_buffer + BUFSAMPLES * 2 * nchans, c, samples_read);

                            if (!chan->dsd_pilot_valid [1])
                                chan->next_state = Generating;
                            else if (!chan->dsd_pilot_valid [2]) {
                                modulateSetDepth (modulator, c, GEN_DEPTH);
                                modulateSetAlignment (modulator, c, 1);
                            }
                        }

                        break;
                }
            }

            for (int i = 0; i < BUFSAMPLES * nchans; ++i)
                float_buffer [i] = source_buffer [i] / 8388608.0;
        }

        samples_to_convert = BUFSAMPLES;

        if (samples_to_convert > source_samples)
            samples_to_convert = source_samples;

        samples_generated = modulateProcess (modulator, float_buffer, samples_to_convert ? samples_to_convert : -1, modulated_buffer, embedded_buffer);

        if (samples_generated) {
            unsigned char initial_buf [samples_generated], final_buf [samples_generated];

            for (int c = 0; c < nchans; ++c) {
                DecoderChannel *chan = channels + c;

                if (chan->next_state == Embedding && chan->state == Syncing) {           // transition from generated to embedded
                    for (int i = 0; i < samples_generated; ++i) {
                        initial_buf [i] = modulated_buffer [i * nchans + c];
                        final_buf [i] = embedded_buffer [i * nchans + c];
                    }

                    dsd_transition (total_dsd_samples / nchans, initial_buf, final_buf, samples_generated);

                    for (int i = 0; i < samples_generated; ++i)
                        composite_buffer [i * nchans + c] = initial_buf [i];

                    modulateSetDepth (modulator, c, IDLE_DEPTH);
                    modulateSetAlignment (modulator, c, 0);
                }
                else if (chan->next_state == Generating && chan->state == Embedding) {    // transition from embedded to generated
                    for (int i = 0; i < samples_generated; ++i) {
                        initial_buf [i] = embedded_buffer [i * nchans + c];
                        final_buf [i] = modulated_buffer [i * nchans + c];
                    }

                    dsd_transition (total_dsd_samples / nchans, initial_buf, final_buf, samples_generated);

                    for (int i = 0; i < samples_generated; ++i)
                        composite_buffer [i * nchans + c] = initial_buf [i];

                    modulateSetAlignment (modulator, c, 0);
                }
                else if (chan->state == Embedding) {
                    for (int i = 0; i < samples_generated; ++i)
                        composite_buffer [(i * nchans) + c] = embedded_buffer [(i * nchans) + c];

                    valid_dsd_samples += samples_generated;
                }
                else if (chan->state == Generating || chan->state == Syncing)
                    for (int i = 0; i < samples_generated; ++i)
                        composite_buffer [(i * nchans) + c] = modulated_buffer [(i * nchans) + c];

                chan->state = chan->next_state;
            }

            fwrite (composite_buffer, nchans, samples_generated, stdout);
            total_dsd_samples += samples_generated * nchans;
        }

        if (source_samples > samples_to_convert) {
            int samples_to_shift = source_samples - samples_to_convert;

            memmove (source_buffer, source_buffer + samples_to_convert * nchans, samples_to_shift * nchans * sizeof (int32_t));
            source_samples = samples_to_shift;

            for (int c = 0; c < nchans; ++c) {
                channels [c].dsd_pilot_valid [0] = channels [c].dsd_pilot_valid [1];
                channels [c].dsd_pilot_valid [1] = channels [c].dsd_pilot_valid [2];
            }
        }
        else
            source_samples = 0;

        if (!source_samples && !samples_generated)
            break;
    }

    fprintf (stderr, "%d total PCM samples, %d total DSD samples, %d were valid (%.1f%%)\n",
        total_pcm_samples, total_dsd_samples, valid_dsd_samples, valid_dsd_samples * 100.0 / total_dsd_samples);

#ifdef STATISTICS
    if (num_transitions)
        fprintf (stderr, "%d DSD transitions, average absolute error = %.3f, average RMS error = %.3f\n",
            num_transitions, total_abs_error / num_transitions, total_rms_error / num_transitions);
#endif

    modulateFree (modulator);
    PilotDetectDestroy (pilot_detector);
    free (source_buffer);
    free (float_buffer);
    free (embedded_buffer);
    free (modulated_buffer);
    free (composite_buffer);

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int read_24bit_samples (FILE *file, int32_t *buffer, int nchans, int samples_to_read)
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
            *buffer++ = sample24 >> 8;
        }

        samples_read++;
    }

    return samples_read;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PilotDetect *PilotDetectInit (int nchans)
{
    PilotDetect *context = (PilotDetect *) calloc (1, sizeof (PilotDetect));
    uint64_t shifter = PILOT_SEQUENCE;

    context->nchans = nchans;

    for (int i = 0; i < 64; ++i) {
        context->parity_masks [i] = shifter;
        shifter = (shifter << 1) | ((shifter >> 63) & 1);
    }

    context->chans = calloc (sizeof (PilotDetectChannel), nchans);

    return context;
}

static int PilotDetectChannelRun (PilotDetect *context, const int32_t *src_buffer, int chan, int nsamples)
{
    PilotDetectChannel *chanptr = context->chans + chan;
    int retval = chanptr->locked;

    for (int index = 0; index < nsamples; ++index) {
        chanptr->parity_shifter = (chanptr->parity_shifter << 1) | __builtin_parity (src_buffer [(index * context->nchans) + chan]);
        chanptr->channel_shifter = (chanptr->channel_shifter << 1) | __builtin_parity (chanptr->parity_shifter & 0x80002001);

        if (chanptr->locked) {
            chanptr->locked = ((chanptr->locked + 1) & 0x3f) | 0x40;
            if (chanptr->channel_shifter != context->parity_masks [chanptr->locked & 0x3f]) {
                // fprintf (stderr, "%d:  unlocked: %.4f (%d/%d)\n", chan, chanptr->sample_index / 352800.0, index, nsamples);
                retval = chanptr->locked = 0;
            }
        }
        else
            for (int i = 0; i < 64; ++i)
                if (chanptr->channel_shifter == context->parity_masks [i]) {
                    if (chanptr->sample_index <= 94) {
                        // fprintf (stderr, "%d: prelocked: %.4f (%d/%d)\n", chan, chanptr->sample_index / 352800.0, index, nsamples);
                        retval = 1;
                    }
                    // else
                    //     fprintf (stderr, "%d:    locked: %.4f (%d/%d)\n", chan, chanptr->sample_index / 352800.0, index, nsamples);

                    chanptr->locked = i | 0x40;
                    break;
                }

        chanptr->sample_index++;
    }

    return retval;
}

static void PilotDetectDestroy (PilotDetect *context)
{
    free (context->chans);
    free (context);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    static DecimationContext *decimator;

    if (!decimator)
        decimator = decimate_dsd_init (0, 0);

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
