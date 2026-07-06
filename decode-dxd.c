#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "modulator.h"
#include "dsd-utils.h"

#define BUFSAMPLES  4704
#define NUM_BUFFERS 3

#define IDLE_LEVEL  1

typedef enum { Init, Embedding, Generating, Syncing } DecoderState;

typedef struct {
    char dsd_pilot_valid [NUM_BUFFERS];
    DecoderState state, next_state;
} DecoderChannel;

static int read_24bit_samples (FILE *file, int32_t *buffer, int nchans, int samples_to_read);

int main (int argc, char **argv)
{
    int total_dsd_samples = 0, total_pcm_samples = 0, valid_dsd_samples = 0, source_samples = 0;
    unsigned char *embedded_buffer, *modulated_buffer, *composite_buffer;
    PilotDetect *pilot_detector;
    int nchans = 2, level = 3;
    int32_t *source_buffer;
    Modulate *modulator;
    float *float_buffer;
    void *decimator;

    if (argc == 1) {
        fprintf (stderr, "Convert raw 24-bit DXD-e to raw DSD via DSD verification/extraction/modulation\n");
        fprintf (stderr, "Usage: decode-dsd <nchans> [level] < 24bit-dxd.raw > 1bit-dsd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required), <level> = 1 to 5 (default = 3)\n");
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

    if (argc > 2) {
        level = atoi (argv [2]);

        if (level < 1 || level > 5) {
            fprintf (stderr, "level must be 1 to 5!\n");
            return 1;
        }
    }

    decimator = decimate_dsd_init (0, 0);
    source_buffer = calloc (sizeof (int32_t), NUM_BUFFERS * BUFSAMPLES * nchans);
    float_buffer = calloc (sizeof (float), BUFSAMPLES * nchans);
    embedded_buffer = calloc (sizeof (char), BUFSAMPLES * nchans);
    modulated_buffer = calloc (sizeof (char), BUFSAMPLES * nchans);
    composite_buffer = calloc (sizeof (char), BUFSAMPLES * nchans);
    modulator = modulateInit (nchans, IDLE_LEVEL, MODULATE_MULTITHREADED | MODULATOR_ALIGN_EMBEDDED);
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
                            modulateSetLevel (modulator, c, level);

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
                                modulateSetLevel (modulator, c, level);
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

                    dsd_transition (decimator, total_dsd_samples / nchans, initial_buf, final_buf, samples_generated);

                    for (int i = 0; i < samples_generated; ++i)
                        composite_buffer [i * nchans + c] = initial_buf [i];

                    modulateSetLevel (modulator, c, IDLE_LEVEL);
                    modulateSetAlignment (modulator, c, 0);
                }
                else if (chan->next_state == Generating && chan->state == Embedding) {    // transition from embedded to generated
                    for (int i = 0; i < samples_generated; ++i) {
                        initial_buf [i] = embedded_buffer [i * nchans + c];
                        final_buf [i] = modulated_buffer [i * nchans + c];
                    }

                    dsd_transition (decimator, total_dsd_samples / nchans, initial_buf, final_buf, samples_generated);

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

    dsd_transition_dumpstats (stderr);

    modulateFree (modulator);
    decimate_dsd_destroy (decimator);
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
