#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "modulator.h"
#include "dsd-utils.h"

typedef enum { Init, Embedding, Generating, Syncing } DecoderState;

#define NUM_BUFFERS 3

typedef struct {
    char dsd_pilot_valid [NUM_BUFFERS];
    DecoderState state, next_state;
    int64_t valid_dsd_samples;
} DecoderChannel;

typedef struct {
    DecoderChannel *channels;
    PilotDetect *pilot_detector;
    Modulate *modulator;
    DecimateDSD *decimator;
    int nchans, level;

    int64_t total_dsd_samples, total_pcm_samples;
    int source_samples, composite_samples, flushing;
    unsigned char *embedded_buffer, *modulated_buffer, *composite_buffer;
    int32_t *source_buffer;
    float *float_buffer;
} Decoder;

static Decoder *decodeInit (int num_channels, int dsd_encode_level);
static int decodeProcess (Decoder *cxt, const int32_t *source, int in_samples, unsigned char *destin, int out_samples);
static int64_t decodeTotalEmbeddedSamples (Decoder *cxt);
static void decodeFree (Decoder *cxt);

static int read_24bit_samples (FILE *file, int32_t *buffer, int nchans, int samples_to_read);

#define BUFFER_SAMPLES  16384

int main (int argc, char **argv)
{
    int64_t total_dsd_samples = 0, total_pcm_samples = 0;
    unsigned char *destin_buffer;
    int nchans = 2, level = 3;
    int32_t *source_buffer;
    Decoder *decoder;

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

    decoder = decodeInit (nchans, level);
    source_buffer = calloc (sizeof (int32_t), BUFFER_SAMPLES * nchans);
    destin_buffer = calloc (1, BUFFER_SAMPLES * nchans);

    uint32_t random = 0x31415926;

    while (1) {
        int buffer_samples = BUFFER_SAMPLES;

        do {
            buffer_samples = random >> 18;
            random = ((random << 4) - random) ^ 1;
            random = ((random << 4) - random) ^ 1;
            random = ((random << 4) - random) ^ 1;
        } while (buffer_samples < 10 || buffer_samples > BUFFER_SAMPLES);

        int samples_read = read_24bit_samples (stdin, source_buffer, nchans, buffer_samples);
        int samples_generated = decodeProcess (decoder, source_buffer, samples_read ? samples_read : -1, destin_buffer, buffer_samples);

        total_pcm_samples += samples_read;
        total_dsd_samples += samples_generated;

        fwrite (destin_buffer, nchans, samples_generated, stdout);

        if (!samples_read && !samples_generated)
            break;

    }

    fprintf (stderr, "%ld total PCM samples, %ld total DSD samples, %ld were embedded (all chans, %.1f%%)\n",
        total_pcm_samples, total_dsd_samples, decodeTotalEmbeddedSamples (decoder),
        decodeTotalEmbeddedSamples (decoder) * 100.0 / total_dsd_samples / nchans);

    dsd_transition_dumpstats (stderr);

    decodeFree (decoder);
    free (source_buffer);
    free (destin_buffer);

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define BUFSAMPLES  4704

#define IDLE_LEVEL  0

static Decoder *decodeInit (int num_channels, int dsd_encode_level)
{
    Decoder *decoder = calloc (1, sizeof (Decoder));

    decoder->nchans = num_channels;
    decoder->level = dsd_encode_level;
    decoder->channels = calloc (num_channels, sizeof (DecoderChannel));
    decoder->decimator = decimateDSDinit (0, 0);
    decoder->modulator = modulateInit (num_channels, IDLE_LEVEL, MODULATE_MULTITHREADED | MODULATOR_ALIGN_EMBEDDED);
    decoder->pilot_detector = PilotDetectInit (num_channels);
    decoder->float_buffer = calloc (sizeof (float), BUFSAMPLES * num_channels);
    decoder->embedded_buffer = calloc (sizeof (char), BUFSAMPLES * num_channels);
    decoder->modulated_buffer = calloc (sizeof (char), BUFSAMPLES * num_channels);
    decoder->composite_buffer = calloc (sizeof (char), BUFSAMPLES * num_channels);
    decoder->source_buffer = calloc (sizeof (int32_t), NUM_BUFFERS * BUFSAMPLES * num_channels);

    return decoder;
}

static int decodeProcess (Decoder *cxt, const int32_t *source, int in_samples, unsigned char *destin, int out_samples)
{
    int samples_returned = 0;

    if (!source || in_samples < 0 || cxt->flushing) {
        cxt->flushing = 1;
        in_samples = 0;
    }

    while (1) {
        if (out_samples && cxt->composite_samples) {
            int samples_to_write = cxt->composite_samples;

            if (samples_to_write > out_samples)
                samples_to_write = out_samples;

            memcpy (destin, cxt->composite_buffer, samples_to_write * cxt->nchans);
            cxt->total_dsd_samples += samples_to_write * cxt->nchans;
            cxt->composite_samples -= samples_to_write;
            destin += samples_to_write * cxt->nchans;
            samples_returned += samples_to_write;
            out_samples -= samples_to_write;

            if (cxt->composite_samples)
                memmove (cxt->composite_buffer, cxt->composite_buffer + samples_to_write * cxt->nchans, cxt->composite_samples * cxt->nchans);
        }
        else if (in_samples && cxt->source_samples < BUFSAMPLES * NUM_BUFFERS) {
            int samples_to_consume = BUFSAMPLES * NUM_BUFFERS - cxt->source_samples;

            if (samples_to_consume > in_samples)
                samples_to_consume = in_samples;

            memcpy (cxt->source_buffer + cxt->source_samples * cxt->nchans, source, samples_to_consume * cxt->nchans * sizeof (int32_t));
            cxt->total_pcm_samples += samples_to_consume;
            cxt->source_samples += samples_to_consume;
            source += samples_to_consume * cxt->nchans;
            in_samples -= samples_to_consume;
        }
        else if (!cxt->composite_samples && (cxt->source_samples == BUFSAMPLES * NUM_BUFFERS || (/* cxt->source_samples && */ cxt->flushing))) {
            int samples_read = BUFSAMPLES;

            if (cxt->flushing)
                samples_read = cxt->source_samples > BUFSAMPLES * 2 ? cxt->source_samples - BUFSAMPLES * 2 : 0;

            for (int c = 0; c < cxt->nchans; ++c) {
                DecoderChannel *chan = cxt->channels + c;

                switch (chan->state) {
                    case Init:
                        chan->next_state = chan->state = Embedding;     // default to embedding until we see an invalid frame

                        for (int b = 0; b < NUM_BUFFERS; ++b)
                            if (cxt->source_samples > BUFSAMPLES * b) {
                                int scan_samples = BUFSAMPLES;

                                if (cxt->source_samples < BUFSAMPLES * (b + 1))
                                    scan_samples = cxt->source_samples - BUFSAMPLES * b;

                                chan->dsd_pilot_valid [b] = PilotDetectChannelRun (cxt->pilot_detector, cxt->source_buffer + BUFSAMPLES * b * cxt->nchans, c, scan_samples);

                                if (!chan->dsd_pilot_valid [b])
                                    chan->next_state = chan->state = Generating;
                            }
                            else
                                break;

                        if (chan->state == Generating)
                            modulateSetLevel (cxt->modulator, c, cxt->level);
                        else
                            modulateSetLevel (cxt->modulator, c, IDLE_LEVEL);

                        modulateSetAlignment (cxt->modulator, c, 0);
                        break;

                    case Generating:
                        if (samples_read) {
                            chan->dsd_pilot_valid [2] = PilotDetectChannelRun (cxt->pilot_detector, cxt->source_buffer + BUFSAMPLES * 2 * cxt->nchans, c, samples_read);

                            if (chan->dsd_pilot_valid [0] && chan->dsd_pilot_valid [1] && chan->dsd_pilot_valid [2]) {
                                modulateSetAlignment (cxt->modulator, c, 1);
                                chan->next_state = Syncing;
                            }
                        }

                        break;

                    case Syncing:
                        if (samples_read) {
                            chan->dsd_pilot_valid [2] = PilotDetectChannelRun (cxt->pilot_detector, cxt->source_buffer + BUFSAMPLES * 2 * cxt->nchans, c, samples_read);

                            if (chan->dsd_pilot_valid [2])
                                chan->next_state = Embedding;
                            else {
                                modulateSetAlignment (cxt->modulator, c, 0);
                                chan->next_state = Generating;
                            }
                        }

                        break;

                    case Embedding:
                        if (samples_read) {
                            chan->dsd_pilot_valid [2] = PilotDetectChannelRun (cxt->pilot_detector, cxt->source_buffer + BUFSAMPLES * 2 * cxt->nchans, c, samples_read);

                            if (!chan->dsd_pilot_valid [1])
                                chan->next_state = Generating;
                            else if (!chan->dsd_pilot_valid [2]) {
                                modulateSetLevel (cxt->modulator, c, cxt->level);
                                modulateSetAlignment (cxt->modulator, c, 1);
                            }
                        }

                        break;
                }
            }

            int samples_to_convert = BUFSAMPLES, samples_generated;

            if (samples_to_convert > cxt->source_samples)
                samples_to_convert = cxt->source_samples;

            for (int i = 0; i < samples_to_convert * cxt->nchans; ++i)
                cxt->float_buffer [i] = cxt->source_buffer [i] / 8388608.0;

            if (cxt->source_samples > samples_to_convert) {
                int samples_to_shift = cxt->source_samples - samples_to_convert;

                memmove (cxt->source_buffer, cxt->source_buffer + samples_to_convert * cxt->nchans, samples_to_shift * cxt->nchans * sizeof (int32_t));
                cxt->source_samples = samples_to_shift;

                for (int c = 0; c < cxt->nchans; ++c) {
                    cxt->channels [c].dsd_pilot_valid [0] = cxt->channels [c].dsd_pilot_valid [1];
                    cxt->channels [c].dsd_pilot_valid [1] = cxt->channels [c].dsd_pilot_valid [2];
                }
            }
            else
                cxt->source_samples = 0;

            samples_generated = modulateProcess (cxt->modulator, cxt->float_buffer, samples_to_convert ? samples_to_convert : -1, cxt->modulated_buffer, cxt->embedded_buffer);

            if (samples_generated) {
                unsigned char initial_buf [samples_generated], final_buf [samples_generated];

                if (cxt->composite_samples + samples_generated > BUFSAMPLES) {
                    fprintf (stderr, "attempt to generate %d composite samples\n", cxt->composite_samples + samples_generated);
                    exit (1);
                }

                for (int c = 0; c < cxt->nchans; ++c) {
                    unsigned char *composite_buffer = cxt->composite_buffer + cxt->composite_samples * cxt->nchans;
                    DecoderChannel *chan = cxt->channels + c;

                    if (chan->next_state == Embedding && chan->state == Syncing) {           // transition from generated to embedded
                        for (int i = 0; i < samples_generated; ++i) {
                            initial_buf [i] = cxt->modulated_buffer [i * cxt->nchans + c];
                            final_buf [i] = cxt->embedded_buffer [i * cxt->nchans + c];
                        }

                        dsd_transition (cxt->decimator, cxt->total_dsd_samples / cxt->nchans, initial_buf, final_buf, samples_generated);

                        for (int i = 0; i < samples_generated; ++i)
                            composite_buffer [i * cxt->nchans + c] = initial_buf [i];

                        modulateSetLevel (cxt->modulator, c, IDLE_LEVEL);
                        modulateSetAlignment (cxt->modulator, c, 0);
                    }
                    else if (chan->next_state == Generating && chan->state == Embedding) {    // transition from embedded to generated
                        for (int i = 0; i < samples_generated; ++i) {
                            initial_buf [i] = cxt->embedded_buffer [i * cxt->nchans + c];
                            final_buf [i] = cxt->modulated_buffer [i * cxt->nchans + c];
                        }

                        dsd_transition (cxt->decimator, cxt->total_dsd_samples / cxt->nchans, initial_buf, final_buf, samples_generated);

                        for (int i = 0; i < samples_generated; ++i)
                            composite_buffer [i * cxt->nchans + c] = initial_buf [i];

                        modulateSetAlignment (cxt->modulator, c, 0);
                    }
                    else if (chan->state == Embedding) {
                        for (int i = 0; i < samples_generated; ++i)
                            composite_buffer [(i * cxt->nchans) + c] = cxt->embedded_buffer [(i * cxt->nchans) + c];

                        chan->valid_dsd_samples += samples_generated;
                    }
                    else if (chan->state == Generating || chan->state == Syncing)
                        for (int i = 0; i < samples_generated; ++i)
                            composite_buffer [(i * cxt->nchans) + c] = cxt->modulated_buffer [(i * cxt->nchans) + c];

                    chan->state = chan->next_state;
                }

                cxt->composite_samples += samples_generated;
            }
            else
                break;
        }
        else
            break;
    }

    return samples_returned;
}

static int64_t decodeTotalEmbeddedSamples (Decoder *cxt)
{
    int64_t total_embedded_samples = 0;

    for (int c = 0; c < cxt->nchans; ++c)
        total_embedded_samples += cxt->channels [c].valid_dsd_samples;

    return total_embedded_samples;
}

static void decodeFree (Decoder *cxt)
{
    free (cxt->float_buffer);
    free (cxt->source_buffer);
    free (cxt->embedded_buffer);
    free (cxt->modulated_buffer);
    free (cxt->composite_buffer);
    modulateFree (cxt->modulator);
    decimateDSDdestroy (cxt->decimator);
    PilotDetectDestroy (cxt->pilot_detector);
    free (cxt->channels);
    free (cxt);
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
