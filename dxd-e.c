#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <wavpack/wavpack.h>

#include "modulator.h"
#include "dsd-utils.h"

// #define THRESHOLD 0.501171

static int analyze_file (FILE *output, char *filename);

int main (int argc, char **argv)
{
    char *infilename = NULL, *outfilename = NULL;

    if (argc == 1) {
        fprintf (stderr, "Process WavPack files to/from DSD to DXD\n");
        fprintf (stderr, "Usage: dxd-e [-options] infile.wv [outfile.wv]\n");
        return 0;
    }

    // loop through command-line arguments

    while (--argc) {
        if (**++argv == '-' && (*argv)[1] == '-' && (*argv)[2]) {
            char *long_option = *argv + 2, *long_param = long_option;

            while (*long_param)
                if (*long_param++ == '=')
                    break;

            // if (!strncmp (long_option, "pitch", 5)) {                   // --pitch
            //     double pitch_cents = strtod (long_param, NULL);

            //     if (pitch_cents < -2400 || pitch_cents > 2400) {
            //         fprintf (stderr, "invalid pitch shift, must be +/- 2400 cents (2 octaves)!\n");
            //         return 1;
            //     }

            //     pitch_ratio = pow (2.0, pitch_cents / 1200.0);
            // }
            // else {
                fprintf (stderr, "unknown option: %s !\n", long_option);
                return 1;
            // }
        }
#if defined (_WIN32)
        else if ((**argv == '-' || **argv == '/') && (*argv)[1])
#else
        else if ((**argv == '-') && (*argv)[1])
#endif
            while (*++*argv)
                switch (**argv) {

                    // case 'Y': case 'y':
                    //     overwrite = 1;
                    //     break;

                    default:
                        fprintf (stderr, "\nillegal option: %c !\n", **argv);
                        return 1;
                }
        else if (!infilename) {
            infilename = malloc (strlen (*argv) + 10);
            strcpy (infilename, *argv);
        }
        else if (!outfilename) {
            outfilename = malloc (strlen (*argv) + 10);
            strcpy (outfilename, *argv);
        }
        else {
            fprintf (stderr, "\nextra unknown argument: %s !\n", *argv);
            return 1;
        }
    }

    if (!infilename) {
        fprintf (stderr, "must specify at least one file!\n");
        return 1;
    }

    return analyze_file (stdout, infilename);
}

#define BUFFER_SAMPLES  1048576

typedef struct {
    float min_value, max_value;
    double value_sum, abs_value_sum, rms_sum, rms_level, peak_rms_level;
    int64_t num_samples, magnitude_histogram [100], threshold;
    int valid_dsd_sectors, no_dsd_sectors;
} ChannelData;

static void analyze_float_data (float *src, int num_samples, int num_channels, int sample_rate, ChannelData *chan_data);
static void float_integer_data (int32_t *src, float *dst, int num_samples, int bps);
static double population_to_magnitude (double population, ChannelData *chan_data);
static char *string_channel (int channel, int channel_mask, int center_width);
static char *string_time (double seconds);

static int analyze_file (FILE *output, char *filename)
{
    char error [80], format [80];
    int flags = OPEN_NORMALIZE | OPEN_DSD_NATIVE | (4 << OPEN_THREADS_SHFT);
    WavpackContext *cxt = WavpackOpenFileInput (filename, error, flags, 0);
    int dsd = 0, dxd = 0;

    if (!cxt) {
        fprintf (output, "can't open file %s: %s\n", filename, error);
        return 1;
    }

    int64_t total_samples = WavpackGetNumSamples64 (cxt);
    int bytes_per_sample = WavpackGetBytesPerSample (cxt);
    int bits_per_sample = WavpackGetBitsPerSample (cxt);
    uint32_t channel_mask = WavpackGetChannelMask (cxt);
    int num_channels = WavpackGetNumChannels (cxt);
    int sample_rate = WavpackGetSampleRate (cxt);
    int qmode = WavpackGetQualifyMode (cxt);
    int mode = WavpackGetMode (cxt);

    if (qmode & QMODE_DSD_AUDIO) {
        if (sample_rate % 44100 == 0) {
            dsd = sample_rate * 8 / 44100;
            sprintf (format, "DSD%d", dsd);
        }
        else if (sample_rate % 48000 == 0) {
            dsd = sample_rate * 8 / 48000;
            sprintf (format, "DSD%d (%f MHz)", dsd, sample_rate / 125000.0);
        }
        else {
            dsd = -1;
            sprintf (format, "DSD @ %f MHz", sample_rate / 125000.0);
        }
    }
    else if (bits_per_sample >= 24 && (sample_rate % 352800 == 0 || sample_rate % 384000 == 0)) {
        dxd = sample_rate / 1000;
        sprintf (format, "DXD%d", dxd);
    }
    else
        sprintf (format, "PCM");

    int sector_samples = sample_rate / 75;
    int buffer_samples = BUFFER_SAMPLES / sector_samples * sector_samples;

    ChannelData *chan_data = calloc (num_channels, sizeof (ChannelData));
    int32_t *source_buffer = malloc (buffer_samples * sizeof (int32_t) * num_channels);
    float *float_buffer = malloc (buffer_samples * sizeof (float) * num_channels);
    int64_t samples_remaining = total_samples, samples_processed = 0;
    unsigned char *dsd_samples = NULL;
    PilotDetect *detector = NULL;
    DecimateDSD *decimator;

    if (qmode & QMODE_DSD_AUDIO) {
        dsd_samples = malloc (buffer_samples * sizeof (char) * num_channels);
        decimator = decimateDSDinit (num_channels, 0);
    }

    if (dxd)
        detector = PilotDetectInit (num_channels);

    while (1) {
        int samples_to_read = buffer_samples, samples_read, samples_ready;

        if (samples_to_read > samples_remaining)
            samples_to_read = samples_remaining;

        samples_read = WavpackUnpackSamples (cxt, source_buffer, samples_to_read);

        if (samples_read != samples_to_read) {
            fprintf (output, "file exhausted prematurely!\n");
            return 1;
        }

        samples_remaining -= samples_read;
        samples_ready = samples_read;

        if (qmode & QMODE_DSD_AUDIO) {
            int dsd_samples_ready = samples_ready * num_channels;

            for (int i = 0; i < dsd_samples_ready; ++i)
                dsd_samples [i] = source_buffer [i] & 0xff;

            samples_ready = decimateDSDrun (decimator, dsd_samples, samples_ready ? samples_ready : -1, source_buffer);

            if (!samples_ready)
                break;

            float_integer_data (source_buffer, float_buffer, samples_ready * num_channels, 3);
        }
        else {
            if (!samples_ready)
                break;

            if (mode & MODE_FLOAT) {
                memcpy (float_buffer, source_buffer, sizeof (float) * samples_ready * num_channels);

                if (dxd)
                    for (int i = 0; i < samples_ready * num_channels; ++i)
                        source_buffer [i] = float_buffer [i] * 8388608.0;
            }
            else
                float_integer_data (source_buffer, float_buffer, samples_ready * num_channels, bytes_per_sample);
        }

        samples_processed += samples_ready;

        analyze_float_data (float_buffer, samples_ready, num_channels, sample_rate, chan_data);

        if (dxd)
            for (int chan = 0; chan < num_channels; ++chan) {
                int samples_to_scan = samples_ready;
                int buffer_index = 0;

                while (samples_to_scan) {
                    int samples = sector_samples;

                    if (samples > samples_to_scan)
                        samples = samples_to_scan;

                    if (PilotDetectChannelRun (detector, source_buffer + buffer_index, chan, samples))
                        chan_data [chan].valid_dsd_sectors++;
                    else
                        chan_data [chan].no_dsd_sectors++;

                    buffer_index += samples * num_channels;
                    samples_to_scan -= samples;
                }
            }
    }

    fprintf (output, "\n\n");
    fprintf (output, "%16s:%12d\n", "channels", num_channels);
    fprintf (output, "%16s:%12s\n", "format", format);
    if (mode & MODE_FLOAT)
        fprintf (output, "%16s:%12s\n", "bit depth", "32-bit FLT");
    else if (dsd)
        fprintf (output, "%16s:%12s\n", "bit depth", "1-bit DSD");
    else
        fprintf (output, "%16s:%8d-bit\n", "bit depth", bits_per_sample);
    fprintf (output, "%16s:%12d\n", "sample rate", dsd ? sample_rate * 8 : sample_rate);
    fprintf (output, "%16s:%12s\n", "duration", string_time ((double) total_samples / sample_rate));
    fprintf (output, "\n");

    if (num_channels > 1) {
        fprintf (output, "%16s:  ", "channels");
        for (int chan = 0; chan < num_channels; ++chan)
            fputs (string_channel (chan, channel_mask, 12), output);
        fprintf (output, "\n");

        for (int i = 0; i < 17 + num_channels * 12; ++i)
            fputc ('-', output);

        fprintf (output, "\n");
    }

    if (dxd) {
        fprintf (output, "%16s:", "DSD detected");
        for (int chan = 0; chan < num_channels; ++chan)
            if (!chan_data [chan].valid_dsd_sectors)
                fprintf (output, "%12s", "** no **");
            else if (!chan_data [chan].no_dsd_sectors)
                fprintf (output, "%12s", "** yes **");
            else
                fprintf (output, "%11.2f%%",
                    chan_data [chan].valid_dsd_sectors * 100.0 / (chan_data [chan].no_dsd_sectors + chan_data [chan].valid_dsd_sectors));
        fprintf (output, "\n");

        // fprintf (output, "%16s:", "invalid sectors");
        // for (int chan = 0; chan < num_channels; ++chan)
        //     fprintf (output, "%12d", chan_data [chan].no_dsd_sectors);
        // fprintf (output, "\n");

        // fprintf (output, "%16s:", "valid sectors");
        // for (int chan = 0; chan < num_channels; ++chan)
        //     fprintf (output, "%12d", chan_data [chan].valid_dsd_sectors);
        // fprintf (output, "\n");
    }

    fprintf (output, "%16s:", "min value");
    for (int chan = 0; chan < num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].min_value);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "max value");
    for (int chan = 0; chan < num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].max_value);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "ave value");
    for (int chan = 0; chan < num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].value_sum / chan_data [chan].num_samples);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "ave magnitude");
    for (int chan = 0; chan < num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].abs_value_sum / chan_data [chan].num_samples);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "99.9% magnitude");
    for (int chan = 0; chan < num_channels; ++chan)
        fprintf (output, "%12f", population_to_magnitude (0.999, chan_data + chan));
    fprintf (output, "\n");

#ifdef THRESHOLD
    fprintf (output, "%16s:", "threshold");
    for (int chan = 0; chan < num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].threshold * 100.0 / chan_data [chan].num_samples);
    fprintf (output, "   (%.6f)", THRESHOLD);
    fprintf (output, "\n");
#endif

    fprintf (output, "%16s:", "rms level");
    for (int chan = 0; chan < num_channels; ++chan)
        fprintf (output, "%9.2f dB", log10 (chan_data [chan].rms_sum / chan_data [chan].num_samples / 0.5) * 10);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "peak level");
    for (int chan = 0; chan < num_channels; ++chan)
        fprintf (output, "%9.2f dB", log10 (chan_data [chan].peak_rms_level / 0.5) * 10);
    fprintf (output, "\n\n");

    WavpackCloseFile (cxt);
    free (source_buffer);
    free (float_buffer);
    free (chan_data);

    if (qmode & QMODE_DSD_AUDIO) {
        decimateDSDdestroy (decimator);
        free (dsd_samples);
    }

    if (dxd)
        PilotDetectDestroy (detector);

    fprintf (output, "read %ld samples successfully, processed %ld samples\n", total_samples, samples_processed);
    return 0;
}

static const char *speakers [] = {
    "FL", "FR", "FC", "LFE", "BL", "BR", "FLC", "FRC", "BC",
    "SL", "SR", "TC", "TFL", "TFC", "TFR", "TBL", "TBC", "TBR"
};

#define NUM_SPEAKERS (sizeof (speakers) / sizeof (speakers [0]))

// not threadsafe!
// don't use more than one instance in a single printf()!
static char *string_channel (int channel, int channel_mask, int center_width)
{
    static char channel_string [20], center_string [20];
    int chan_in_mask = 0, speaker_index;

    for (speaker_index = 0; speaker_index < NUM_SPEAKERS; ++speaker_index)
        if (channel_mask & (1 << speaker_index)) {
            if (chan_in_mask == channel) {
                sprintf (channel_string, "%d-%s", channel + 1, speakers [speaker_index]);
                break;
            }

            chan_in_mask++;
        }

    if (!strlen (channel_string))
        sprintf (channel_string, "%d", channel + 1);

    if (center_width && strlen (channel_string) < center_width) {
        int pre_spaces = (center_width - strlen (channel_string)) / 2;
        int post_spaces = center_width - pre_spaces - strlen (channel_string);

        center_string [0] = 0;
        while (pre_spaces--) strcat (center_string, " ");
        strcat (center_string, channel_string);
        while (post_spaces--) strcat (center_string, " ");
        return center_string;
    }
    else
        return channel_string;
}

// not threadsafe!
static char *string_time (double seconds)
{
    static char time_string [32];
    int minutes = (int) floor (seconds / 60.0);
    int hours = (int) floor (seconds / 3600.0);

    seconds -= minutes * 60.0;
    minutes -= hours * 60;

    sprintf (time_string, "%d:%02d:%05.2f", hours, minutes, seconds);
    return time_string;
}

static void float_integer_data (int32_t *src, float *dst, int num_samples, int bps)
{
    static float scalars [] = { 0.0, 1.0 / 128.0, 1.0 / 32768.0, 1.0 / 8388608.0, 1.0 / 2147483648.0 };
    float scalar = scalars [bps];

    for (int i = 0; i < num_samples; ++i)
        dst [i] = src [i] * scalar;
}

static void analyze_float_data (float *src, int num_samples, int num_channels, int sample_rate, ChannelData *chan_data)
{
    double attack = 1.0 / sample_rate * 43.0, decay = 1.0 - attack;

    for (int c = 0; c < num_channels; ++c) {
        double rms_sum = 0.0, value_sum = 0.0, abs_value_sum = 0.0;
        ChannelData *data = chan_data + c;
        int scount = num_samples;
        float *faudio = src + c;

        if (!data->num_samples) {
            data->min_value = *faudio;
            data->max_value = *faudio;
        }

        while (scount--) {
            int magnitude_percent = (int) floor (fabs (*faudio) * 100.0);

            value_sum += *faudio;
            rms_sum += *faudio * *faudio;
            abs_value_sum += fabs (*faudio);
            if (*faudio < data->min_value) data->min_value = *faudio;
            if (*faudio > data->max_value) data->max_value = *faudio;
            data->rms_level = data->rms_level * decay + *faudio * *faudio * attack;
            if (data->rms_level > data->peak_rms_level) data->peak_rms_level = data->rms_level;
            data->magnitude_histogram [magnitude_percent < 100 ? magnitude_percent : 99]++;
#ifdef THRESHOLD
            if (fabs (*faudio) < THRESHOLD) data->threshold++;
#endif
            faudio += num_channels;
        }

        data->abs_value_sum += abs_value_sum;
        data->num_samples += num_samples;
        data->value_sum += value_sum;
        data->rms_sum += rms_sum;
    }
}

static double population_to_magnitude (double population, ChannelData *chan_data)
{
    double population_target = chan_data->num_samples * population;
    int64_t magnitude_histogram [100];

    memcpy (magnitude_histogram, chan_data->magnitude_histogram, sizeof (magnitude_histogram));

    for (int i = 1; i < 100; ++i)
        magnitude_histogram [i] += magnitude_histogram [i - 1];

    if (magnitude_histogram [99] != chan_data->num_samples) {
        fprintf (stderr, "histogram is corrupt!\n");
        exit (1);
    }

    for (int i = 0; i < 100; ++i)
        if (magnitude_histogram [i] == population_target) {
            // fprintf (stderr, "%.6f: exact match at i = %d\n", population_target, i);
            return i / 100.0;
        }
        else if (magnitude_histogram [i] > population_target) {
            if (i) {
                // fprintf (stderr, "%.6f: partial match at i = %d, hist [%d] = %ld, hist [%d] = %ld\n",
                //    population_target, i, i, magnitude_histogram [i], i - 1, magnitude_histogram [i - 1]);
                double delta = magnitude_histogram [i] - magnitude_histogram [i - 1];
                double point = population_target - magnitude_histogram [i - 1];
                return (i + point / delta) / 100.0;
            }
            else {
                // fprintf (stderr, "%.6f: partial match at i = 0, hist [0] = %ld\n", population_target, magnitude_histogram [0]);
                return population_target / magnitude_histogram [0] / 100.0;
            }
        }

    fprintf (stderr, "fall through population check!\n");
    return 0.0;
}
