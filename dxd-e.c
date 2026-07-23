#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <wavpack/wavpack.h>

#include "modulator.h"
#include "dsd-utils.h"

// #define THRESHOLD 0.501171

typedef struct {
    float min_value, max_value;
    double value_sum, abs_value_sum, rms_sum, rms_level, peak_rms_level;
    int64_t num_samples, magnitude_histogram [100], threshold;
    int valid_dsd_sectors, no_dsd_sectors;
} ChannelData;

typedef struct {
    char format [32];
    ChannelData *chan_data;
    int64_t total_samples;
    int bytes_per_sample, bits_per_sample;
    int num_channels, sample_rate, qmode, mode, dsd, dxd, errors;
    uint32_t channel_mask;
} WavpackFileInfo;

static WavpackFileInfo *analyze_file (FILE *output, char *filename, char *error);
static void display_file_info (FILE *output, WavpackFileInfo *file_info);
static int convert_dsd_to_dxd (char *infilename, char *outfilename, char *error);
static int convert_dxd_to_dsd (char *infilename, char *outfilename, char *error);

static int embed_dsd = 1, embed_pilot = 1;

int main (int argc, char **argv)
{
    char *infilename = NULL, *outfilename = NULL, error [80];
    int overwrite = 0, res = 0;
    WavpackFileInfo *info;

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

            if (!strcmp (long_option, "no-embed"))                          // --no-embed
                embed_dsd = 0;
            else if (!strcmp (long_option, "no-pilot"))                     // --no-pilot
                embed_pilot = 0;
            else {
                fprintf (stderr, "unknown option: %s !\n", long_option);
                return 1;
            }
        }
#if defined (_WIN32)
        else if ((**argv == '-' || **argv == '/') && (*argv)[1])
#else
        else if ((**argv == '-') && (*argv)[1])
#endif
            while (*++*argv)
                switch (**argv) {

                    case 'Y': case 'y':
                        overwrite = 1;
                        break;

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

    if (outfilename && !overwrite) {
        FILE *test_read_open = fopen (outfilename, "rb");
        if (test_read_open) {
            fclose (test_read_open);
            fprintf (stderr, "output file %s exists, use -y to overwrite!\n", outfilename);
            return 1;
        }
    }

    fprintf (stderr, "analyzing file %s...\n", infilename);
    info = analyze_file (stdout, infilename, error);

    if (!info) {
        fprintf (stderr, "error in file %s: %s\n", infilename, error);
        return 1;
    }

    display_file_info (stdout, info);

    if (outfilename) {
        if (info->dsd) {
            if (embed_dsd)
                fprintf (stderr, "converting %s file %s to DXD%d-e file %s (%s)...\n",
                    info->format, infilename, info->sample_rate / 1000, outfilename,
                    embed_pilot ? "with embedded DSD and pilot" : "with embedded DSD only");
            else
                fprintf (stderr, "converting %s file %s to DXD%d file %s (no embedded DSD)...\n",
                    info->format, infilename, info->sample_rate / 1000, outfilename);

            res = convert_dsd_to_dxd (infilename, outfilename, error);

            if (res)
                fprintf (stderr, "error: can't convert file %s, %s!\n", infilename, error);
        }
        else if (info->dxd) {
            fprintf (stderr, "converting %s file %s to DSD file %s...\n",
                info->format, infilename, outfilename);

            res = convert_dxd_to_dsd (infilename, outfilename, error);

            if (res)
                fprintf (stderr, "error: can't convert file %s, %s!\n", infilename, error);
        }
        else {
            fprintf (stderr, "error: can't convert file %s, not DSD or DXD!\n", infilename);
            res = 1;
        }
    }

    free (info->chan_data);
    free (info);
    return res;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define BUFFER_SAMPLES  1048576

static void analyze_float_data (float *src, int num_samples, int num_channels, int sample_rate, ChannelData *chan_data);
static void float_integer_data (int32_t *src, float *dst, int num_samples, int bps);
static double population_to_magnitude (double population, ChannelData *chan_data);
static char *string_channel (int channel, int channel_mask, int center_width);
static char *string_time (double seconds);

static WavpackFileInfo *analyze_file (FILE *output, char *filename, char *error)
{
    int flags = OPEN_NORMALIZE | OPEN_DSD_NATIVE | (4 << OPEN_THREADS_SHFT);
    WavpackContext *cxt = WavpackOpenFileInput (filename, error, flags, 0);
    WavpackFileInfo *file_info;

    if (!cxt)
        return NULL;

    file_info = calloc (1, sizeof (WavpackFileInfo));
    file_info->total_samples = WavpackGetNumSamples64 (cxt);
    file_info->bytes_per_sample = WavpackGetBytesPerSample (cxt);
    file_info->bits_per_sample = WavpackGetBitsPerSample (cxt);
    file_info->channel_mask = WavpackGetChannelMask (cxt);
    file_info->num_channels = WavpackGetNumChannels (cxt);
    file_info->sample_rate = WavpackGetSampleRate (cxt);
    file_info->qmode = WavpackGetQualifyMode (cxt);
    file_info->mode = WavpackGetMode (cxt);

    if (file_info->qmode & QMODE_DSD_AUDIO) {
        if (file_info->sample_rate % 44100 == 0) {
            file_info->dsd = file_info->sample_rate * 8 / 44100;
            sprintf (file_info->format, "DSD%d", file_info->dsd);
        }
        else if (file_info->sample_rate % 48000 == 0) {
            file_info->dsd = file_info->sample_rate * 8 / 48000;
            sprintf (file_info->format, "DSD%d (%f MHz)", file_info->dsd, file_info->sample_rate / 125000.0);
        }
        else {
            file_info->dsd = -1;
            sprintf (file_info->format, "DSD @ %f MHz", file_info->sample_rate / 125000.0);
        }
    }
    else if (file_info->bits_per_sample >= 24 && (file_info->sample_rate % 352800 == 0 || file_info->sample_rate % 384000 == 0)) {
        file_info->dxd = file_info->sample_rate / 1000;
        sprintf (file_info->format, "DXD%d", file_info->dxd);
    }
    else
        sprintf (file_info->format, "PCM");

    int sector_samples = file_info->sample_rate / 75;
    int buffer_samples = BUFFER_SAMPLES / sector_samples * sector_samples;

    ChannelData *chan_data = file_info->chan_data = calloc (file_info->num_channels, sizeof (ChannelData));
    int32_t *source_buffer = malloc (buffer_samples * sizeof (int32_t) * file_info->num_channels);
    float *float_buffer = malloc (buffer_samples * sizeof (float) * file_info->num_channels);
    int64_t samples_remaining = file_info->total_samples, samples_processed = 0;
    unsigned char *dsd_samples = NULL;
    DecimateDSD *decimator = NULL;
    PilotDetect *detector = NULL;

    if (file_info->qmode & QMODE_DSD_AUDIO) {
        dsd_samples = malloc (buffer_samples * sizeof (char) * file_info->num_channels);
        decimator = decimateDSDinit (file_info->num_channels, 0);
    }

    if (file_info->dxd)
        detector = PilotDetectInit (file_info->num_channels);

    while (1) {
        int samples_to_read = buffer_samples, samples_read, samples_ready;

        if (samples_to_read > samples_remaining)
            samples_to_read = samples_remaining;

        samples_read = WavpackUnpackSamples (cxt, source_buffer, samples_to_read);

        if (samples_read != samples_to_read) {
            strcpy (error, "file exhausted prematurely");
            WavpackCloseFile (cxt);
            free (file_info->chan_data);
            free (file_info);
            return NULL;
        }

        samples_remaining -= samples_read;
        samples_ready = samples_read;

        if (file_info->qmode & QMODE_DSD_AUDIO) {
            int dsd_samples_ready = samples_ready * file_info->num_channels;

            for (int i = 0; i < dsd_samples_ready; ++i)
                dsd_samples [i] = source_buffer [i] & 0xff;

            samples_ready = decimateDSDrun (decimator, dsd_samples, samples_ready ? samples_ready : -1, source_buffer);

            if (!samples_ready)
                break;

            float_integer_data (source_buffer, float_buffer, samples_ready * file_info->num_channels, 3);
        }
        else {
            if (!samples_ready)
                break;

            if (file_info->mode & MODE_FLOAT) {
                memcpy (float_buffer, source_buffer, sizeof (float) * samples_ready * file_info->num_channels);

                if (file_info->dxd)
                    for (int i = 0; i < samples_ready * file_info->num_channels; ++i)
                        source_buffer [i] = float_buffer [i] * 8388608.0;
            }
            else
                float_integer_data (source_buffer, float_buffer, samples_ready * file_info->num_channels, file_info->bytes_per_sample);
        }

        samples_processed += samples_ready;

        analyze_float_data (float_buffer, samples_ready, file_info->num_channels, file_info->sample_rate, chan_data);

        if (file_info->dxd)
            for (int chan = 0; chan < file_info->num_channels; ++chan) {
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

                    buffer_index += samples * file_info->num_channels;
                    samples_to_scan -= samples;
                }
            }
    }

    file_info->errors = WavpackGetNumErrors (cxt);

    WavpackCloseFile (cxt);
    free (source_buffer);
    free (float_buffer);

    if (file_info->qmode & QMODE_DSD_AUDIO) {
        decimateDSDdestroy (decimator);
        free (dsd_samples);
    }

    if (file_info->dxd)
        PilotDetectDestroy (detector);

    return file_info;
}

static void display_file_info (FILE *output, WavpackFileInfo *file_info)
{
    ChannelData *chan_data = file_info->chan_data;

    fprintf (output, "\n");
    fprintf (output, "%16s:%12s\n", "format", file_info->format);

    if (file_info->mode & MODE_FLOAT)
        fprintf (output, "%16s:%12s\n", "bit depth", "32-bit FLT");
    else if (file_info->dsd)
        fprintf (output, "%16s:%12s\n", "bit depth", "1-bit DSD");
    else
        fprintf (output, "%16s:%8d-bit\n", "bit depth", file_info->bits_per_sample);

    fprintf (output, "%16s:%12d\n", "sample rate", file_info->dsd ? file_info->sample_rate * 8 : file_info->sample_rate);
    fprintf (output, "%16s:%12s\n", "duration", string_time ((double) file_info->total_samples / file_info->sample_rate));

    if (file_info->errors)
        fprintf (output, "%16s:%12d\n", "errors", file_info->errors);

    fprintf (output, "\n");

    if (file_info->num_channels > 0) {
        fprintf (output, "%16s:  ", "channels");
        for (int chan = 0; chan < file_info->num_channels; ++chan)
            fputs (string_channel (chan, file_info->channel_mask, 12), output);
        fprintf (output, "\n");

        for (int i = 0; i < 17 + file_info->num_channels * 12; ++i)
            fputc ('-', output);

        fprintf (output, "\n");
    }

    if (file_info->dxd) {
        fprintf (output, "%16s:", "DSD detected");
        for (int chan = 0; chan < file_info->num_channels; ++chan)
            if (!chan_data [chan].valid_dsd_sectors)
                fprintf (output, "%12s", "** no **");
            else if (!chan_data [chan].no_dsd_sectors)
                fprintf (output, "%12s", "** yes **");
            else
                fprintf (output, "%11.2f%%",
                    chan_data [chan].valid_dsd_sectors * 100.0 / (chan_data [chan].no_dsd_sectors + chan_data [chan].valid_dsd_sectors));
        fprintf (output, "\n");

        // fprintf (output, "%16s:", "invalid sectors");
        // for (int chan = 0; chan < file_info->num_channels; ++chan)
        //     fprintf (output, "%12d", chan_data [chan].no_dsd_sectors);
        // fprintf (output, "\n");

        // fprintf (output, "%16s:", "valid sectors");
        // for (int chan = 0; chan < file_info->num_channels; ++chan)
        //     fprintf (output, "%12d", chan_data [chan].valid_dsd_sectors);
        // fprintf (output, "\n");
    }

    fprintf (output, "%16s:", "min value");
    for (int chan = 0; chan < file_info->num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].min_value);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "max value");
    for (int chan = 0; chan < file_info->num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].max_value);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "ave value");
    for (int chan = 0; chan < file_info->num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].value_sum / chan_data [chan].num_samples);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "ave magnitude");
    for (int chan = 0; chan < file_info->num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].abs_value_sum / chan_data [chan].num_samples);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "99.9% magnitude");
    for (int chan = 0; chan < file_info->num_channels; ++chan) {
        double magnitude = population_to_magnitude (0.999, chan_data + chan);
        if (magnitude > chan_data [chan].max_value) magnitude = chan_data [chan].max_value;
        else if (magnitude < chan_data [chan].min_value) magnitude = chan_data [chan].min_value;
        fprintf (output, "%12f", magnitude);
    }
    fprintf (output, "\n");

#ifdef THRESHOLD
    fprintf (output, "%16s:", "threshold");
    for (int chan = 0; chan < file_info->num_channels; ++chan)
        fprintf (output, "%12f", chan_data [chan].threshold * 100.0 / chan_data [chan].num_samples);
    fprintf (output, "   (%.6f)", THRESHOLD);
    fprintf (output, "\n");
#endif

    fprintf (output, "%16s:", "rms level");
    for (int chan = 0; chan < file_info->num_channels; ++chan)
        fprintf (output, "%9.2f dB", log10 (chan_data [chan].rms_sum / chan_data [chan].num_samples / 0.5) * 10);
    fprintf (output, "\n");

    fprintf (output, "%16s:", "peak level");
    for (int chan = 0; chan < file_info->num_channels; ++chan)
        fprintf (output, "%9.2f dB", log10 (chan_data [chan].peak_rms_level / 0.5) * 10);
    fprintf (output, "\n\n");
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
// don't use more than one instance in a single printf()!
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// This structure and function are used to write completed WavPack blocks in
// a device independent way.

typedef struct {
    uint32_t bytes_written, first_block_size;
    FILE *file;
    int error;
} write_id;

static int DoWriteFile (FILE *hFile, void *lpBuffer, uint32_t nNumberOfBytesToWrite, uint32_t *lpNumberOfBytesWritten)
{
    uint32_t bcount;

    *lpNumberOfBytesWritten = 0;

    while (nNumberOfBytesToWrite) {
        bcount = (uint32_t) fwrite ((unsigned char *) lpBuffer + *lpNumberOfBytesWritten, 1, nNumberOfBytesToWrite, hFile);

        if (bcount) {
            *lpNumberOfBytesWritten += bcount;
            nNumberOfBytesToWrite -= bcount;
        }
        else
            break;
    }

    return !ferror (hFile);
}

static int write_block (void *id, void *data, int32_t length)
{
    write_id *wid = (write_id *) id;
    uint32_t bcount;

    if (wid->error)
        return 0;

    if (wid && wid->file && data && length) {
        if (!DoWriteFile (wid->file, data, length, &bcount) || bcount != length) {
            fclose (wid->file);
            wid->file = NULL;
            wid->error = 1;
            return 0;
        }
        else {
            wid->bytes_written += length;

            if (!wid->first_block_size)
                wid->first_block_size = bcount;
        }
    }

    return 1;
}

static int convert_dsd_to_dxd (char *infilename, char *outfilename, char *error)
{
    int flags = OPEN_DSD_NATIVE | (4 << OPEN_THREADS_SHFT);
    WavpackContext *incxt = WavpackOpenFileInput (infilename, error, flags, 0), *outcxt;
    WavpackFileInfo *file_info;
    WavpackConfig config = {};
    write_id wv_file = {};

    if (!incxt)
        return 1;

    file_info = calloc (1, sizeof (WavpackFileInfo));
    file_info->total_samples = WavpackGetNumSamples64 (incxt);
    file_info->bytes_per_sample = WavpackGetBytesPerSample (incxt);
    file_info->bits_per_sample = WavpackGetBitsPerSample (incxt);
    file_info->channel_mask = WavpackGetChannelMask (incxt);
    file_info->num_channels = WavpackGetNumChannels (incxt);
    file_info->sample_rate = WavpackGetSampleRate (incxt);
    file_info->qmode = WavpackGetQualifyMode (incxt);
    file_info->mode = WavpackGetMode (incxt);

    if (!(file_info->qmode & QMODE_DSD_AUDIO)) {
        strcpy (error, "file not DSD audio mode");
        WavpackCloseFile (incxt);
        free (file_info);
        return 1;
    }

    outcxt = WavpackOpenFileOutput (write_block, &wv_file, NULL);
    wv_file.file = fopen (outfilename, "w+b");

    if (wv_file.file == NULL) {
        strcpy (error, "can't create output file");
        WavpackCloseFile (outcxt);
        WavpackCloseFile (incxt);
        free (file_info);
        return 1;
    }

    config.bits_per_sample = 24;
    config.bytes_per_sample = 3;
    config.sample_rate = file_info->sample_rate;
    config.num_channels = file_info->num_channels;
    config.channel_mask = file_info->channel_mask;
    config.worker_threads = 4;

    if (!WavpackSetConfiguration64 (outcxt, &config, file_info->total_samples, NULL)) {
        strcpy (error, WavpackGetErrorMessage (outcxt));
        fclose (wv_file.file);
        WavpackCloseFile (outcxt);
        WavpackCloseFile (incxt);
        free (file_info);
        return 1;
    }

    WavpackPackInit (outcxt);

    int sector_samples = file_info->sample_rate / 75;
    int buffer_samples = BUFFER_SAMPLES / sector_samples * sector_samples;
    int nchans = file_info->num_channels;

    int32_t *source_buffer = malloc (buffer_samples * sizeof (int32_t) * nchans);
    int64_t samples_remaining = file_info->total_samples, samples_processed = 0;
    unsigned char *dsd_buffer = malloc (buffer_samples * sizeof (char) * nchans);
    DecimateDSD *decimator = decimateDSDinit (nchans, 0);
    EmbedContext *dsd_embedder = NULL;
    int dsd_samples = 0;

    if (embed_dsd)
        dsd_embedder = dsd_embed_init (nchans, embed_pilot ? EMBED_PILOT_SIGNAL : 0);

    while (1) {
        int samples_to_read = buffer_samples - dsd_samples, samples_read, samples_decimated;

        if (samples_to_read > samples_remaining)
            samples_to_read = samples_remaining;

        samples_read = WavpackUnpackSamples (incxt, source_buffer, samples_to_read);

        if (samples_read != samples_to_read) {
            strcpy (error, "file exhausted prematurely");
            WavpackCloseFile (incxt);
            WavpackCloseFile (outcxt);
            fclose (wv_file.file);
            free (file_info->chan_data);
            free (file_info);
            return 1;
        }

        samples_remaining -= samples_read;

        for (int i = 0; i < samples_read * nchans; ++i)
            dsd_buffer [i + dsd_samples * nchans] = source_buffer [i] & 0xff;

        samples_decimated = decimateDSDrun (decimator, dsd_buffer + dsd_samples * nchans, samples_read ? samples_read : -1, source_buffer);
        dsd_samples += samples_read;

        if (!samples_decimated)
            break;

        if (dsd_embedder) {
            dsd_embed_run (dsd_embedder, source_buffer, dsd_buffer, samples_decimated);

            if (dsd_samples > samples_decimated) {
                memmove (dsd_buffer, dsd_buffer + samples_decimated * nchans, (dsd_samples - samples_decimated) * nchans);
                dsd_samples -= samples_decimated;
            }
            else
                dsd_samples = 0;
        }
        else
            dsd_samples = 0;

        if (!WavpackPackSamples (outcxt, source_buffer, samples_decimated)) {
            strcpy (error, WavpackGetErrorMessage (outcxt));
            WavpackCloseFile (incxt);
            WavpackCloseFile (outcxt);
            fclose (wv_file.file);
            free (file_info->chan_data);
            free (file_info);
            return 1;
        }

        samples_processed += samples_decimated;
    }

    if (!WavpackFlushSamples (outcxt)) {
        strcpy (error, WavpackGetErrorMessage (outcxt));
        WavpackCloseFile (incxt);
        WavpackCloseFile (outcxt);
        fclose (wv_file.file);
        free (file_info->chan_data);
        free (file_info);
        return 1;
    }

    if (WavpackGetSampleIndex64 (outcxt) != file_info->total_samples) {
        strcpy (error, "incorrect number of samples written");
        WavpackCloseFile (incxt);
        WavpackCloseFile (outcxt);
        fclose (wv_file.file);
        free (file_info->chan_data);
        free (file_info);
        return 1;
    }

    if (WavpackGetNumErrors (incxt))
        fprintf (stderr, "warning: %d errors detected!\n", WavpackGetNumErrors (incxt));

    WavpackCloseFile (incxt);
    WavpackCloseFile (outcxt);
    fclose (wv_file.file);

    if (dsd_embedder)
        dsd_embed_destroy (dsd_embedder);

    decimateDSDdestroy (decimator);
    free (file_info->chan_data);
    free (file_info);
    free (source_buffer);
    free (dsd_buffer);

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int convert_dxd_to_dsd (char *infilename, char *outfilename, char *error)
{
    return 0;
}
