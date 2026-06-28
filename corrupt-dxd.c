#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sys/random.h>

int main (int argc, char **argv)
{
    int nchans = 2, random = 0, short_runs = 0, long_runs = 0, always = 0;

    if (argc == 1) {
        fprintf (stderr, "Corrupt raw 24-bit DXD audio, destroying random runs of embedded DSD with no impact on PCM\n");
        fprintf (stderr, "Usage: corrupt-dxd <nchans> [R|r] [A|a] [S|s] [L|l] < 24bit-dxd.raw > 24bit-dxd.raw\n");
        fprintf (stderr, "       <nchans> = 1 to 16 (required)\n");
        fprintf (stderr, "       [R|r] randomize every time\n");
        fprintf (stderr, "       [A|a] corrupt always\n");
        fprintf (stderr, "       [S|s] short runs\n");
        fprintf (stderr, "       [L|l] long runs\n");
        return 0;
    }

    if (argc > 1) {
        nchans = atoi (argv [1]);

        if (nchans < 1 || nchans > 16) {
            fprintf (stderr, "must be 1 to 16 channels!\n");
            return 1;
        }
    }

    if (argc > 2)
        for (int argi = 2; argi < argc; ++argi) {
            if (strlen (argv [argi]) == 1 && (*argv [argi] == 'r' || *argv [argi] == 'R'))
                random = 1;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 's' || *argv [argi] == 'S'))
                short_runs = 1;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'l' || *argv [argi] == 'L'))
                long_runs = 1;
            else if (strlen (argv [argi]) == 1 && (*argv [argi] == 'a' || *argv [argi] == 'A'))
                always = 1;
            else {
                fprintf (stderr, "unknown argument: %s\n", argv [argi]);
                return 1;
            }
        }

    unsigned int seed = 0x31415926;
    int *sample_run_count = calloc (nchans, sizeof (int));
    int *corrupt_flag = calloc (nchans, sizeof (int));
    int done = 0;

    if (random && getrandom (&seed, sizeof (seed), 0) != sizeof (seed))
        fprintf (stderr, "corrupt-dxd: getrandom() not working!\n");

    while (!done) {
        for (int c = 0; c < nchans; ++c) {
            int32_t sample24 = getchar ();

            if (sample24 == EOF) {
                done = 1;
                break;
            }

            if (!sample_run_count [c]) {
                do {
                    if (short_runs)
                        sample_run_count [c] = seed >> 14;
                    else if (long_runs)
                        sample_run_count [c] = seed >> 10;
                    else
                        sample_run_count [c] = seed >> 12;

                    seed = ((seed << 4) - seed) ^ 1;
                    seed = ((seed << 4) - seed) ^ 1;
                    seed = ((seed << 4) - seed) ^ 1;
                }
                while (sample_run_count [c] < 64);

                corrupt_flag [c] = always | (seed >> 31);
                seed = ((seed << 4) - seed) ^ 1; seed = ((seed << 4) - seed) ^ 1; seed = ((seed << 4) - seed) ^ 1;
                // fprintf (stderr, "corrupt %d: %s for %d samples\n", c, corrupt_flag [c] ? "corrupt" : "straight", sample_run_count [c]);
            }

            sample24 <<= 8;
            sample24 += getchar () << 16;
            sample24 += getchar () << 24;
            sample24 >>= 8;

            if (corrupt_flag [c]) {
                if (seed >> 31) {
                    int parity = __builtin_parity (sample24);

                    if (sample24 >= 0)
                        while (__builtin_parity (--sample24) == parity);
                    else
                        while (__builtin_parity (++sample24) == parity);
                }

                seed = ((seed << 4) - seed) ^ 1;
                seed = ((seed << 4) - seed) ^ 1;
                seed = ((seed << 4) - seed) ^ 1;
            }

            putchar (sample24 & 0xff);
            putchar ((sample24 >> 8) & 0xff);
            putchar ((sample24 >> 16) & 0xff);
            sample_run_count [c]--;
        }
    }

    return 0;
}
