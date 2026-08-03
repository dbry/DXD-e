## DXD-e

Introducing the DXD-e Audio Data Format

Copyright (c) 2026 David Bryant.

All Rights Reserved.

Distributed under the [BSD Software License](https://github.com/dbry/DXD-e/blob/master/COPYING).

## Introduction

This is a library that implements a proposed digital audio intermediary format, called DXD-e, that
allows audio player and workstation applications to handle DSD (Direct Stream Digital) audio with
only minor modifications to their input and output plugins. This is accomplished by embedding the
original 1-bit DSD audio stream, along with a digital pilot signal, into a conventional decimated
24-bit PCM version of the DSD stream (which is known otherwise as DXD). This embedding consumes the
9 LSBs of each 24-bit sample (8 for the DSD and 1 for the pilot) while noise-shaping filters are
employed to completely bury the resulting noise underneath the existing high-frequency DSD
quantization noise always found in a DXD stream. In short, the DXD stream is made to hold a lossless
copy of the original DSD stream without any degradation.

## Details and Applications

An existing audio pipeline passes this high sample rate PCM stream (nominally 352.8 kHz for DSD64)
without even being aware of the DSD payload. Typical audio transformations will almost certainly
corrupt the integrity of the DSD, but this will also kill the digital pilot signal which serves
the sole purpose of indicating whether the DSD is present and valid.

On the output side, a filter constantly monitors the PCM stream for the presence of the digital pilot
signal. If it is present then the embedded DSD audio stream is known to be correct and is extracted
and passed on verbatim. If the pilot is _not_ present then a DSD modulator is employed to generate a
_new_ DSD stream from the [presumably] modified PCM. At the transitions between areas of valid and
invalid DSD, the time-aligned embedded and generated DSD streams are seamlessly stitched together at
some optimum point (which is easier said than done and certainly the trickiest part of the project).

So what could this be used for? For audio playback applications, being able to utilize fades and
cross-fades while still [most of the time] sending raw _lossless_ bitstream DSD audio to a DSD-capable
DAC, either directly or via DoP, becomes possible.

For digital audio workstation applications, it becomes possible to perform simple edits like fades and
trims and joins while avoiding glitches and preserving the exact bitstream over most of the track.
In both of these use cases, of course, using effects that modify every sample (e.g., gain or EQ) are
going to result in a re-modulated DSD stream, although even this can be achieved with high quality
and is no worse than existing solutions.

To accommodate differing applications, the DSD modulator has five quality levels that span the range
from 2nd-order to 4th-order noise-shaping filters. The lower order filters will operate in real time
on modern PC hardware and so can be used in audio player applications. The higher order filters do
not operate anywhere near real time but offer the highest possible quality for audio workstation
conversion output, which would presumably only be done at the completion of a project.

## Building

This package consists of the C library modules that implement all of the conversions. There is also
a command-line utility called DXD-E that demonstrates the process by converting from DSD to DXD-e and
back, and so can already be used with any DAW to losslessly edit DSD files. Because the WavPack audio
format is unique in supporting both PCM and DSD, the application uses that format exclusively. However,
it’s easy enough to convert from DSF or DFF or WAV files using the WavPack command-line programs. Note
that there are also several other command-line programs in the repo that simply act as filters to do
various conversions, but they are mostly for test purposes and would not interest most users.

A very simple Makefile is provided for building all the command-line programs on Linux, and specifically
the **DSD-E** utility. Note that **libwavpack-dev** is a dependency, and a recent version is recommended
(but not required) because multi-threading is heavily utilized. The Makefile is hardcoded for an
optimized build on Intel PCs using gcc, and has been tested on ARM, but can easily be edited for other
architectures. MinGW-w64 was used to build the Windows executable provided here, and it contains a
static copy of libwavpack.

## Instructions for **DXD-E**

Here is the "help" display for **DXD-E**:
```
 Usage:     DXD-E [-options] infile.wv [outfile.wv]

            * input WavPack file is analyzed and summary is displayed
            * if output file specified then conversion is performed:
              - DSD is converted to DXD-e (override with --no-embed)
              - DXD or DXD-e is converted back to DSD

 Options:  -1|2|3|4|5      = DSD encoding quality level, default = 3
                             1-3 are real-time capable but lower quality
                             4-5 are higher quality but can be very slow
           -h or --help    = display this usage guide
           --keep-gain     = do not reduce gain to recommended levels when
                             generating new DSD audio (applies only to
                             conversions without DSD extraction)
           --no-embed      = do not embed DSD or pilot in DXD file and
                             never extract DSD even if pilot detected
           --no-pilot      = do not add pilot signal to DXD-e file and
                             always extract DSD even if no pilot detected
           --no-random     = do not force the pilot signal to start at
                             a random point in its cycle (testing only)
           -q or --quiet   = skip progress display and detailed file info
           -y              = overwrite outfile if it exists
```

## Technical Notes

- The DSD decimator and modulator and the utilities to embed the DSD signal and pilot signal into the DXD stream
and the code to process the DXD-e stream and switch between and splice the extracted streams with the generated
streams are all provided here as examples. Better implementations of all these are probably possible and might
be better suited for specific applications. The only aspect that **is** required to be compatible with the
official DXD-e format is the location of the DSD data (the lower 8 bits of each DXD sample) and the digital
pilot signal (which is a parity bit just above the DSD data (i.e., b.8) that follows a unique 64-bit pattern
using feedback through a shift-register). Also, it is important that the embedded DSD is aligned such that
the DXD PCM sample corresponds temporally to the middle of the DSD "byte" that it contains (i.e., halfway
between b.3 and b.4).

- The digital pilot signal is 64-bits long, so would essentially never be triggered inadvertently. The feedback
shift-register provides a repeat period of about 2^37 samples, which is over 13 hours of DSD64, and every channel
of DSD to DXD-e conversions are always started at random posiitons in that space, so it is essentially impossible
for a collision to occur when editing.

- The pilot signal is based on the parity of each full 24-bit sample. Virtually all audio transformations would
corrupt that property randomly from sample to sample and destroy the pilot. However the _complement_ operation does
_not_ affect parity, so a long stretch of complemented samples would appear as unmodified DSD. However,
interestingly, that complement operation would also toggle all the DSD bits, which would have the same negating
affect on the audio signal, so it's not an error condition at all! Conversely, actual _negate_ operations do not
preserve parity and so present no issue.

- DSD audio is very sensitive to high sample magnitudes, and the SACD disc spec recommends the level be generally
limited to -6.0 dBFS and forbids anything over -2.9 dBFS. That said, some commercially available DSD audio tracks
_do_ exceed this level, and audio pipelines have no such hard limits. Because of these concerns, the DSD modulator
provided here has been designed to handle arbitrary input levels, up to and including clipping, and this is
accomplished using several methods. As the sample level increases above -6.0 dBFS, the modulator increases the ply
search depth to maintain stability. Above -2.9 dBFS to full-scale a soft clipper is slowly introduced that limits
the maximum encoded sample value to -1.3 dB. Also, in addition to the soft clipping, the noise-shaping filters
progressively relax to avoid instability. The net effect is that the modulator can be provided arbitrary input
and still generate audibly acceptable results while providing optimum results below the SACD mandated limit.

