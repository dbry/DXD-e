## DXD-e

Experiments with the DSD and DXD audio formats

Copyright (c) 2026 David Bryant.

All Rights Reserved.

Distributed under the [BSD Software License](https://github.com/dbry/DXD-e/blob/master/COPYING).

## What is this?

This is experimental code to investigate DSD and DXD audio. Specifically, the idea of embedding the
source DSD stream, with a digital pilot signal, inside the DXD PCM audio stream (hence DXD-e). This
would enable audio playback and DAW applications that currently only handle PCM audio to easily add
DSD capability by simply adding new input and output interfaces (assuming their audio pipelines can
losslessly pass 24-bit PCM).

This is a work in progress.
