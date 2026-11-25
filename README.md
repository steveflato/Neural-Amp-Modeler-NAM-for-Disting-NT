# Neural Amp Modeler (NAM) for Disting NT

A disting NT plug-in that runs **Neural Amp Modeler A2-Lite** captures, the
kind you download from [TONE3000](https://www.tone3000.com). Put the `.nam`
files straight on the SD card. You don't need to convert them first.

Inference uses TONE3000's dependency-free Cortex-M7 A2-Lite engine
([tone-3000/embedded](https://github.com/tone-3000/embedded) v2.0.0, MIT).
On a 480 MHz Cortex-M7, TONE3000 measured it at about 4,060 cycles/sample,
or 41% of one core at 48 kHz.

## Install

1. Copy `plugins/nam_nt.o` to `/programs/plug-ins/` on the SD card.
2. Make a folder `/nam` at the root of the card and put your captures in it:
   `.nam` files from TONE3000, or `.namb` files (the compact binary format).
3. Add the algorithm **NAM** (category: Effect) and choose the **Channels**
   specification: **1** = mono, **2** = stereo.

The folder is scanned when the card mounts. To rescan after adding files,
re-insert the card (or reload the preset).

## Which captures work

| Capture | Works? |
|---|---|
| **NAM A2** (every TONE3000 capture since June 2026) | ✅ The plug-in loads the **A2-Lite** slim point that every A2 file contains |
| A2-Lite `.namb` | ✅ |
| A2-Full `.namb` | ❌ "A2-Full: too big". A2-Full costs ~19k cycles/sample, which is ~190% of a 480 MHz M7 |
| A1 (Standard / Lite / Feather / Nano), LSTM | ❌ "No A2-Lite inside" |

The `.nam` is parsed on the module by a streaming JSON reader. It reads the file
in small chunks, keeps the 1,871 A2-Lite weights and the `loudness` /
`input_level_dbu` metadata, and drops everything else, so even a 650 KB `.nam`
loads in a single pass with no large buffer.

## Parameters

**Amp page**
- **Model**: the capture, by file name (sorted). Confirm to load. Output fades
  out, the new capture loads and prewarms, and output fades back in.
- **Input gain**: −40 to +40 dB. "Drive" into the capture.
- **Output gain**: −40 to +24 dB.
- **Mix**: dry/wet. The dry path is latency-matched at 96 kHz.

**Levels page**
- **Calibrate in** (default On): if the capture has `input_level_dbu`, the
  input is scaled so a eurorack signal hits the model as hard as the real
  gear was hit. The reference is 5 V peak = +13.2 dBu = 0 dBFS into the model.
  Off means 5 V peak = 0 dBFS.
- **Normalize out** (default On): uses the capture's `loudness` to bring it to
  −18 dB, clamped to ±12 dB, so switching captures doesn't jump in level.
  This is the same gain staging as the NAM plug-in and the TONE3000 pedal.
- **Gate**: a noise gate on the model input. Fully down is off.

**Routing page**
- **Input / Output / Output mode**: the usual. With the stereo specification
  these become **Input L / Input R / Output L / Output R**, each output with
  its own mode.
- **Stereo** (stereo specification): one copy of the capture runs in both
  specifications, so stereo costs the same CPU as mono.
  - **Stereo in:** (L + R) / 2 goes through the capture and the wet signal
    goes to both outputs, while the dry signal stays stereo. It works like
    a mono amp in a stereo rig.
  - **Input R = None:** mono in to stereo out.
  - **Output R = None:** stereo in to mono out on L.
  - There is no dual-engine "true stereo" mode. Two copies of the capture
    would need about 111% of the NT's CPU, since one copy measures about 56%.
- **Drive CV**: an optional CV input that adds input gain at +6 dB per volt.
  It's read once per block.

The display also shows a profiler line: the engine's **cycles per sample** and
the NT's **block size**. Please include those numbers in any performance report.

The display shows the capture, its loudness and calibration level, and
input/output meters. Presets store the model **by file name**, so adding or
renaming other files doesn't change which capture a preset loads.

## Requirements

The plug-in declares API v14 and uses the filesystem API that was added to
the plug-in API in October 2026 (after v14). Use the latest disting NT firmware.

## Sample rate

A2 captures are trained at 48 kHz.
- **48 kHz**: native.
- **96 kHz**: the plug-in runs the model at 48 kHz through a 63-tap halfband
  2× resampler (passband flat to 20 kHz, −75 dB stopband). This adds 61
  samples (0.64 ms) of latency, and the dry path is delayed to match.
- **Any other rate**: the plug-in runs anyway, but the tone will be off, and the
  screen shows "SR not 48k!".

## Memory

| Region | Size | Holds |
|---|---|---|
| DTC | 10.6 KB | engine instance + capture weights (the hot data) |
| SRAM | 101 KB | the model's ring buffers |
| DRAM | 47 KB | file list, parser, `.namb` buffer |

The mono and stereo specifications use the same memory.

Plug-in code: about 19 KB.

## Build

```sh
git clone https://github.com/expertsleepersltd/distingNT_API   # needs file.h (filesystem API)
make NT_API_PATH=path/to/distingNT_API      # -> plugins/nam_nt.o
make test   NT_API_PATH=...                 # host build of the test harness
```

You need `arm-none-eabi-gcc`. The engine is built with `-O2 -ffast-math` (TONE3000's
recommended flags), and the rest with `-Os`.

## Tests (`test/`)

`host_test` compiles the real plug-in source against a mocked NT API and a
local folder standing in for the SD card. `test/run_tests.sh` runs all of it:

- **Parser**: the A2 `.nam` from NeuralAmpModelerCore, fed in 1-byte, whole-file
  and random chunks. The weights are **bit-exact** against Python's JSON reader.
- **Accuracy**: the plug-in end-to-end (bus in → gain → engine → bus out)
  versus NeuralAmpModelerCore's own output: **ESR −135 dB**, max error 2e-7,
  at 24- and 128-frame blocks.
- **Stereo**: summed stereo in, and mono in to stereo out, both match the golden
  output and a separate mono run, at 48 and 96 kHz.
- **Behaviour**: preset recall by name, switching while running, bad files
  muting and reporting, recovery, mix, and gain staging maths, all at 48 and 96 kHz.

Hardware: on a disting NT at 48 kHz, NAM uses about **55.6% CPU**, in mono or stereo.

## Licences

Plug-in code: MIT. Engine (`engine/`): MIT, © 2026 TONE3000. See
`engine/LICENSE-tone3000` and `engine/NOTICE-tone3000`. The example capture
in `sd-card/nam/` is NeuralAmpModelerCore's A2 test model (MIT, © Steven Atkinson).
