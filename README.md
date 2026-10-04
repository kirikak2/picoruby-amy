# picoruby-amy

The [AMY](https://github.com/shorepine/amy) synthesizer engine for PicoRuby,
playing through the board's I2S audio output and presented as a
[picoruby-midi](https://github.com/kirikak2/picoruby-midi) transport, so it is
driven like any other MIDI device.

AMY (MIT, Brian Whitman and Dan Ellis) covers DX7-style FM, Juno-6 style
analog, PCM, partials and a piano; it ships 128 DX7 and 128 Juno presets. This
gem starts with FM: `AMY::FM` wraps the DX7 presets with named parameters.

## Usage

```ruby
require 'midi'
require 'amy'

fm  = AMY::FM.new(channel: 0, voices: 6, patch: 128)   # DX7 presets: 128..255
dev = MIDI::Device.new(fm.transport)                   # a MIDI transport
dev.trigger(60, 100, duration: 300)

fm.algorithm = 5                 # 1..32
fm.feedback  = 0.4
fm.op(2).ratio  = 3.5            # DX7 operator numbering, 1..6
fm.op(2).level  = 0.8
fm.op(1).envelope(attack: 10, decay: 300, sustain: 0.5, release: 400)
fm.filter_freq = 2000            # turns a lowpass on
fm.reverb = 0.3
```

MIDI channel N (0-based, as in `MIDI::Device`) plays AMY synth N + 1, once
that synth exists.

### Two ways to control a parameter

* **CC mapping inside AMY** — `fm.map_cc(74, :filter_freq)`. AMY applies the
  CC itself, so any MIDI source (an on-screen knob sending CC, a keyboard)
  works and no Ruby runs per change. Mappable: `:filter_freq`, `:resonance`,
  `:feedback`, `:pan`, `:portamento`, `:volume`, `:reverb`, `:chorus`, and per
  operator `:level` / `:ratio` (with `op:`).
* **Direct set** — for what AMY cannot map to a CC (envelopes, algorithm,
  patch): `fm.set(:attack, 30, op: 2)`, `fm.op(2).attack = 30`,
  `fm.algorithm = 7`. Call these from a `UI.knob` block or a
  `MIDI::Input#on(:control_change)` handler. `AMY.scale(v, min, max, log:)`
  maps 0..127 to a range.

Envelopes are breakpoint lists that AMY takes whole, so each operator keeps
its ADSR in Ruby and resends it on every change.

`fm.patch = n` (and `fm.refresh`) reads the preset back from AMY, so
`fm.algorithm`, `fm.feedback` and `fm.op(n).level / ratio / attack / decay /
sustain / release` report the preset's values -- e.g. to move knobs there
with `UI.knob_set(i, AMY.unscale(fm.op(1).attack, 1, 2000, log: true),
notify: false)`. A DX7 envelope has more stages than an ADSR and is
summarised (attack = first stage, decay = the middle stages, sustain = the
level held, release = last stage); changing one stage then replaces the
operator's DX7 envelope with an ADSR built from those values.

### Anything else

```ruby
AMY.command(synth: 1, osc: 0, filter_type: 1, filter_freq: 1500)  # by name
AMY.wire("i1K130iv6Z")                                            # raw wire message
AMY.reset                                                         # power-on state
AMY.patch_name(130)                                               # => "BRASS 3"
```

See AMY's [API reference](https://github.com/shorepine/amy/blob/main/docs/api.md)
for the parameters.

## Building

The engine lives in `lib/amy` (git submodule, pinned). On ESP-IDF build its
sources as a component (AMY ships none; Midori's is `components/amy`) and
compile `ports/esp32/amy_port.c` with the board's configuration:

| Define | Meaning | Default |
|--------|---------|---------|
| `AMY_GEM_ENABLED` | 0 builds stubs | 0 |
| `AMY_GEM_I2S_BCLK` / `_WS` / `_DOUT` / `_MCLK` | I2S pins (-1 = unused) | -1 |
| `AMY_GEM_I2S_PORT` | I2S controller, -1 = any | -1 |
| `AMY_GEM_I2S_PHILIPS` | 1 standard I2S, 0 MSB / left-justified | 1 |
| `AMY_GEM_MCLK_MULTIPLE` | MCLK / fs; must match the codec | 256 |
| `AMY_GEM_DMA_BLOCKS` | I2S DMA depth in 256-frame blocks | 4 |
| `AMY_GEM_TASK_CORE` / `_PRIORITY` / `_STACK` | the audio task | 0 / 3 / 16 KB |
| `AMY_GEM_OVERLOAD_THRESHOLD` | render load at which AMY resets itself | 0.7 |

The audio task renders and writes every block itself (AMY's own render
tasks are not used), so its core and priority are the only scheduling knobs.
It must run before anything that can hog its core for longer than the DMA
ring (about 23 ms at the defaults).

A codec or amplifier that needs powering up is the application's job:

```c
#include "amy_gem.h"
static void power(bool on, void *arg) { /* codec over I2C, amp enable pin */ }
AMY_GEM_set_power_callback(power, NULL);   // before AMY starts
```

The callback runs on the audio task once the I2S clocks run.

`trigger` and the note scheduler reach AMY through picoruby-midi's transport
registry (`MIDI_transport_register`), which this gem joins when it starts.

## License

MIT. AMY itself is MIT (see `lib/amy/LICENSE`).
