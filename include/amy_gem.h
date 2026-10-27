/*
 * PicoRuby AMY - C API
 *
 * Runs the AMY synthesizer (https://github.com/shorepine/amy) on its own
 * audio task, writes the rendered blocks to an I2S output, and presents
 * the synth to picoruby-midi as a MIDI transport.
 *
 * The gem owns the engine, the audio task and the I2S channel. Everything
 * that depends on the board beyond the I2S pins -- powering a codec up over
 * I2C, switching an amplifier on -- stays in the application, which hands
 * it in with AMY_GEM_set_power_callback(). Pins and the rest of the
 * build-time knobs are listed in amy_gem_config.h.
 */

#ifndef AMY_GEM_H_
#define AMY_GEM_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Board hook: bring the codec / amplifier up (on = true) or down.
 * Called on the audio task, after the I2S channel is clocking and before
 * the first block is written. Keep it to register writes. */
typedef void (*amy_gem_power_cb_t)(bool on, void *arg);

/* Register the board hook. Call before AMY_GEM_start(). */
void AMY_GEM_set_power_callback(amy_gem_power_cb_t cb, void *arg);

/* Start the audio task and the engine. Blocks until AMY is running.
 * Idempotent: once running, further calls only return 0.
 * Returns 0 on success, -1 on failure (or when the gem was built without
 * AMY_GEM_ENABLED). */
int AMY_GEM_start(void);

/* Whether the engine is running. */
bool AMY_GEM_running(void);

/* Return AMY to its power-on state: every synth, patch, CC mapping,
 * channel binding and effect is cleared. The audio task and I2S keep
 * running. */
void AMY_GEM_reset(void);

/* Queue one AMY wire message (e.g. "i1K130iv6Z"). Safe from any task.
 * Returns 0, or -1 when the engine is not running. */
int AMY_GEM_send_message(const char *message);

/* Hand one USB-MIDI packet to AMY (the transport's send_packet). A channel
 * message goes to every synth bound to its channel (AMY_GEM_bind_synth),
 * a system message to AMY's own MIDI input. Returns 0, or -1 when not
 * running / not a message AMY takes. */
int AMY_GEM_send_packet(uint8_t cable, uint8_t cin,
                        uint8_t b1, uint8_t b2, uint8_t b3);

/* AMY synths a MIDI channel can play: 1 .. AMY_GEM_MAX_PARTS. AMY keeps
 * CC mappings for synths 1..16 only (AMY_NUM_MIDI_CHANNELS). */
#define AMY_GEM_MAX_PARTS 16

/* Make MIDI channel `channel` (0..15) play AMY synth `synth`, in place of
 * the channel it played before; channel -1 = no channel. Several synths may
 * share a channel (a layer): send_packet hands each of them the message.
 * A channel with no synth bound is silent. AMY_GEM_reset() unbinds all.
 * Returns 0, or -1 for a synth / channel out of range. */
int AMY_GEM_bind_synth(uint8_t synth, int channel);

/* The synths channel `channel` plays, as a mask: bit n - 1 = synth n. */
uint16_t AMY_GEM_channel_synths(uint8_t channel);

/* The bit picoruby-midi's transport registry assigned to AMY (0 until
 * AMY_GEM_start() succeeded). This is the transport_id seen from Ruby. */
uint8_t AMY_GEM_transport_bit(void);

/* Smoothed fraction of real time spent rendering (0.0 .. 1.0+). */
float AMY_GEM_render_load(void);

/* How many times the overload failsafe has reset the engine. */
uint32_t AMY_GEM_overload_count(void);

/* Blocks rendered and written to I2S since start (5.8 ms each at the
 * default 256 frames / 44.1 kHz). Climbing = the audio task is alive. */
uint32_t AMY_GEM_block_count(void);

/* Write a text report of how long the audio task took to render each block
 * since the last reset: average / min / max render time against the block
 * period, blocks over budget, the shortest I2S wait, audible oscs and a
 * load histogram. Built with AMY_DEBUG, AMY's per-stage profile follows.
 * The figures are wall time on the audio task's core (preemption counts).
 * reset = true starts a new measurement window. Blocks for up to one audio
 * block while the audio task takes the snapshot. Returns the length
 * written, or -1 when the engine is not running. */
int AMY_GEM_perf_report(char *buf, size_t len, bool reset);

/* Write the FM-relevant state of AMY synth `synth` (as a DX7 patch lays it
 * out) into `buf` as text, one item per line:
 *   algo <n>
 *   fb <feedback>
 *   op <osc> <level> <ratio> <t0>,<v0>,<t1>,<v1>,...   (eg0 breakpoints)
 * osc is voice-relative; ratio is 0 for a fixed-frequency operator. Read
 * from the synth's first voice. Returns the length written, or -1 when the
 * engine is not running or the synth does not exist. */
int AMY_GEM_fm_state(uint8_t synth, char *buf, size_t len);

/* Write the whole configuration of AMY synth `synth` into `buf` as text,
 * one line per item, for reading a patch back (AMY::Synth#refresh):
 *   synth <num_voices> <oscs_per_voice> <synth_level>
 *   osc <n> [w=<wave>] [p=<preset>] [a=<coefs>] [f=<coefs>] [F=<coefs>]
 *           [d=<coefs>] [Q=<coefs>] [G=<filter_type>] [R=<resonance>]
 *           [m=<portamento ms>] [c=<chained osc>] [L=<mod source>]
 *           [A=<t0>,<v0>,...] [B=<t0>,<v0>,...] [T=<eg0 type>] [X=<eg1 type>]
 *   fx V=<volume> h=<reverb 4> k=<chorus 4> M=<echo 5>
 * Keys are AMY wire codes. Only values that differ from an osc's defaults
 * are listed; <coefs> is the ControlCoefficient list (const, note, vel,
 * eg0, eg1, mod0, bend, ext0, ext1, mod1) with unset slots empty and the
 * frequency constants in Hz. Osc numbers are voice-relative, read from the
 * synth's first voice. Returns the length written, or -1 when the engine
 * is not running or the synth does not exist. */
int AMY_GEM_synth_state(uint8_t synth, char *buf, size_t len);

/* Scale everything written to I2S by `gain` (0.0 .. 1.0, default 1.0).
 * A master volume for boards whose amplifier has no volume control of its
 * own; a board with a codec should set the volume there instead. Safe from
 * any task, and may be set before AMY_GEM_start(). */
void AMY_GEM_set_output_gain(float gain);
float AMY_GEM_output_gain(void);

/* Play AMY's start-up bleep (a quick test that audio comes out). */
void AMY_GEM_bleep(void);

#ifdef __cplusplus
}
#endif

#endif /* AMY_GEM_H_ */
