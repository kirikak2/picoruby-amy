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

/* Return AMY to its power-on state: every synth, patch, CC mapping and
 * effect is cleared. The audio task and I2S keep running. */
void AMY_GEM_reset(void);

/* Queue one AMY wire message (e.g. "i1K130iv6Z"). Safe from any task.
 * Returns 0, or -1 when the engine is not running. */
int AMY_GEM_send_message(const char *message);

/* Hand one USB-MIDI packet to AMY's MIDI input (the transport's
 * send_packet). Returns 0, or -1 when not running / not a channel message. */
int AMY_GEM_send_packet(uint8_t cable, uint8_t cin,
                        uint8_t b1, uint8_t b2, uint8_t b3);

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

/* Play AMY's start-up bleep (a quick test that audio comes out). */
void AMY_GEM_bleep(void);

#ifdef __cplusplus
}
#endif

#endif /* AMY_GEM_H_ */
