/*
 * PicoRuby AMY - mruby bindings
 *
 * Thin: every method maps onto one AMY_GEM_* call. The Ruby layer in
 * mrblib/amy.rb builds wire messages and owns all the naming.
 * Feature parity with src/mrubyc/amy.c.
 */

#include <math.h>
#include <mruby.h>
#include <mruby/presym.h>
#include <mruby/class.h>
#include <mruby/string.h>

#include "../../include/amy_gem.h"

/* AMY._start -> true / false */
static mrb_value
mrb_amy_start(mrb_state *mrb, mrb_value self)
{
    return mrb_bool_value(AMY_GEM_start() == 0);
}

/* AMY._running */
static mrb_value
mrb_amy_running(mrb_state *mrb, mrb_value self)
{
    return mrb_bool_value(AMY_GEM_running());
}

/* AMY._reset */
static mrb_value
mrb_amy_reset(mrb_state *mrb, mrb_value self)
{
    AMY_GEM_reset();
    return mrb_nil_value();
}

/* AMY._send(wire_message) -> 0 / -1 */
static mrb_value
mrb_amy_send(mrb_state *mrb, mrb_value self)
{
    const char *msg;
    mrb_get_args(mrb, "z", &msg);
    return mrb_fixnum_value(AMY_GEM_send_message(msg));
}

/* AMY._send_packet(cable, cin, b1, b2, b3) -> 0 / -1 */
static mrb_value
mrb_amy_send_packet(mrb_state *mrb, mrb_value self)
{
    mrb_int cable, cin, b1, b2, b3;
    mrb_get_args(mrb, "iiiii", &cable, &cin, &b1, &b2, &b3);
    int ret = AMY_GEM_send_packet((uint8_t)cable, (uint8_t)cin,
                                  (uint8_t)b1, (uint8_t)b2, (uint8_t)b3);
    return mrb_fixnum_value(ret);
}

/* AMY._transport_bit */
static mrb_value
mrb_amy_transport_bit(mrb_state *mrb, mrb_value self)
{
    return mrb_fixnum_value(AMY_GEM_transport_bit());
}

/* AMY._render_load */
static mrb_value
mrb_amy_render_load(mrb_state *mrb, mrb_value self)
{
    return mrb_float_value(mrb, (mrb_float)AMY_GEM_render_load());
}

/* AMY._overloads */
static mrb_value
mrb_amy_overloads(mrb_state *mrb, mrb_value self)
{
    return mrb_fixnum_value((mrb_int)AMY_GEM_overload_count());
}

/* AMY._blocks */
static mrb_value
mrb_amy_blocks(mrb_state *mrb, mrb_value self)
{
    return mrb_fixnum_value((mrb_int)AMY_GEM_block_count());
}

/* AMY._bleep */
static mrb_value
mrb_amy_bleep(mrb_state *mrb, mrb_value self)
{
    AMY_GEM_bleep();
    return mrb_nil_value();
}

/* AMY._log_unscale(x, min, max) -> t in 0..1 with min * (max / min) ** t == x
 * (the inverse of _log_scale, for putting a value back on a knob). */
static mrb_value
mrb_amy_log_unscale(mrb_state *mrb, mrb_value self)
{
    mrb_float x, lo, hi;
    mrb_get_args(mrb, "fff", &x, &lo, &hi);
    float t;
    if (lo <= 0.0 || hi <= 0.0 || x <= 0.0 || hi == lo) {
        t = (hi == lo) ? 0.0f : (float)((x - lo) / (hi - lo));
    } else {
        t = logf((float)(x / lo)) / logf((float)(hi / lo));
    }
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return mrb_float_value(mrb, (mrb_float)t);
}

/* AMY._log_scale(t, min, max) -> min * (max / min) ** t
 * Exponential interpolation for AMY.scale(log: true). */
static mrb_value
mrb_amy_log_scale(mrb_state *mrb, mrb_value self)
{
    mrb_float t, lo, hi;
    mrb_get_args(mrb, "fff", &t, &lo, &hi);
    if (lo <= 0.0 || hi <= 0.0) {
        /* Undefined on a log scale: fall back to linear. */
        return mrb_float_value(mrb, lo + (hi - lo) * t);
    }
    return mrb_float_value(mrb, (mrb_float)(lo * powf((float)(hi / lo), (float)t)));
}

/* Shared body of _fm_state / _synth_state: AMY_GEM_*_state() fills a
 * caller-supplied buffer, which becomes the returned String. */
static mrb_value
state_string(mrb_state *mrb, int (*fn)(uint8_t, char *, size_t), size_t len)
{
    mrb_int synth;
    mrb_get_args(mrb, "i", &synth);
    char *buf = (char *)mrb_malloc_simple(mrb, len);
    if (buf == NULL) {
        return mrb_nil_value();
    }
    int n = fn((uint8_t)synth, buf, len);
    mrb_value str = (n < 0) ? mrb_nil_value() : mrb_str_new(mrb, buf, n);
    mrb_free(mrb, buf);
    return str;
}

/* AMY._fm_state(synth) -> String (see AMY_GEM_fm_state), or nil */
static mrb_value
mrb_amy_fm_state(mrb_state *mrb, mrb_value self)
{
    return state_string(mrb, AMY_GEM_fm_state, 1024);
}

/* AMY._synth_state(synth) -> String (see AMY_GEM_synth_state), or nil */
static mrb_value
mrb_amy_synth_state(mrb_state *mrb, mrb_value self)
{
    return state_string(mrb, AMY_GEM_synth_state, 2048);
}

void
mrb_picoruby_amy_gem_init(mrb_state *mrb)
{
    struct RClass *m = mrb_define_module_id(mrb, MRB_SYM(AMY));

    mrb_define_module_function_id(mrb, m, MRB_SYM(_start),         mrb_amy_start,         MRB_ARGS_NONE());
    mrb_define_module_function_id(mrb, m, MRB_SYM(_running),       mrb_amy_running,       MRB_ARGS_NONE());
    mrb_define_module_function_id(mrb, m, MRB_SYM(_reset),         mrb_amy_reset,         MRB_ARGS_NONE());
    mrb_define_module_function_id(mrb, m, MRB_SYM(_send),          mrb_amy_send,          MRB_ARGS_REQ(1));
    mrb_define_module_function_id(mrb, m, MRB_SYM(_send_packet),   mrb_amy_send_packet,   MRB_ARGS_REQ(5));
    mrb_define_module_function_id(mrb, m, MRB_SYM(_transport_bit), mrb_amy_transport_bit, MRB_ARGS_NONE());
    mrb_define_module_function_id(mrb, m, MRB_SYM(_render_load),   mrb_amy_render_load,   MRB_ARGS_NONE());
    mrb_define_module_function_id(mrb, m, MRB_SYM(_overloads),     mrb_amy_overloads,     MRB_ARGS_NONE());
    mrb_define_module_function_id(mrb, m, MRB_SYM(_bleep),         mrb_amy_bleep,         MRB_ARGS_NONE());
    mrb_define_module_function_id(mrb, m, MRB_SYM(_blocks),        mrb_amy_blocks,        MRB_ARGS_NONE());
    mrb_define_module_function_id(mrb, m, MRB_SYM(_log_scale),     mrb_amy_log_scale,     MRB_ARGS_REQ(3));
    mrb_define_module_function_id(mrb, m, MRB_SYM(_log_unscale),   mrb_amy_log_unscale,   MRB_ARGS_REQ(3));
    mrb_define_module_function_id(mrb, m, MRB_SYM(_fm_state),      mrb_amy_fm_state,      MRB_ARGS_REQ(1));
    mrb_define_module_function_id(mrb, m, MRB_SYM(_synth_state),   mrb_amy_synth_state,   MRB_ARGS_REQ(1));
}

void
mrb_picoruby_amy_gem_final(mrb_state *mrb)
{
    /* The audio task keeps running across scripts; the host resets the
     * synth state between them (AMY_GEM_reset). */
}
