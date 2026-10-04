/*
 * PicoRuby AMY - mrubyc bindings
 *
 * Thin: every method maps onto one AMY_GEM_* call. The Ruby layer in
 * mrblib/amy.rb builds wire messages and owns all the naming.
 */

#include <math.h>
#include <mrubyc.h>
#include <alloc.h>

#include "../../include/amy_gem.h"

/* AMY._start -> true / false */
static void
c_amy_start(mrbc_vm *vm, mrbc_value v[], int argc)
{
    (void)argc;
    if (AMY_GEM_start() == 0) {
        SET_TRUE_RETURN();
    } else {
        SET_FALSE_RETURN();
    }
}

/* AMY._running */
static void
c_amy_running(mrbc_vm *vm, mrbc_value v[], int argc)
{
    (void)argc;
    if (AMY_GEM_running()) {
        SET_TRUE_RETURN();
    } else {
        SET_FALSE_RETURN();
    }
}

/* AMY._reset */
static void
c_amy_reset(mrbc_vm *vm, mrbc_value v[], int argc)
{
    (void)argc;
    AMY_GEM_reset();
    SET_NIL_RETURN();
}

/* AMY._send(wire_message) -> 0 / -1 */
static void
c_amy_send(mrbc_vm *vm, mrbc_value v[], int argc)
{
    if (argc != 1 || mrbc_type(v[1]) != MRBC_TT_STRING) {
        SET_INT_RETURN(-1);
        return;
    }
    SET_INT_RETURN(AMY_GEM_send_message(mrbc_string_cstr(&v[1])));
}

/* AMY._send_packet(cable, cin, b1, b2, b3) -> 0 / -1 */
static void
c_amy_send_packet(mrbc_vm *vm, mrbc_value v[], int argc)
{
    if (argc != 5) {
        SET_INT_RETURN(-1);
        return;
    }
    int ret = AMY_GEM_send_packet((uint8_t)GET_INT_ARG(1), (uint8_t)GET_INT_ARG(2),
                                  (uint8_t)GET_INT_ARG(3), (uint8_t)GET_INT_ARG(4),
                                  (uint8_t)GET_INT_ARG(5));
    SET_INT_RETURN(ret);
}

/* AMY._transport_bit */
static void
c_amy_transport_bit(mrbc_vm *vm, mrbc_value v[], int argc)
{
    (void)argc;
    SET_INT_RETURN(AMY_GEM_transport_bit());
}

/* AMY._render_load */
static void
c_amy_render_load(mrbc_vm *vm, mrbc_value v[], int argc)
{
    (void)argc;
    SET_FLOAT_RETURN(AMY_GEM_render_load());
}

/* AMY._overloads */
static void
c_amy_overloads(mrbc_vm *vm, mrbc_value v[], int argc)
{
    (void)argc;
    SET_INT_RETURN((mrbc_int_t)AMY_GEM_overload_count());
}

/* AMY._blocks */
static void
c_amy_blocks(mrbc_vm *vm, mrbc_value v[], int argc)
{
    (void)argc;
    SET_INT_RETURN((mrbc_int_t)AMY_GEM_block_count());
}

/* AMY._bleep */
static void
c_amy_bleep(mrbc_vm *vm, mrbc_value v[], int argc)
{
    (void)argc;
    AMY_GEM_bleep();
    SET_NIL_RETURN();
}

static float
arg_float(mrbc_value *v)
{
    switch (mrbc_type(*v)) {
    case MRBC_TT_INTEGER: return (float)mrbc_integer(*v);
    case MRBC_TT_FLOAT:   return (float)mrbc_float(*v);
    default:              return 0.0f;
    }
}

/* AMY._log_unscale(x, min, max) -> t in 0..1 with min * (max / min) ** t == x
 * (the inverse of _log_scale, for putting a value back on a knob). */
static void
c_amy_log_unscale(mrbc_vm *vm, mrbc_value v[], int argc)
{
    if (argc != 3) {
        SET_NIL_RETURN();
        return;
    }
    float x = arg_float(&v[1]);
    float lo = arg_float(&v[2]);
    float hi = arg_float(&v[3]);
    float t;
    if (lo <= 0.0f || hi <= 0.0f || x <= 0.0f || hi == lo) {
        t = (hi == lo) ? 0.0f : (x - lo) / (hi - lo);
    } else {
        t = logf(x / lo) / logf(hi / lo);
    }
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    SET_FLOAT_RETURN(t);
}

/* AMY._fm_state(synth) -> String (see AMY_GEM_fm_state), or nil */
static void
c_amy_fm_state(mrbc_vm *vm, mrbc_value v[], int argc)
{
    if (argc != 1) {
        SET_NIL_RETURN();
        return;
    }
    static const size_t len = 1024;
    char *buf = mrbc_raw_alloc(len);
    if (buf == NULL) {
        SET_NIL_RETURN();
        return;
    }
    int n = AMY_GEM_fm_state((uint8_t)GET_INT_ARG(1), buf, len);
    if (n < 0) {
        mrbc_raw_free(buf);
        SET_NIL_RETURN();
        return;
    }
    mrbc_value str = mrbc_string_new(vm, buf, n);
    mrbc_raw_free(buf);
    SET_RETURN(str);
}

/* AMY._synth_state(synth) -> String (see AMY_GEM_synth_state), or nil */
static void
c_amy_synth_state(mrbc_vm *vm, mrbc_value v[], int argc)
{
    if (argc != 1) {
        SET_NIL_RETURN();
        return;
    }
    static const size_t len = 2048;
    char *buf = mrbc_raw_alloc(len);
    if (buf == NULL) {
        SET_NIL_RETURN();
        return;
    }
    int n = AMY_GEM_synth_state((uint8_t)GET_INT_ARG(1), buf, len);
    if (n < 0) {
        mrbc_raw_free(buf);
        SET_NIL_RETURN();
        return;
    }
    mrbc_value str = mrbc_string_new(vm, buf, n);
    mrbc_raw_free(buf);
    SET_RETURN(str);
}

/* AMY._log_scale(t, min, max) -> min * (max / min) ** t
 * Exponential interpolation for AMY.scale(log: true). Done here because
 * this mruby/c build has neither Math nor Float#** (MRBC_USE_MATH = 0). */
static void
c_amy_log_scale(mrbc_vm *vm, mrbc_value v[], int argc)
{
    if (argc != 3) {
        SET_NIL_RETURN();
        return;
    }
    float t = arg_float(&v[1]);
    float lo = arg_float(&v[2]);
    float hi = arg_float(&v[3]);
    if (lo <= 0.0f || hi <= 0.0f) {
        /* Undefined on a log scale: fall back to linear. */
        SET_FLOAT_RETURN(lo + (hi - lo) * t);
        return;
    }
    SET_FLOAT_RETURN(lo * powf(hi / lo, t));
}

void
mrbc_amy_init(mrbc_vm *vm)
{
    /* Keep this init minimal (methods only, no class constants): see the
     * note in picoruby-usb_midi_device's binding about VM heap corruption
     * suspected around constant registration during picogem require. */
    mrbc_class *module_AMY = mrbc_define_module(vm, "AMY");

    mrbc_define_method(vm, module_AMY, "_start",         c_amy_start);
    mrbc_define_method(vm, module_AMY, "_running",       c_amy_running);
    mrbc_define_method(vm, module_AMY, "_reset",         c_amy_reset);
    mrbc_define_method(vm, module_AMY, "_send",          c_amy_send);
    mrbc_define_method(vm, module_AMY, "_send_packet",   c_amy_send_packet);
    mrbc_define_method(vm, module_AMY, "_transport_bit", c_amy_transport_bit);
    mrbc_define_method(vm, module_AMY, "_render_load",   c_amy_render_load);
    mrbc_define_method(vm, module_AMY, "_overloads",     c_amy_overloads);
    mrbc_define_method(vm, module_AMY, "_bleep",         c_amy_bleep);
    mrbc_define_method(vm, module_AMY, "_blocks",        c_amy_blocks);
    mrbc_define_method(vm, module_AMY, "_log_scale",     c_amy_log_scale);
    mrbc_define_method(vm, module_AMY, "_log_unscale",   c_amy_log_unscale);
    mrbc_define_method(vm, module_AMY, "_fm_state",      c_amy_fm_state);
    mrbc_define_method(vm, module_AMY, "_synth_state",   c_amy_synth_state);
}
