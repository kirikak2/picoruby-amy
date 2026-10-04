/*
 * PicoRuby AMY - ESP32 port
 *
 * One audio task does everything real-time: it renders an AMY block and
 * writes it to the I2S channel, whose DMA queue paces the loop (the write
 * blocks until a block's worth of room frees up, i.e. every 5.8 ms at
 * 256 frames / 44.1 kHz). AMY's own render / fill tasks are not used
 * (platform.multicore = platform.multithread = 0), so the priority and
 * core of this one task are the only scheduling knobs.
 *
 * AMY's built-in ESP32 I2S output is not used either: it is fixed to the
 * MSB slot format and an auto-picked port, while a board may need
 * standard I2S (e.g. NS4168 amplifiers) or a given port. AMY runs with
 * audio = AMY_AUDIO_IS_NONE and hands us each block through
 * write_samples_fn instead.
 *
 * The audio task never writes to the console. Where the console is a USB
 * CDC port (Tab5 / CrowPanel in midi_device mode), writing to it from any
 * core other than TinyUSB's corrupts the device stack and the console goes
 * dead (see CLAUDE.md, console_input.c). The task runs on core 0, so its
 * stdout / stderr -- per task under newlib -- go to /dev/null, which also
 * covers AMY's own fprintf(stderr, ...) and the board power callback's
 * logs. Start-up progress is logged by AMY_GEM_start() on the caller's
 * task instead.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../include/amy_gem.h"
#include "../../include/amy_gem_config.h"

#if AMY_GEM_ENABLED

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "amy.h"
#include "midi_transport.h"   /* picoruby-midi: transport registry */
#if CONFIG_IDF_TARGET_ESP32P4
#include "hal/usb_serial_jtag_ll.h"
#include "hal/usb_wrap_ll.h"
#include "soc/io_mux_reg.h"     /* USB_INT_PHY0/1_DM/DP_GPIO_NUM */
#endif

static const char *TAG = "AMY";

static amy_gem_power_cb_t s_power_cb = NULL;
static void *s_power_arg = NULL;

static i2s_chan_handle_t s_tx = NULL;
static TaskHandle_t s_task = NULL;
static SemaphoreHandle_t s_ready = NULL;
static volatile bool s_running = false;
static volatile bool s_start_failed = false;
static volatile uint32_t s_overloads = 0;
static volatile esp_err_t s_i2s_err = ESP_OK;
static volatile uint32_t s_blocks = 0;     /* blocks rendered: proof of life */
static uint8_t s_transport_bit = 0;

/* How far the audio task got in starting up, for the timeout message in
 * AMY_GEM_start(). */
typedef enum {
    STEP_TASK_CREATED = 0,
    STEP_I2S_READY,
    STEP_POWER_ON,
    STEP_AMY_STARTED,
} start_step_t;
static volatile start_step_t s_step = STEP_TASK_CREATED;
static const char *const s_step_names[] = {
    "task created (before I2S setup)",
    "I2S running (before the board power callback)",
    "board power on (inside amy_start)",
    "AMY started",
};

#define AMY_START_TIMEOUT_MS 5000

/* ---------------------------------------------------------------------
 * I2S
 * --------------------------------------------------------------------- */

static size_t write_samples(const uint8_t *buffer, size_t nbytes)
{
    size_t written = 0;
    /* Blocks until the DMA ring has room: this is what paces the task. */
    i2s_channel_write(s_tx, buffer, nbytes, &written, portMAX_DELAY);
    return written;
}

static esp_err_t i2s_setup(void)
{
    i2s_port_t port = (AMY_GEM_I2S_PORT < 0) ? I2S_NUM_AUTO
                                             : (i2s_port_t)AMY_GEM_I2S_PORT;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(port, I2S_ROLE_MASTER);
    /* One descriptor per AMY block, so the ring is sized (and the latency
     * counted) in blocks. */
    chan_cfg.dma_desc_num = AMY_GEM_DMA_BLOCKS;
    chan_cfg.dma_frame_num = AMY_BLOCK_SIZE;
    /* Play silence, not the last block over and over, if we ever fall
     * behind. */
    chan_cfg.auto_clear = true;

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    if (err != ESP_OK) return err;

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AMY_SAMPLE_RATE),
#if AMY_GEM_I2S_PHILIPS
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
#else
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                    I2S_SLOT_MODE_STEREO),
#endif
        .gpio_cfg = {
            .mclk = (AMY_GEM_I2S_MCLK < 0) ? I2S_GPIO_UNUSED : (gpio_num_t)AMY_GEM_I2S_MCLK,
            .bclk = (gpio_num_t)AMY_GEM_I2S_BCLK,
            .ws   = (gpio_num_t)AMY_GEM_I2S_WS,
            .dout = (gpio_num_t)AMY_GEM_I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = (i2s_mclk_multiple_t)AMY_GEM_MCLK_MULTIPLE;

    err = i2s_channel_init_std_mode(s_tx, &std_cfg);
    if (err == ESP_OK) err = i2s_channel_enable(s_tx);
    if (err != ESP_OK) {
        i2s_del_channel(s_tx);
        s_tx = NULL;
    }
    return err;
}

#if CONFIG_IDF_TARGET_ESP32P4
/* ---------------------------------------------------------------------
 * ESP32-P4: I2S pins on the USB PHY pads
 *
 * GPIO 24/25 and 26/27 are the pads of the P4's two internal full-speed USB
 * PHYs. Routing a pad to another function goes through gpio_ll_func_sel(),
 * which also switches the USB pads off -- assuming the default mux, PHY0 ->
 * USB-Serial/JTAG and PHY1 -> USB OTG1.1: GPIO 26/27 clear the OTG's pad
 * enable, 24/25 the USJ's. A board that swapped the mux (Tab5 in
 * midi_device mode: OTG1.1 on PHY0 behind its USB-C connector, see
 * picoruby-usb_midi_device) gets the wrong controller cut: I2S on 26/27
 * switched off the OTG -- the live TinyUSB CDC console and USB-MIDI device
 * dropped -- while USJ, which really owns PHY1, kept the pads in USB mode
 * underneath the I2S signals.
 *
 * Take a snapshot of both pad enables before the I2S pins are routed, put
 * them back afterwards, and then switch off only the controller that the
 * current mux actually puts on the pads I2S took.
 * --------------------------------------------------------------------- */

static bool pin_on_pads(int pin, int dm, int dp)
{
    return pin == dm || pin == dp;
}

static bool i2s_uses_phy(int dm, int dp)
{
    return pin_on_pads(AMY_GEM_I2S_BCLK, dm, dp) || pin_on_pads(AMY_GEM_I2S_WS, dm, dp)
        || pin_on_pads(AMY_GEM_I2S_DOUT, dm, dp) || pin_on_pads(AMY_GEM_I2S_MCLK, dm, dp);
}

typedef struct {
    bool otg_pad;
    bool usj_pad;
} usb_pads_t;

static usb_pads_t usb_pads_save(void)
{
    usb_pads_t p = {
        .otg_pad = usb_wrap_ll_phy_is_pad_enabled(&USB_WRAP),
        .usj_pad = usb_serial_jtag_ll_phy_is_pad_enabled(),
    };
    return p;
}

static void p4_usb_pads_fixup(usb_pads_t before)
{
    bool on_phy0 = i2s_uses_phy(USB_INT_PHY0_DM_GPIO_NUM, USB_INT_PHY0_DP_GPIO_NUM);
    bool on_phy1 = i2s_uses_phy(USB_INT_PHY1_DM_GPIO_NUM, USB_INT_PHY1_DP_GPIO_NUM);
    if (!on_phy0 && !on_phy1) return;

    /* sw_usb_phy_sel (with sw_hw_usb_phy_sel): USJ -> PHY1, OTG1.1 -> PHY0 */
    bool swapped = LP_SYS.usb_ctrl.sw_hw_usb_phy_sel && LP_SYS.usb_ctrl.sw_usb_phy_sel;

    bool otg_pad = before.otg_pad;
    bool usj_pad = before.usj_pad;
    if (on_phy0) {
        if (swapped) otg_pad = false; else usj_pad = false;
    }
    if (on_phy1) {
        if (swapped) usj_pad = false; else otg_pad = false;
    }
    usb_wrap_ll_phy_enable_pad(&USB_WRAP, otg_pad);
    usb_serial_jtag_ll_phy_enable_pad(usj_pad);
}
#endif /* CONFIG_IDF_TARGET_ESP32P4 */

/* ---------------------------------------------------------------------
 * Audio task
 * --------------------------------------------------------------------- */

static void overload_hook(float load)
{
    /* Called on the audio task after AMY has already silenced and reset
     * itself. Only counted (AMY.overloads): no logging from this task. */
    (void)load;
    s_overloads++;
}

static amy_config_t make_config(void)
{
    amy_config_t c = amy_default_config();

    c.audio = AMY_AUDIO_IS_NONE;          /* we own the I2S channel */
    c.midi = AMY_MIDI_IS_NONE;            /* MIDI arrives via the transport */
    c.write_samples_fn = write_samples;

    /* Everything on this task (see the file comment). */
    c.platform.multicore = 0;
    c.platform.multithread = 0;

    c.features.default_synths = 0;        /* scripts set up their own synths */
    c.features.startup_bleep = 0;
    c.features.audio_in = 0;
    c.features.echo = 0;                  /* a few hundred KB of delay line */

    c.overload_threshold = AMY_GEM_OVERLOAD_THRESHOLD;
    c.amy_external_overload_hook = overload_hook;

    /* Big, rarely-touched state in PSRAM; the per-block render buffers
     * stay in internal RAM (MALLOC_CAP_DEFAULT prefers it for small
     * allocations). Nothing here is static, so the internal .bss layout
     * is untouched (docs/MEMORY_ALLOCATION.md). */
    c.ram_caps_events = MALLOC_CAP_SPIRAM;
    c.ram_caps_oscs   = MALLOC_CAP_SPIRAM;
    c.ram_caps_synth  = MALLOC_CAP_SPIRAM;
    c.ram_caps_sysex  = MALLOC_CAP_SPIRAM;
    c.ram_caps_delay  = MALLOC_CAP_SPIRAM;
    c.ram_caps_sample = MALLOC_CAP_SPIRAM;
    c.ram_caps_block  = MALLOC_CAP_DEFAULT;
    c.ram_caps_fbl    = MALLOC_CAP_DEFAULT;

    return c;
}

static void audio_task(void *arg)
{
    (void)arg;

    /* Off the console for good (see the file comment). */
    FILE *null_out = fopen("/dev/null", "w");
    if (null_out != NULL) {
        stdout = null_out;
        stderr = null_out;
    }

#if CONFIG_IDF_TARGET_ESP32P4
    usb_pads_t pads_before = usb_pads_save();
#endif
    esp_err_t err = i2s_setup();
#if CONFIG_IDF_TARGET_ESP32P4
    p4_usb_pads_fixup(pads_before);
#endif
    if (err != ESP_OK) {
        s_i2s_err = err;
        s_start_failed = true;
        xSemaphoreGive(s_ready);
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    s_step = STEP_I2S_READY;

    /* The clocks are running (auto_clear feeds silence), so a codec that
     * needs MCLK can be configured now. */
    if (s_power_cb) s_power_cb(true, s_power_arg);
    s_step = STEP_POWER_ON;

    /* amy_start() must run on the task that calls amy_update(): AMY
     * registers the calling task as the one it notifies. */
    amy_start(make_config());
    s_step = STEP_AMY_STARTED;

    s_running = true;
    xSemaphoreGive(s_ready);

    for (;;) {
        amy_update();   /* render one block, write it via write_samples() */
        s_blocks++;
    }
}

/* ---------------------------------------------------------------------
 * MIDI transport
 * --------------------------------------------------------------------- */

static int amy_tx_send_packet(void *ctx, uint8_t cable, uint8_t cin,
                              uint8_t b1, uint8_t b2, uint8_t b3)
{
    (void)ctx;
    return AMY_GEM_send_packet(cable, cin, b1, b2, b3);
}

static bool amy_tx_is_connected(void *ctx)
{
    (void)ctx;
    return s_running;
}

static const midi_transport_ops_t s_transport_ops = {
    .send_packet     = amy_tx_send_packet,
    .read_bytes      = NULL,                    /* send-only */
    .bytes_available = NULL,
    .is_connected    = amy_tx_is_connected,
    .transport_id    = MIDI_TRANSPORT_ID_NONE,  /* not a wire transport */
};

static const midi_transport_t s_transport = {
    .ops = &s_transport_ops,
    .ctx = NULL,
};

/* ---------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void AMY_GEM_set_power_callback(amy_gem_power_cb_t cb, void *arg)
{
    s_power_cb = cb;
    s_power_arg = arg;
}

int AMY_GEM_start(void)
{
    if (s_running) return 0;
    if (s_task != NULL) {
        /* A start is already in flight (or timed out earlier). */
        ESP_LOGE(TAG, "start still pending; audio task at: %s", s_step_names[s_step]);
        return -1;
    }

    if (s_ready == NULL) {
        s_ready = xSemaphoreCreateBinary();
        if (s_ready == NULL) return -1;
    }
    s_start_failed = false;

    BaseType_t ok = xTaskCreatePinnedToCore(audio_task, "amy_audio",
                                            AMY_GEM_TASK_STACK, NULL,
                                            AMY_GEM_TASK_PRIORITY, &s_task,
                                            AMY_GEM_TASK_CORE);
    if (ok != pdPASS) {
        s_task = NULL;
        ESP_LOGE(TAG, "failed to create the audio task");
        return -1;
    }

    if (xSemaphoreTake(s_ready, pdMS_TO_TICKS(AMY_START_TIMEOUT_MS)) != pdTRUE) {
        /* Leave the task alone: it may still finish, and AMY.start can be
         * retried (s_task stays set, so a retry reports instead of
         * starting a second engine). */
        ESP_LOGE(TAG, "start timed out after %d ms; audio task stuck at: %s",
                 AMY_START_TIMEOUT_MS, s_step_names[s_step]);
        return -1;
    }
    if (s_start_failed || !s_running) {
        ESP_LOGE(TAG, "I2S setup failed: %s", esp_err_to_name(s_i2s_err));
        return -1;
    }

    ESP_LOGI(TAG, "AMY running: %d Hz, %d-frame blocks, I2S BCLK=%d WS=%d DOUT=%d MCLK=%d (x%d)",
             AMY_SAMPLE_RATE, AMY_BLOCK_SIZE,
             AMY_GEM_I2S_BCLK, AMY_GEM_I2S_WS, AMY_GEM_I2S_DOUT, AMY_GEM_I2S_MCLK,
             AMY_GEM_MCLK_MULTIPLE);

    s_transport_bit = MIDI_transport_register(&s_transport);
    if (s_transport_bit == 0) {
        ESP_LOGW(TAG, "MIDI transport registry full: trigger() will not reach AMY");
    }
    return 0;
}

bool AMY_GEM_running(void)
{
    return s_running;
}

void AMY_GEM_reset(void)
{
    if (!s_running) return;
    /* RESET_ALL_OSCS: frees every osc and also clears synths, patches,
     * MIDI mappings and the bus effects (amy_reset_oscs()). */
    char msg[16];
    snprintf(msg, sizeof(msg), "S%dZ", RESET_ALL_OSCS);
    amy_add_message(msg);
}

int AMY_GEM_send_message(const char *message)
{
    if (!s_running || message == NULL) return -1;
    /* amy_add_message() takes a mutable buffer; never hand it the
     * caller's (possibly read-only) string. */
    size_t len = strlen(message);
    char stack_buf[256];
    char *buf = (len < sizeof(stack_buf)) ? stack_buf : malloc(len + 1);
    if (buf == NULL) return -1;
    memcpy(buf, message, len + 1);
    amy_add_message(buf);
    if (buf != stack_buf) free(buf);
    return 0;
}

int AMY_GEM_send_packet(uint8_t cable, uint8_t cin,
                        uint8_t b1, uint8_t b2, uint8_t b3)
{
    (void)cable;
    if (!s_running) return -1;

    /* USB-MIDI Code Index Number -> MIDI message length. Only channel
     * voice and single-byte realtime/common messages; SysEx (CIN 0x4-0x7)
     * is not passed on. */
    uint32_t len;
    switch (cin & 0x0F) {
    case 0x8: case 0x9: case 0xA: case 0xB: case 0xE:
        len = 3; break;
    case 0xC: case 0xD:
        len = 2; break;
    case 0x5: case 0xF:
        len = 1; break;
    default:
        return -1;
    }
    uint8_t data[3] = { b1, b2, b3 };
    amy_event_midi_message_received(data, len, 0);
    return 0;
}

uint8_t AMY_GEM_transport_bit(void)
{
    return s_transport_bit;
}

float AMY_GEM_render_load(void)
{
    return s_running ? amy_get_render_load() : 0.0f;
}

uint32_t AMY_GEM_overload_count(void)
{
    return s_overloads;
}

uint32_t AMY_GEM_block_count(void)
{
    return s_blocks;
}

int AMY_GEM_fm_state(uint8_t synth, char *buf, size_t len)
{
    if (!s_running || buf == NULL || len == 0) return -1;
    if (instrument_get_num_voices(synth, NULL) < 1) return -1;

    size_t n = 0;
    buf[0] = '\0';
#define APPEND(...) do { \
        if (n < len) { \
            int w = snprintf(buf + n, len - n, __VA_ARGS__); \
            if (w > 0) n += (size_t)w; \
        } \
    } while (0)

    /* Under the render lock: a render (or a patch load's flush) may be
     * changing the oscs we read. Only formatting happens inside. */
    amy_grab_render_lock();
    amy_event e;
    void *state = NULL;
    do {
        state = yield_synth_events(synth, &e, false, state);
        if (AMY_IS_UNSET(e.osc)) continue;               /* the preamble */
        if (e.osc == 0) {
            if (AMY_IS_SET(e.algorithm)) APPEND("algo %u\n", (unsigned)e.algorithm);
            if (AMY_IS_SET(e.feedback))  APPEND("fb %.4f\n", (double)e.feedback);
            continue;
        }
        if (AMY_IS_UNSET(e.amp_coefs[0])) continue;      /* not an operator */
        APPEND("op %u %.4f %.4f ", (unsigned)e.osc, (double)e.amp_coefs[0],
               AMY_IS_SET(e.ratio) ? (double)e.ratio : 0.0);
        /* A breakpoint list ends where both time and value are unset; a
         * DX7 patch leaves the first time unset, meaning 0. */
        for (int i = 0; i < MAX_BREAKPOINTS; i++) {
            bool has_t = AMY_IS_SET(e.eg0_times[i]);
            bool has_v = AMY_IS_SET(e.eg0_values[i]);
            if (!has_t && !has_v) break;
            APPEND("%s%lu,%.4f", i ? "," : "",
                   has_t ? (unsigned long)e.eg0_times[i] : 0UL,
                   has_v ? (double)e.eg0_values[i] : 0.0);
        }
        APPEND("\n");
    } while (state != NULL);
    amy_release_render_lock();
#undef APPEND
    if (n >= len) n = len - 1;
    return (int)n;
}

void AMY_GEM_bleep(void)
{
    if (!s_running) return;
    /* amy_bleep() schedules its three events relative to `start`, which
     * is in AMY time, not 0. */
    amy_bleep(amy_sysclock());
}

#else  /* !AMY_GEM_ENABLED: link-time stubs */

void AMY_GEM_set_power_callback(amy_gem_power_cb_t cb, void *arg) { (void)cb; (void)arg; }
int AMY_GEM_start(void) { return -1; }
bool AMY_GEM_running(void) { return false; }
void AMY_GEM_reset(void) {}
int AMY_GEM_send_message(const char *message) { (void)message; return -1; }
int AMY_GEM_send_packet(uint8_t cable, uint8_t cin, uint8_t b1, uint8_t b2, uint8_t b3)
{
    (void)cable; (void)cin; (void)b1; (void)b2; (void)b3;
    return -1;
}
uint8_t AMY_GEM_transport_bit(void) { return 0; }
float AMY_GEM_render_load(void) { return 0.0f; }
uint32_t AMY_GEM_overload_count(void) { return 0; }
uint32_t AMY_GEM_block_count(void) { return 0; }
void AMY_GEM_bleep(void) {}
int AMY_GEM_fm_state(uint8_t synth, char *buf, size_t len)
{
    (void)synth;
    if (buf && len) buf[0] = '\0';
    return -1;
}

#endif /* AMY_GEM_ENABLED */
