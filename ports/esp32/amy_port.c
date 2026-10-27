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
#include <stdarg.h>
#include "../../include/amy_gem.h"
#include "../../include/amy_gem_config.h"

#if AMY_GEM_ENABLED

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_cpu.h"
#include "esp_rom_sys.h"
#include "esp_memory_utils.h"
#include "amy.h"
#include "midi_transport.h"   /* picoruby-midi: transport registry */
#if CONFIG_IDF_TARGET_ESP32P4
#include "hal/usb_serial_jtag_ll.h"
#include "hal/usb_wrap_ll.h"
#include "soc/io_mux_reg.h"     /* USB_INT_PHY0/1_DM/DP_GPIO_NUM, PIN_FUNC_GPIO */
#include "soc/io_mux_struct.h"  /* IO_MUX */
#include "soc/lp_system_struct.h"
#include "soc/i2s_periph.h"     /* i2s_periph_signal */
#include "esp_rom_gpio.h"
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
/* Which AMY synths each MIDI channel plays: bit n - 1 = synth n (1..16).
 * Written by AMY_GEM_bind_synth() on the Ruby task, read by whichever task
 * sends a packet; each entry is one aligned 16-bit store, so no lock. */
static volatile uint16_t s_channel_synths[16];
/* Output gain in Q15 (32768 = unity), applied to every block on its way to
 * I2S. For boards with no codec to set a volume on. */
static volatile int32_t s_gain_q15 = 32768;
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
 * Render timing (AMY.perf)
 *
 * The audio task times every block with the CPU cycle counter: render is
 * from the top of the loop to the block reaching write_samples() (deltas,
 * oscillators, mix, effects, output gain), wait is the time spent in
 * i2s_channel_write() until the DMA ring has room. Both are wall time on
 * the audio task's core, so a higher-priority task preempting the render
 * counts towards it.
 *
 * The accumulators belong to the audio task. A reader hands it a request
 * (s_perf_req) and the task answers between two blocks, so nothing is read
 * half-updated and a reset never lands in the middle of a block -- which
 * matters for AMY's own profiler (AMY_DEBUG): resetting it while a stage is
 * being timed would add the whole uptime to that stage.
 * --------------------------------------------------------------------- */

#define PERF_HIST_BUCKETS 12   /* render load in 10% steps, the last >= 110% */
#define REVERB_LINES 10        /* AMY's stereo reverb: 4 delay + 6 early-reflection lines */

typedef struct {
    uint32_t blocks;
    uint64_t render_sum;       /* cycles */
    uint32_t render_min;
    uint32_t render_max;
    uint32_t wait_min;
    uint32_t over_budget;      /* blocks that took longer to render than to play */
    uint64_t oscs_sum;         /* audible oscs, counted before each block */
    uint16_t oscs_max;
    uint32_t hist[PERF_HIST_BUCKETS];
} perf_acc_t;

typedef struct {
    bool reset;
    perf_acc_t acc;
    int8_t reverb_internal;    /* bus 0's reverb lines in internal RAM, -1 = none */
#ifdef AMY_DEBUG
    int64_t profile_us;        /* wall time the profile covers */
    struct profile profiles[NO_TAG];
#endif
} perf_req_t;

#ifdef AMY_DEBUG
extern const char *profile_tag_name(enum itags tag);
#endif

static perf_acc_t s_perf;
static uint32_t s_block_cycles;          /* how long a block plays, in cycles */
static uint32_t s_render_start;
static uint16_t s_block_oscs;
static perf_req_t *volatile s_perf_req = NULL;
static SemaphoreHandle_t s_perf_done = NULL;

static void perf_reset(void)
{
    memset(&s_perf, 0, sizeof(s_perf));
    s_perf.render_min = UINT32_MAX;
    s_perf.wait_min = UINT32_MAX;
}

static void perf_record(uint32_t render, uint32_t wait)
{
    perf_acc_t *p = &s_perf;
    p->blocks++;
    p->render_sum += render;
    if (render < p->render_min) p->render_min = render;
    if (render > p->render_max) p->render_max = render;
    if (wait < p->wait_min) p->wait_min = wait;
    if (render > s_block_cycles) p->over_budget++;
    uint64_t bucket = (uint64_t)render * 10 / s_block_cycles;
    p->hist[bucket < PERF_HIST_BUCKETS ? bucket : PERF_HIST_BUCKETS - 1]++;
    p->oscs_sum += s_block_oscs;
    if (s_block_oscs > p->oscs_max) p->oscs_max = s_block_oscs;
}

static uint16_t count_audible_oscs(void)
{
    /* Under the render lock: a patch load on another task can free oscs. */
    uint16_t n = 0;
    amy_grab_render_lock();
    for (uint16_t osc = 0; osc < AMY_OSCS; osc++) {
        if (synth[osc] != NULL && synth[osc]->status == SYNTH_AUDIBLE) n++;
    }
    amy_release_render_lock();
    return n;
}

/* Between blocks, on the audio task: answer a pending AMY_GEM_perf_report(). */
static void perf_service(void)
{
    perf_req_t *req = __atomic_exchange_n(&s_perf_req, NULL, __ATOMIC_ACQ_REL);
    if (req == NULL) return;
    req->acc = s_perf;
    /* Read on this task: the reverb lines are only allocated and freed by
     * the render thread, so they cannot go away under us. Each line can
     * land apart (AMY falls back to PSRAM line by line). */
    reverb_params_t *rev = amy_global.bus[0]->reverb.rev;
    if (rev == NULL || rev->delay_1 == NULL) {
        req->reverb_internal = -1;
    } else {
        const delay_line_t *lines[REVERB_LINES] = {
            rev->delay_1, rev->delay_2, rev->delay_3, rev->delay_4,
            rev->ref_1, rev->ref_2, rev->ref_3, rev->ref_4, rev->ref_5, rev->ref_6,
        };
        req->reverb_internal = 0;
        for (int i = 0; i < REVERB_LINES; i++) {
            if (esp_ptr_internal(lines[i]->samples)) req->reverb_internal++;
        }
    }
#ifdef AMY_DEBUG
    memcpy(req->profiles, profiles, sizeof(req->profiles));
    req->profile_us = amy_get_us() - profile_start_us;
#endif
    if (req->reset) {
        perf_reset();
#ifdef AMY_DEBUG
        amy_profiles_init();
#endif
    }
    xSemaphoreGive(s_perf_done);
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
 * would switch off the OTG -- the live TinyUSB CDC console and USB-MIDI
 * device -- while USJ, which really owns PHY1, keeps the pads in USB mode
 * underneath the I2S signals.
 *
 * Putting the pad back afterwards is not enough: for as long as the I2S
 * channel takes to set up, D+ loses its pull-up, the host sees the device
 * leave, and it did not come back until the cable was replugged (seen on
 * the Tab5, 2026-10-25). So I2S pins on USB pads never go through
 * gpio_func_sel(): the driver gets them as unused, and route_pins() does
 * what it would have done (IO_MUX to GPIO, then the GPIO matrix) without
 * touching a pad enable. Then the controller the current mux actually puts
 * on those pads -- and only that one -- is switched off (usb_pads_fixup()),
 * before the I2S signals reach them.
 * --------------------------------------------------------------------- */

static bool pin_on_pads(int pin, int dm, int dp)
{
    return pin == dm || pin == dp;
}

static bool on_usb_pads(int pin)
{
    return pin_on_pads(pin, USB_INT_PHY0_DM_GPIO_NUM, USB_INT_PHY0_DP_GPIO_NUM)
        || pin_on_pads(pin, USB_INT_PHY1_DM_GPIO_NUM, USB_INT_PHY1_DP_GPIO_NUM);
}

static bool i2s_uses_phy(int dm, int dp)
{
    return pin_on_pads(AMY_GEM_I2S_BCLK, dm, dp) || pin_on_pads(AMY_GEM_I2S_WS, dm, dp)
        || pin_on_pads(AMY_GEM_I2S_DOUT, dm, dp) || pin_on_pads(AMY_GEM_I2S_MCLK, dm, dp);
}

/* Switch off the controller whose PHY the I2S pins take, as the mux is now */
static void usb_pads_fixup(void)
{
    bool on_phy0 = i2s_uses_phy(USB_INT_PHY0_DM_GPIO_NUM, USB_INT_PHY0_DP_GPIO_NUM);
    bool on_phy1 = i2s_uses_phy(USB_INT_PHY1_DM_GPIO_NUM, USB_INT_PHY1_DP_GPIO_NUM);
    if (!on_phy0 && !on_phy1) return;

    /* sw_usb_phy_sel (with sw_hw_usb_phy_sel): USJ -> PHY1, OTG1.1 -> PHY0 */
    bool swapped = LP_SYS.usb_ctrl.sw_hw_usb_phy_sel && LP_SYS.usb_ctrl.sw_usb_phy_sel;
    if (on_phy0) {
        if (swapped) usb_wrap_ll_phy_enable_pad(&USB_WRAP, false); else usb_serial_jtag_ll_phy_enable_pad(false);
    }
    if (on_phy1) {
        if (swapped) usb_serial_jtag_ll_phy_enable_pad(false); else usb_wrap_ll_phy_enable_pad(&USB_WRAP, false);
    }
}

/* A pin for the I2S driver: unused if it is on a USB pad (route_pins()) */
static gpio_num_t driver_pin(int pin)
{
    return (pin < 0 || on_usb_pads(pin)) ? I2S_GPIO_UNUSED : (gpio_num_t)pin;
}

/* i2s_gpio_check_and_set() for an output, without its gpio_func_sel() */
static void route_out(int pin, uint32_t signal)
{
    if (pin < 0 || !on_usb_pads(pin)) return;
    IO_MUX.gpio[pin].mcu_sel = PIN_FUNC_GPIO;
    esp_rom_gpio_connect_out_signal(pin, signal, false, false);
}

/* The pins the driver was not given, as it would have routed them (master TX) */
static void route_pins(void)
{
    i2s_chan_info_t info;
    if (i2s_channel_get_info(s_tx, &info) != ESP_OK) return;
    const i2s_signal_conn_t *sig = &i2s_periph_signal[info.id];
    usb_pads_fixup();
    route_out(AMY_GEM_I2S_BCLK, sig->m_tx_bck_sig);
    route_out(AMY_GEM_I2S_WS, sig->m_tx_ws_sig);
    route_out(AMY_GEM_I2S_DOUT, sig->data_out_sig);
    route_out(AMY_GEM_I2S_MCLK, sig->mck_out_sig);
}
#else
static gpio_num_t driver_pin(int pin)
{
    return pin < 0 ? I2S_GPIO_UNUSED : (gpio_num_t)pin;
}
#endif /* CONFIG_IDF_TARGET_ESP32P4 */

/* ---------------------------------------------------------------------
 * I2S
 * --------------------------------------------------------------------- */

static size_t write_samples(const uint8_t *buffer, size_t nbytes)
{
    size_t written = 0;
    /* The block is AMY's own render buffer (amy_update() casts it to
     * uint8_t *), so it can be scaled in place. */
    int32_t gain = s_gain_q15;
    if (gain != 32768) {
        int16_t *s = (int16_t *)buffer;
        size_t n = nbytes / sizeof(int16_t);
        for (size_t i = 0; i < n; i++) {
            s[i] = (int16_t)((s[i] * gain) >> 15);
        }
    }
    uint32_t rendered = esp_cpu_get_cycle_count();
    /* Blocks until the DMA ring has room: this is what paces the task. */
    i2s_channel_write(s_tx, buffer, nbytes, &written, portMAX_DELAY);
    perf_record(rendered - s_render_start, esp_cpu_get_cycle_count() - rendered);
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
            .mclk = driver_pin(AMY_GEM_I2S_MCLK),
            .bclk = driver_pin(AMY_GEM_I2S_BCLK),
            .ws   = driver_pin(AMY_GEM_I2S_WS),
            .dout = driver_pin(AMY_GEM_I2S_DOUT),
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
#if CONFIG_IDF_TARGET_ESP32P4
    if (err == ESP_OK) route_pins();
#endif
    if (err == ESP_OK) err = i2s_channel_enable(s_tx);
    if (err != ESP_OK) {
        i2s_del_channel(s_tx);
        s_tx = NULL;
    }
    return err;
}

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
    /* Echo: its delay line (~512 KB at the default 743 ms maximum) is
     * allocated -- in PSRAM, see ram_caps_delay -- only when a script
     * first turns the echo level above 0. */
    c.features.echo = 1;

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

    /* The reverb's ten delay lines (~110 KB) are read and written every
     * sample; with them in PSRAM the reverb measured ~440 us a block on the
     * Tab5. They go to internal RAM if it has room to spare now. They are
     * allocated later (when a script first turns the reverb up), and AMY
     * falls back to ram_caps_delay if internal RAM has filled up by then. */
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    c.ram_caps_reverb = (internal_free >= AMY_GEM_REVERB_INTERNAL_MIN_FREE)
                      ? (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
                      : MALLOC_CAP_SPIRAM;

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

    esp_err_t err = i2s_setup();
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

    s_block_cycles = AMY_BLOCK_US * esp_rom_get_cpu_ticks_per_us();
    perf_reset();

    s_running = true;
    xSemaphoreGive(s_ready);

    for (;;) {
        perf_service();
        s_block_oscs = count_audible_oscs();
        s_render_start = esp_cpu_get_cycle_count();
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
    if (s_perf_done == NULL) {
        s_perf_done = xSemaphoreCreateBinary();
        if (s_perf_done == NULL) return -1;
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
    for (int ch = 0; ch < 16; ch++) s_channel_synths[ch] = 0;
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

/* AMY's MIDI input (amy_event_midi_message_received) plays channel N on
 * synth N + 1. Here the channel picks the synths from s_channel_synths
 * instead, and each one gets what AMY would have done for its own channel:
 * the instrument messages, then the notes and CC mappings. */
void amy_received_control_change(uint8_t channel, uint8_t control, uint8_t value);
void amy_received_program_change(uint8_t channel, uint8_t program);
void amy_received_pedal(uint8_t channel, uint8_t value);
void amy_received_all_notes_off(uint8_t channel);
void amy_received_pitch_bend(uint8_t channel, uint8_t low_byte, uint8_t high_byte);

static void deliver_to_synth(uint8_t synth, uint8_t *data, uint32_t len)
{
    uint8_t status = data[0] & 0xF0;
    if (status == 0xB0 && data[1] == 0x40) amy_received_pedal(synth, data[2]);
    else if (status == 0xB0 && data[1] == 0x7B) amy_received_all_notes_off(synth);
    else if (status == 0xB0) amy_received_control_change(synth, data[1], data[2]);
    else if (status == 0xC0) amy_received_program_change(synth, data[1]);
    else if (status == 0xE0) amy_received_pitch_bend(synth, data[1], data[2]);
    midi_message_handler_to_queue(status, synth, data + 1, (uint16_t)(len - 1),
                                  AMY_UNSET_VALUE((uint32_t)0), NULL, NULL);
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
    if (b1 >= 0xF0) {
        /* System messages carry no channel: AMY's own handling. */
        amy_event_midi_message_received(data, len, 0);
        return 0;
    }
    uint16_t mask = s_channel_synths[b1 & 0x0F];
    for (uint8_t synth = 1; mask != 0; synth++, mask >>= 1) {
        if (mask & 1) deliver_to_synth(synth, data, len);
    }
    return 0;
}

int AMY_GEM_bind_synth(uint8_t synth, int channel)
{
    if (synth < 1 || synth > AMY_GEM_MAX_PARTS) return -1;
    if (channel > 15) return -1;
    uint16_t bit = (uint16_t)(1u << (synth - 1));
    for (int ch = 0; ch < 16; ch++) {
        uint16_t m = s_channel_synths[ch];
        m = (ch == channel) ? (m | bit) : (m & ~bit);
        s_channel_synths[ch] = m;
    }
    return 0;
}

uint16_t AMY_GEM_channel_synths(uint8_t channel)
{
    return channel < 16 ? s_channel_synths[channel] : 0;
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

/* snprintf onto the end of buf, keeping *n at most len - 1. */
static void appendf(char *buf, size_t len, size_t *n, const char *fmt, ...)
{
    if (*n + 1 >= len) return;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + *n, len - *n, fmt, ap);
    va_end(ap);
    if (w < 0) return;
    *n += (size_t)w;
    if (*n >= len) *n = len - 1;
}

int AMY_GEM_perf_report(char *buf, size_t len, bool reset)
{
    if (!s_running || buf == NULL || len == 0) return -1;

    perf_req_t *req = heap_caps_calloc(1, sizeof(perf_req_t), MALLOC_CAP_DEFAULT);
    if (req == NULL) return -1;
    req->reset = reset;
    __atomic_store_n(&s_perf_req, req, __ATOMIC_RELEASE);
    if (xSemaphoreTake(s_perf_done, pdMS_TO_TICKS(100)) != pdTRUE) {
        /* The audio task did not come round. If it still has not taken the
         * request, withdraw it; if it has, it is about to answer. */
        if (__atomic_exchange_n(&s_perf_req, NULL, __ATOMIC_ACQ_REL) == req) {
            free(req);
            return -1;
        }
        xSemaphoreTake(s_perf_done, portMAX_DELAY);
    }

    const perf_acc_t *p = &req->acc;
    float mhz = (float)esp_rom_get_cpu_ticks_per_us();
    float block_us = (float)AMY_BLOCK_US;
    size_t n = 0;
    buf[0] = '\0';
    appendf(buf, len, &n, "AMY render: %lu blocks (%.1f s), block %lu us, CPU %.0f MHz\n",
            (unsigned long)p->blocks, p->blocks * block_us / 1e6f,
            (unsigned long)AMY_BLOCK_US, mhz);
    if (p->blocks > 0) {
        float avg = (float)p->render_sum / p->blocks / mhz;
        float min = p->render_min / mhz;
        float max = p->render_max / mhz;
        appendf(buf, len, &n, "  render  avg %.0f us (%.1f%%)  min %.0f us  max %.0f us (%.1f%%)\n",
                avg, 100.0f * avg / block_us, min, max, 100.0f * max / block_us);
        appendf(buf, len, &n, "  over budget %lu blocks, I2S wait min %.0f us\n",
                (unsigned long)p->over_budget, p->wait_min / mhz);
        appendf(buf, len, &n, "  audible oscs  avg %.1f  max %u\n",
                (float)p->oscs_sum / p->blocks, (unsigned)p->oscs_max);
        appendf(buf, len, &n, "  load");
        for (int i = 0; i < PERF_HIST_BUCKETS; i++) {
            if (p->hist[i] == 0) continue;
            if (i == PERF_HIST_BUCKETS - 1) {
                appendf(buf, len, &n, "  >=%d%%:%lu", i * 10, (unsigned long)p->hist[i]);
            } else {
                appendf(buf, len, &n, "  %d-%d%%:%lu", i * 10, i * 10 + 10, (unsigned long)p->hist[i]);
            }
        }
        appendf(buf, len, &n, "\n");
    }
    if (req->reverb_internal < 0) {
        appendf(buf, len, &n, "  reverb lines: not allocated");
    } else {
        appendf(buf, len, &n, "  reverb lines: %d/%d in internal RAM",
                req->reverb_internal, REVERB_LINES);
    }
    appendf(buf, len, &n, ", internal RAM free %u KB (largest block %u KB)\n",
            (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024),
            (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) / 1024));

#ifdef AMY_DEBUG
    /* AMY's own stages (nested: render_osc_wave includes the render_lut*
     * and filter calls under it). Timed with esp_timer, whose overhead
     * inflates the stages that run many times a block. us/block compares
     * with "render avg" above; %time is of real time, like its percentages. */
    float profile_blocks = req->profile_us / block_us;
    appendf(buf, len, &n, "AMY stages over %.1f s (AMY_DEBUG, nested):\n", req->profile_us / 1e6f);
    appendf(buf, len, &n, "  %-36s %9s %9s %7s %8s\n", "stage", "calls", "us/block", "%time", "us/call");
    for (int tag = 0; tag < NO_TAG; tag++) {
        const struct profile *t = &req->profiles[tag];
        if (t->calls == 0) continue;
        float per_block = profile_blocks > 0 ? (float)t->us_total / profile_blocks : 0.0f;
        appendf(buf, len, &n, "  %-36s %9lu %9.1f %6.1f%% %8.1f\n",
                profile_tag_name((enum itags)tag), (unsigned long)t->calls,
                per_block, 100.0f * per_block / block_us,
                (float)t->us_total / (float)t->calls);
    }
#endif

    free(req);
    return (int)n;
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

/* ---- AMY_GEM_synth_state: begin (host-tested; keep self-contained) ---- */
typedef struct {
    char *buf;
    size_t len;
    size_t n;
} state_out_t;

static void out(state_out_t *o, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

static void out(state_out_t *o, const char *fmt, ...)
{
    if (o->n >= o->len) return;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(o->buf + o->n, o->len - o->n, fmt, ap);
    va_end(ap);
    if (w > 0) o->n += (size_t)w;
}

/* " key=c0,c1,..." for a ControlCoefficient list; unset slots are left
 * empty and trailing ones dropped. Nothing when every slot is unset. */
static void out_coefs(state_out_t *o, const char *key, const float *c)
{
    int last = -1;
    for (int i = 0; i < NUM_COMBO_COEFS; i++) {
        if (AMY_IS_SET(c[i])) last = i;
    }
    if (last < 0) return;
    out(o, " %s=", key);
    for (int i = 0; i <= last; i++) {
        if (i) out(o, ",");
        if (AMY_IS_SET(c[i])) out(o, "%.6g", (double)c[i]);
    }
}

/* " key=t0,v0,t1,v1,..." for a breakpoint set. AMY reports only the
 * entries that differ from an osc's defaults, so the defaults are put back
 * here: every time defaults to 0, and eg0 defaults to the key gate
 * (0,1.0,0,0) -- an eg0 attack to 1.0 reads back with that value unset.
 * The list ends at the first entry with neither time nor value (past
 * eg0's two default entries). Nothing when the set is all defaults. */
static void out_bps(state_out_t *o, const char *key,
                    const uint32_t *t, const float *v, bool eg0)
{
    int count = 0;
    for (int i = 0; i < MAX_BREAKPOINTS; i++) {
        if (AMY_IS_UNSET(t[i]) && AMY_IS_UNSET(v[i])) {
            if (eg0 && i < 2) continue;
            break;
        }
        count = i + 1;
    }
    if (count == 0) return;
    if (eg0 && count < 2) count = 2;
    out(o, " %s=", key);
    for (int i = 0; i < count; i++) {
        float dv = (eg0 && i == 0) ? 1.0f : 0.0f;
        out(o, "%s%lu,%.4f", i ? "," : "",
            AMY_IS_SET(t[i]) ? (unsigned long)t[i] : 0UL,
            AMY_IS_SET(v[i]) ? (double)v[i] : (double)dv);
    }
}

static int synth_state_locked(uint8_t synth, char *buf, size_t len)
{
    state_out_t o = { buf, len, 0 };
    buf[0] = '\0';
    amy_event e;
    void *state = NULL;
    do {
        state = yield_synth_events(synth, &e, true, state);
        if (AMY_IS_SET(e.num_voices)) {                   /* the preamble */
            out(&o, "synth %u %u %.4f\n", (unsigned)e.num_voices,
                (unsigned)e.oscs_per_voice,
                AMY_IS_SET(e.synth_level) ? (double)e.synth_level : 1.0);
        } else if (AMY_IS_SET(e.osc)) {
            out(&o, "osc %u", (unsigned)e.osc);
            if (AMY_IS_SET(e.wave))   out(&o, " w=%u", (unsigned)e.wave);
            if (AMY_IS_SET(e.preset)) out(&o, " p=%d", (int)e.preset);
            out_coefs(&o, "a", e.amp_coefs);
            out_coefs(&o, "f", e.freq_coefs);
            out_coefs(&o, "F", e.filter_freq_coefs);
            out_coefs(&o, "d", e.duty_coefs);
            out_coefs(&o, "Q", e.pan_coefs);
            if (AMY_IS_SET(e.filter_type)) out(&o, " G=%u", (unsigned)e.filter_type);
            if (AMY_IS_SET(e.resonance))   out(&o, " R=%.4f", (double)e.resonance);
            if (AMY_IS_SET(e.portamento_ms)) out(&o, " m=%u", (unsigned)e.portamento_ms);
            if (AMY_IS_SET(e.chained_osc)) out(&o, " c=%u", (unsigned)e.chained_osc);
            if (AMY_IS_SET(e.mod_source[0])) out(&o, " L=%u", (unsigned)e.mod_source[0]);
            out_bps(&o, "A", e.eg0_times, e.eg0_values, true);
            out_bps(&o, "B", e.eg1_times, e.eg1_values, false);
            if (AMY_IS_SET(e.eg_type[0])) out(&o, " T=%u", (unsigned)e.eg_type[0]);
            if (AMY_IS_SET(e.eg_type[1])) out(&o, " X=%u", (unsigned)e.eg_type[1]);
            out(&o, "\n");
        } else if (AMY_IS_SET(e.reverb_level)) {          /* the bus effects */
            out(&o, "fx V=%.4f h=%.4f,%.4f,%.4f,%.1f k=%.4f,%.1f,%.4f,%.4f"
                " M=%.4f,%.1f,%.1f,%.4f,%.4f\n",
                AMY_IS_SET(e.volume) ? (double)e.volume : 1.0,
                (double)e.reverb_level, (double)e.reverb_liveness,
                (double)e.reverb_damping, (double)e.reverb_xover_hz,
                (double)e.chorus_level, (double)e.chorus_max_delay,
                (double)e.chorus_lfo_freq, (double)e.chorus_depth,
                (double)e.echo_level, (double)e.echo_delay_ms,
                AMY_IS_SET(e.echo_max_delay_ms) ? (double)e.echo_max_delay_ms : 0.0,
                (double)e.echo_feedback, (double)e.echo_filter_coef);
        }
    } while (state != NULL);
    if (o.n >= len) o.n = len - 1;
    return (int)o.n;
}
/* ---- AMY_GEM_synth_state: end ---- */

int AMY_GEM_synth_state(uint8_t synth, char *buf, size_t len)
{
    if (!s_running || buf == NULL || len == 0) return -1;
    if (instrument_get_num_voices(synth, NULL) < 1) return -1;
    amy_grab_render_lock();
    int n = synth_state_locked(synth, buf, len);
    amy_release_render_lock();
    return n;
}

void AMY_GEM_set_output_gain(float gain)
{
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 1.0f) gain = 1.0f;
    s_gain_q15 = (int32_t)(gain * 32768.0f + 0.5f);
}

float AMY_GEM_output_gain(void)
{
    return s_gain_q15 / 32768.0f;
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
int AMY_GEM_bind_synth(uint8_t synth, int channel) { (void)synth; (void)channel; return -1; }
uint16_t AMY_GEM_channel_synths(uint8_t channel) { (void)channel; return 0; }
uint8_t AMY_GEM_transport_bit(void) { return 0; }
float AMY_GEM_render_load(void) { return 0.0f; }
uint32_t AMY_GEM_overload_count(void) { return 0; }
uint32_t AMY_GEM_block_count(void) { return 0; }
int AMY_GEM_perf_report(char *buf, size_t len, bool reset)
{
    (void)reset;
    if (buf && len) buf[0] = '\0';
    return -1;
}
void AMY_GEM_bleep(void) {}
void AMY_GEM_set_output_gain(float gain) { (void)gain; }
float AMY_GEM_output_gain(void) { return 1.0f; }
int AMY_GEM_synth_state(uint8_t synth, char *buf, size_t len)
{
    (void)synth;
    if (buf && len) buf[0] = '\0';
    return -1;
}
int AMY_GEM_fm_state(uint8_t synth, char *buf, size_t len)
{
    (void)synth;
    if (buf && len) buf[0] = '\0';
    return -1;
}

#endif /* AMY_GEM_ENABLED */
