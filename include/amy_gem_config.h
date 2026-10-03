/*
 * PicoRuby AMY - build-time configuration
 *
 * The gem names no board. The host project defines these (e.g. with
 * target_compile_definitions on the component that compiles
 * ports/esp32/amy_port.c); anything left undefined takes the default here.
 */

#ifndef AMY_GEM_CONFIG_H_
#define AMY_GEM_CONFIG_H_

/* 0 builds link-time stubs (AMY_GEM_start() fails), so the gem can sit in
 * a build for a board without audio output. */
#ifndef AMY_GEM_ENABLED
#  define AMY_GEM_ENABLED 0
#endif

/* I2S output pins. -1 = not connected (MCLK only when the receiving chip
 * needs a master clock, e.g. a codec). */
#ifndef AMY_GEM_I2S_BCLK
#  define AMY_GEM_I2S_BCLK -1
#endif
#ifndef AMY_GEM_I2S_WS
#  define AMY_GEM_I2S_WS -1
#endif
#ifndef AMY_GEM_I2S_DOUT
#  define AMY_GEM_I2S_DOUT -1
#endif
#ifndef AMY_GEM_I2S_MCLK
#  define AMY_GEM_I2S_MCLK -1
#endif

/* I2S controller number, or -1 to let the driver pick a free one. */
#ifndef AMY_GEM_I2S_PORT
#  define AMY_GEM_I2S_PORT -1
#endif

/* Slot format: 1 = standard I2S (Philips), 0 = MSB / left-justified. */
#ifndef AMY_GEM_I2S_PHILIPS
#  define AMY_GEM_I2S_PHILIPS 1
#endif

/* MCLK as a multiple of the sample rate. Must match what the receiving
 * codec is told (e.g. ES8388 DACFsRatio). Ignored without an MCLK pin. */
#ifndef AMY_GEM_MCLK_MULTIPLE
#  define AMY_GEM_MCLK_MULTIPLE 256
#endif

/* I2S DMA depth in AMY blocks. Each block is AMY_BLOCK_SIZE frames
 * (256 = 5.8 ms at 44.1 kHz); the ring is the slack a late block can use
 * and also the output latency. */
#ifndef AMY_GEM_DMA_BLOCKS
#  define AMY_GEM_DMA_BLOCKS 4
#endif

/* The audio task renders every block and writes it to I2S. It must win
 * the CPU the moment the DMA has room, so it sits above whatever else
 * shares its core. */
#ifndef AMY_GEM_TASK_CORE
#  define AMY_GEM_TASK_CORE 0
#endif
#ifndef AMY_GEM_TASK_PRIORITY
#  define AMY_GEM_TASK_PRIORITY 3
#endif
#ifndef AMY_GEM_TASK_STACK
#  define AMY_GEM_TASK_STACK (16 * 1024)
#endif

/* AMY resets itself when rendering stays above this fraction of real time
 * (for AMY's overload window). Below 1.0 so lower-priority tasks on the
 * audio core keep some CPU. */
#ifndef AMY_GEM_OVERLOAD_THRESHOLD
#  define AMY_GEM_OVERLOAD_THRESHOLD 0.7f
#endif

#endif /* AMY_GEM_CONFIG_H_ */
