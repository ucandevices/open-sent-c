/* STM32F042 HAL backend for SENT RX and TX.
 *
 * RX: a TIMx input-capture ISR calls sent_stm32f042_rx_on_capture_edge_isr()
 *     on each falling edge; overflow events call sent_stm32f042_rx_on_overflow_isr().
 *     The main loop drains completed batches via the poll_timestamps_us HAL hook.
 *
 * TX: the main loop calls submit_frame; a TIM14 compare ISR drives the SENT
 *     output pin by popping one toggle interval at a time from the flat array
 *     via sent_stm32f042_tx_pop_next_interval_ticks_from_isr(). */
#ifndef SENT_HAL_STM32F042_H
#define SENT_HAL_STM32F042_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sent/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENT_STM32F042_RX_MAX_BATCH_SIZE    SENT_MAX_TIMESTAMPS      /* timestamps per batch (13) */
#define SENT_STM32F042_RX_MAX_READY_BATCHES 3U                       /* ISR→main ring-buffer depth */
#define SENT_STM32F042_TX_MAX_INTERVALS     SENT_MAX_INTERVALS        /* tick intervals per frame */
#define SENT_STM32F042_TX_MAX_TOGGLES       (SENT_MAX_INTERVALS * 2U) /* 2 toggle edges per interval */

/* Compile-time configuration passed to sent_stm32f042_rx_hal_init(). */
typedef struct {
    uint32_t timer_clock_hz;         /* input-capture timer frequency [Hz] */
    uint16_t timer_autoreload;       /* timer ARR value (determines overflow period) */
    uint8_t capture_batch_size;      /* edges per batch; max SENT_STM32F042_RX_MAX_BATCH_SIZE (13) */
    uint8_t ready_queue_depth;       /* ISR→main ring depth; max SENT_STM32F042_RX_MAX_READY_BATCHES */
    /* Sync-detection threshold [µs].  When a captured interval is >= this value
     * and active_count > 0, the current partial batch is discarded and a fresh
     * batch is started with the preceding edge (the sync falling-edge) as
     * timestamp[0].  Set to 0 to disable.
     * For MLX90377 at 3 µs tick: sync = 168 µs, max nibble = 81 µs → use 100 µs.
     * If two consecutive long intervals occur (long pause then sync), the second
     * long interval re-fires and corrects the alignment automatically. */
    uint32_t sync_min_us;
} sent_stm32f042_rx_config_t;

/* One completed timestamp batch passed from ISR to main loop. */
typedef struct {
    uint32_t timestamps_us[SENT_STM32F042_RX_MAX_BATCH_SIZE];
    uint8_t count;
} sent_stm32f042_rx_batch_t;

/* Runtime state for the STM32F042 RX HAL.  ISR and main-loop fields are
 * split by volatility: volatile for ISR-written counters, plain for main-only. */
typedef struct {
    sent_stm32f042_rx_config_t config;
    volatile uint32_t overflow_count;          /* incremented by overflow ISR */
    volatile uint32_t dropped_batches;         /* ready queue was full at push time */
    uint64_t last_counter_ticks;               /* absolute tick count at last edge */
    uint32_t last_timestamp_us;                /* accumulated timestamp [us] */
    uint32_t ticks_to_us_mul_q12;              /* Q12 fixed-point multiplier (computed at start) */
    uint32_t ticks_to_us_frac_q12;             /* Q12 fractional accumulator */
    uint32_t ticks_to_us_max_delta;            /* largest tick chunk that won't overflow mul_q12 */
    uint32_t active_timestamps_us[SENT_STM32F042_RX_MAX_BATCH_SIZE];  /* edges being accumulated */
    sent_stm32f042_rx_batch_t ready_batches[SENT_STM32F042_RX_MAX_READY_BATCHES];  /* ring buffer */
    volatile uint8_t ready_head;               /* ISR writes here */
    volatile uint8_t ready_tail;               /* main loop reads here */
    uint8_t active_count;                      /* edges in the active buffer */
    bool running;                              /* gates ISR processing */
} sent_stm32f042_rx_hal_t;

/* Compile-time configuration passed to sent_stm32f042_tx_hal_init(). */
typedef struct {
    uint16_t default_pause_ticks;    /* inter-frame gap [ticks] used when submit passes 0 */
    uint8_t  low_ticks;              /* LOW-phase duration per interval [ticks]; SAE J2716 min = 5 */
    uint16_t tx_tick_x10_us;         /* TX tick period [0.1 us units]; 30 = 3.0 us (SAE J2716 default) */
} sent_stm32f042_tx_config_t;

/* Runtime state for the STM32F042 TX HAL. */
typedef struct {
    sent_stm32f042_tx_config_t config;
    /* Pre-expanded toggle durations: [LOW, HIGH, LOW, HIGH, ...] for each SENT interval.
     * Written by stm32_tx_submit; read by ISR. count published last (M0 store barrier). */
    uint16_t intervals[SENT_STM32F042_TX_MAX_TOGGLES];
    volatile uint8_t count;          /* total toggle entries; 0 = idle */
    uint8_t active_index;            /* ISR-only: index of the next toggle to pop */
    bool running;
} sent_stm32f042_tx_hal_t;

/* Initialize RX HAL with the given timer config; NULL uses 48 MHz defaults. */
void sent_stm32f042_rx_hal_init(sent_stm32f042_rx_hal_t* hal,
                                const sent_stm32f042_rx_config_t* config);
/* Initialize TX HAL with the given config; NULL uses SAE J2716 defaults. */
void sent_stm32f042_tx_hal_init(sent_stm32f042_tx_hal_t* hal,
                                const sent_stm32f042_tx_config_t* config);

/* Populate out_hal with STM32 RX function pointers backed by impl. */
void sent_stm32f042_make_rx_hal(sent_stm32f042_rx_hal_t* impl, sent_rx_hal_t* out_hal);
/* Populate out_hal with STM32 TX function pointers backed by impl. */
void sent_stm32f042_make_tx_hal(sent_stm32f042_tx_hal_t* impl, sent_tx_hal_t* out_hal);

/* Call from the input-capture ISR on each falling edge with the raw counter value. */
void sent_stm32f042_rx_on_capture_edge_isr(sent_stm32f042_rx_hal_t* hal,
                                            uint16_t captured_counter);
/* Call from the timer overflow (update) ISR to extend the 16-bit counter to 64-bit. */
void sent_stm32f042_rx_on_overflow_isr(sent_stm32f042_rx_hal_t* hal);
/* Returns the number of timestamp batches lost because the ready queue was full. */
uint32_t sent_stm32f042_rx_dropped_batches(const sent_stm32f042_rx_hal_t* hal);

/* Call from the TIM14 compare ISR; returns false when the frame is complete. */
bool sent_stm32f042_tx_pop_next_interval_ticks_from_isr(sent_stm32f042_tx_hal_t* hal,
                                                         uint16_t* out_interval_ticks);
/* Returns 1 if a frame is in progress, 0 if idle. */
size_t sent_stm32f042_tx_pending_frames(const sent_stm32f042_tx_hal_t* hal);
/* Returns the configured TX tick period [0.1 us units]. */
uint16_t sent_stm32f042_tx_get_tick_x10_us(const sent_stm32f042_tx_hal_t* hal);

#ifdef __cplusplus
}
#endif

#endif  /* SENT_HAL_STM32F042_H */
