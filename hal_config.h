/* HAL backend selector.
 *
 * Auto-detects the target from compiler-defined device macros:
 *   STM32F042x6 → SENT_HAL_STM32F042
 *   anything else → SENT_HAL_HOST (x86 simulation via pthreads)
 *
 * Override by defining exactly one of SENT_HAL_HOST or SENT_HAL_STM32F042
 * on the compiler command line before including any SENT header. */
#ifndef SENT_HAL_CONFIG_H
#define SENT_HAL_CONFIG_H

#if defined(SENT_HAL_HOST) && defined(SENT_HAL_STM32F042)
#error "Only one HAL backend can be enabled at a time."
#endif

#if !defined(SENT_HAL_HOST) && !defined(SENT_HAL_STM32F042)
#if defined(STM32F042x6)
#define SENT_HAL_STM32F042 1
#else
#define SENT_HAL_HOST 1
#endif
#endif

#endif  /* SENT_HAL_CONFIG_H */
