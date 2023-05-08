#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_GD32F30X_CLOCKS_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_GD32F30X_CLOCKS_H_

#include "gd32-common.h"

/**
 * @name Register offsets
 * @{
 */
#define GD32_APB2RST_OFFSET       0x0CU
#define GD32_APB1RST_OFFSET       0x10U


/** @} */

/**
 * @name Clock enable/disable definitions for peripherals
 * @{
 */

#define GD32_RESET_USART1     GD32_RESET_CONFIG(APB1RST, 17U)
#define GD32_RESET_WWDGT      GD32_RESET_CONFIG(APB1RST, 11U)
#define GD32_RESET_GPIOA      GD32_RESET_CONFIG(APB2RST, 2U)
#define GD32_RESET_GPIOB      GD32_RESET_CONFIG(APB2RST, 3U)
#define GD32_RESET_GPIOC      GD32_RESET_CONFIG(APB2RST, 4U)


/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_GD32F30X_CLOCKS_H_ */