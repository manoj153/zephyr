/*
 * Copyright (c) 2023 Efinix Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __UART_EFINIX_SAPPHIRE_PRIV_H__
#define __UART_EFINIX_SAPPHIRE_PRIV_H__

#define BSP_UART_DATA	       0x00
#define BSP_UART_STATUS	       0x04
#define BSP_UART_CLOCK_DIVIDER 0x08
#define BSP_UART_FRAME_CONFIG  0x0C

#define BSP_UART_WRITE_AVAILABILITY_MASK GENMASK(23, 16)
#define BSP_UART_READ_OCCUPANCY_MASK	 GENMASK(31, 24)

#endif /* __UART_EFINIX_SAPPHIRE_PRIV_H__ */
