/*
 * Copyright (c) 2021 Electrolance Solutions
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for STM32L0 processor
 */

#include <device.h>
#include <init.h>
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>
#include <linker/linker-defs.h>
#include <string.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_bus.h>
#include <BlueNRG_LP.h>
/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
static int st_bluenrg_lp_init(const struct device *arg)
{
	uint32_t key;

	ARG_UNUSED(arg);

	key = irq_lock();

	/* Install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	if (SystemInit(SYSCLK_64M, BLE_SYSCLK_32M) != SUCCESS) {
		/* Error during system clock configuration take appropriate action */
		while (1)
			;
	}

	irq_unlock(key);

		return 0;
}

SYS_INIT(st_bluenrg_lp_init, PRE_KERNEL_1, 0);
