/*
 * Copyright (c) 2023 Efinix Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __RISCV32_EFINIX_SAPPHIRE_SOC_H_
#define __RISCV32_EFINIX_SAPPHIRE_SOC_H_

#include "soc_common.h"
#include <zephyr/arch/common/sys_io.h>
#include <zephyr/devicetree.h>

#ifndef _ASMLANGUAGE

static inline uint32_t sapphire_read32(uint32_t addr)
{
	return sys_read32(addr);
}

static inline void sapphire_write32(uint32_t addr, uint32_t data)
{
	sys_write32(data, addr);
}

static inline void sapphire_write8(uint32_t addr, uint8_t data)
{
	sys_write8(data, addr);
}

static inline uint16_t sapphire_msb_read16(uint32_t addr)
{
	uint32_t data = sapphire_read32(addr);

	return (uint16_t)(data >> 16) & 0xFFFF;
}

static inline uint8_t sapphire_msb_read8(uint32_t addr)
{
	uint32_t data = sapphire_read32(addr);

	return (uint8_t)(data >> 24);
}

#endif /* _ASMLANGUAGE */

#endif /* __RISCV32_EFINIX_SAPPHIRE_SOC_H_ */
