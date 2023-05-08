/**
 * @file SoC configuration header for the GD32F403 SoC series.
 */

#ifndef _SOC__H_
#define _SOC__H_

#include <zephyr/sys/util.h>

#ifndef _ASMLANGUAGE
#include <gd32f30x.h>

/* The GigaDevice HAL headers define this, but it conflicts with the Zephyr can.h */
#undef CAN_MODE_NORMAL

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
