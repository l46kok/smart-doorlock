/*
 * mcu.c
 *
 *  Created on: Oct 31, 2016
 *      Author: shuh
 */
#include "rom.h"
#include "rom_map.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "prcm.h"

void RebootMCU() {
	sl_Stop(30);
	MAP_PRCMHibernateIntervalSet(330);
	MAP_PRCMHibernateWakeupSourceEnable(PRCM_HIB_SLOW_CLK_CTR);
	MAP_PRCMHibernateEnter();
}
