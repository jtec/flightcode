/*
 * \file clTimebase.h
 * \author Jan
 */

#ifndef TIMEBASE_H_
#define TIMEBASE_H_

#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"

class TimeBase{
	public:
		static void Init();
		static void waitMicrosec(int32_t t);
		static void getSystemTime(uint32_t* t);
		static uint32_t getSystemTimeMs();
		static volatile uint32_t systemTime_mSec;
		static int32_t sysTick_ticksPerMicrosecond;
		static uint32_t sysTick_reloadValue;
	private:
		// Timebase is a purely static class, so no public constructor.
		TimeBase();
		~TimeBase();
};

extern "C" void SysTick_Handler_timebase(void);

#endif /* TIMEBASE_H_ */
