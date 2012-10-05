/*
 * \file InterruptManager.h
 * \brief
 * \author Jan
 */

#ifndef INTERRUPTMANAGER_H_
#define INTERRUPTMANAGER_H_

//Inclusions
#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"

//Structure declaration
class InterruptManager{
public:
	static void registerInterruptHandler(void(*interruptHandler)(), uint32_t interruptID);
	// Memory addresses of interrupt vectors, used to identify the Interrupt source
	// when calling the registerInterruptHandler([...]) function.
	// TODO use memory offsets of the vector entries and check wether the
	// table is located in SRAM or FLASH
	// TODO provide a checkIfAlreadyInUse(interruptID) to enable modules
	// to check if another module is already using an interrupt. Multiple handlers possible?
	// FIXME complete list of handlers.
	static const uint32_t SysTick_Handler 				= 0x2000003C;

	static const uint32_t USART1_Handler 				= 0x200000D4;		// TODO test
	static const uint32_t USART2_Handler 				= 0x200000D8;		// TODO test
	static const uint32_t USART3_Handler 				= 0x200000DC;
	static const uint32_t USART6_Handler 				= 0x2000015C;		// TODO test

	// TODO Test all of these:
	static const uint32_t DMA1_Stream0_Handler					= 0x2000006C;
	static const uint32_t DMA1_Stream1_Handler					= 0x20000070;
	static const uint32_t DMA1_Stream2_Handler					= 0x20000074;
	static const uint32_t DMA1_Stream3_Handler					= 0x20000078;
	static const uint32_t DMA1_Stream4_Handler					= 0x2000007C;
	static const uint32_t DMA1_Stream5_Handler					= 0x20000080;
	static const uint32_t DMA1_Stream6_Handler					= 0x20000084;
	static const uint32_t DMA1_Stream7_Handler					= 0x200000FC;

	static const uint32_t DMA2_Stream0_Handler					= 0x20000120;
	static const uint32_t DMA2_Stream1_Handler					= 0x20000124;
	static const uint32_t DMA2_Stream2_Handler					= 0x20000128;
	static const uint32_t DMA2_Stream3_Handler					= 0x2000012C;
	static const uint32_t DMA2_Stream4_Handler					= 0x20000130;

	static const uint32_t DMA2_Stream5_Handler					= 0x20000150;
	static const uint32_t DMA2_Stream6_Handler					= 0x20000154;
	static const uint32_t DMA2_Stream7_Handler					= 0x20000158;


private:
	InterruptManager();
	~InterruptManager();
};

#endif /* INTERRUPTMANAGER_H_ */
