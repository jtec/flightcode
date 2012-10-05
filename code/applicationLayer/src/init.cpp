/*************************************
 * stm32_rcc.c
 * author: jan
 * last modifications:
 *
 * description: functions to initialize and configure the chip peripherals
 **************************************/
/*Includes*********************************/
#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"
#include "../inc/init.h"
#include "../../convenienceLayer/inc/clAssistant.h"

void disable_JTAG_and_SWD();
/*Definitions******************************/


/*Functions********************************/
/**
 * Initializes the STM32.
 */
void Init(void)
{
	//set up NVIC
	//NVIC_Configuration();

	//disable_JTAG_and_SWD();	// since JTAG and SWD share some pins with a USART
	// TODO integrate remapping/JTAG,SWD disabling; maybe use a PinManager module where
	// each module/driver has to request access to a pin (throwing an exception if a pin is accessed twice)?
	// Initialize Singleton classes.
	//TimeBase::Init();
	//Service::Init();
}//eof



void NVIC_Configuration(void)
{
	// Vector table offset from bottom of SRAM.
	uint32_t interruptVectorTable_RAMOffset = 0;
	// Set the Vector Table base location at isr_vectorsram_offset
	NVIC_SetVectorTable(NVIC_VectTab_RAM, interruptVectorTable_RAMOffset);
	//NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
}
