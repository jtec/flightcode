/**
 *\file InterruptManager.cpp
 *\brief This provides methods to dynamically insert interrupt handler methods into the interrupt vector table.
 * The handler methods have to be static class methods compiled as C (extern "C") or C functions.
 *\author Jan
*/

#include "../inc/clInterruptManager.h"

/**
 * Constructor, is private since the class is purely static.
 */
InterruptManager::InterruptManager()
{
}//eof

/**
 * Destructor, is private.
 */
InterruptManager::~InterruptManager()
{
}//eof

/**
 * \brief Inserts a given function pointer into the interrupt vector table.
 * \param[in] interruptHandler - Function pointer to the interrupt handler method/function.
 * \param[in] interruptID - The interrupt ID specifies to which interrupt the given function
 * is supposed to be linked; IDs are provided as static members of the InterruptManager class.
 *
 * FIXME Assumes that the vector table is located at the very beginning of SRAM;
 * should check the SCB->VTOR register first to check whether the vector table
 * is in flash or RAM and wether there is an offset.
 *
 */
void InterruptManager::registerInterruptHandler(void(*interruptHandler)(), uint32_t interruptID)
{
	// Put address of interrupt handler function into the vector table. The address of
	// the appropriate interrupt vector in the table is given by the interrupt ID:
	uint32_t* ptr = (uint32_t*)interruptID;
	*ptr = (uint32_t)(interruptHandler);
}//eof

/**
 * \brief Default Interrupt handler to make visible where the processor goes when there is an
 * unexpected interrupt.
 */

/**
 * This function catches unimplemented interrupts to ease debugging (setting breakpoints in
 * assembler files does not work for some reason). It is called by the default interrupt Handler(){
 * in the assembler startup code.
 */
extern "C" void ISR_DefaultHandler(){
	while(true){
		int a=0;
	}
}

/**
 * At compile time, the following functions are linked to the interrupt vector table.
 * If your controller ends up in one of this functions, because an unexpected interrupt occurred
 * (unexpected because apparently there is no other handler function/method to catch it) it stays there
 * in an infinite loop, allowing to detect and analyze the problem using the debugger.
 */

// Core Interrupts

extern "C" void   NMI_Handler(){
	while(true){
		int a=0;
	}
}
extern "C" void   HardFault_Handler(){
	while(true){
		int a=0;
	}
}
extern "C" void   MemManage_Handler(){
	while(true){
		int a=0;
	}
}
extern "C" void   BusFault_Handler(){
	while(true){
		int a=0;
	}
}
extern "C" void   UsageFault_Handler(){
	while(true){
		int a=0;
	}
}

extern "C" void   SVC_Handler(){
	while(true){
		int a=0;
	}
}
extern "C" void   DebugMon_Handler(){
	while(true){
		int a=0;
	}
}

extern "C" void   PendSV_Handler(){
	while(true){
		int a=0;
	}
}

extern "C" void   SysTick_Handler(){
	while(true){
		int a=0;
	}
}

// External/Peripheral Interrupts

extern "C" void WWDG_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void PVD_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TAMP_STAMP_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void RTC_WKUP_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void FLASH_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void RCC_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void EXTI0_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void EXTI1_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void EXTI2_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void EXTI3_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void EXTI4_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA1_Stream0_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA1_Stream1_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA1_Stream2_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA1_Stream3_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA1_Stream4_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA1_Stream5_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA1_Stream6_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void ADC_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void CAN1_TX_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void CAN1_RX0_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void CAN1_RX1_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void CAN1_SCE_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void EXTI9_5_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM1_BRK_TIM9_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM1_UP_TIM10_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM1_TRG_COM_TIM11_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM1_CC_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM2_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM3_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM4_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void I2C1_EV_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void I2C1_ER_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void I2C2_EV_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void I2C2_ER_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void SPI1_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void SPI2_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void USART1_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void USART2_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void USART3_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void EXTI15_10_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void RTC_Alarm_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void OTG_FS_WKUP_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM8_BRK_TIM12_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM8_UP_TIM13_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM8_TRG_COM_TIM14_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM8_CC_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA1_Stream7_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void FSMC_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void SDIO_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM5_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void SPI3_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void UART4_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void UART5_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM6_DAC_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void TIM7_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA2_Stream0_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA2_Stream1_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA2_Stream2_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA2_Stream3_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA2_Stream4_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void ETH_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void ETH_WKUP_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void CAN2_TX_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void CAN2_RX0_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void CAN2_RX1_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void CAN2_SCE_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void OTG_FS_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA2_Stream5_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA2_Stream6_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DMA2_Stream7_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void USART6_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void I2C3_EV_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void I2C3_ER_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void OTG_HS_EP1_OUT_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void OTG_HS_EP1_IN_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void OTG_HS_WKUP_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void OTG_HS_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void DCMI_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void CRYP_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void HASH_RNG_IRQHandler(){
	while(true){
		int a=0;
	}
}
extern "C" void FPU_IRQHandler(){
	while(true){
		int a=0;
	}
}
