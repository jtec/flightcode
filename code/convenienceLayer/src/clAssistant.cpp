 /*************************************
 * \file clAssistent.cpp
 * \brief This class provides service functions to facilitate recurring simple but annoying tasks
 * like switching on a peripheral clock.
 * author: jan
 * 
 **************************************/

 /*Includes*********************************/
#include "../inc/clAssistant.h"
#include "../inc/clTimebase.h"

//Define static fields
uint8_t 	clAssistant::CoreCycleCounterLocked = 0;
uint32_t 	clAssistant::cyclesPerMicrosecond 	= 0;
uint32_t 	clAssistant::nanosecondsPerAction 	= 0;
uint32_t* 	clAssistant::DWT_CYCCNT 			= (uint32_t *)0xE0001004;
uint32_t* 	clAssistant::DWT_CONTROL 			= (uint32_t *)0xE0001000;
uint32_t* 	clAssistant::SCB_DEMCR 				= (uint32_t *)0xE000EDFC;

void clAssistant::Init()
{
	float nPA = 0.45*((float)SystemCoreClock/100000.0F);		//TODO measure execution time
	clAssistant::nanosecondsPerAction = (uint32_t)nPA;
	clAssistant::DWT_CYCCNT 	= (uint32_t*)0xE0001004;
	clAssistant::DWT_CONTROL 	= (uint32_t*)0xE0001000;
	clAssistant::SCB_DEMCR 		= (uint32_t*)0xE000EDFC;

	clAssistant::CoreCycleCounterLocked = 0;
	clAssistant::cyclesPerMicrosecond = SystemCoreClock/1000000;
}//eof

clAssistant::clAssistant()
{
}//eof

uint8_t clAssistant::enableGPIOClock(GPIO_TypeDef* gpio){
	uint32_t peripheralAddress = (uint32_t)gpio;
	if (peripheralAddress==(uint32_t)GPIOA){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)GPIOB){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)GPIOC){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)GPIOD){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)GPIOE){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)GPIOF){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)GPIOG){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)GPIOH){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)GPIOI){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
		return 0;
	}
		return 1;
}//eof

uint8_t clAssistant::enableDMAClock(DMA_TypeDef* dma){
	if ((uint32_t)dma == (uint32_t)DMA1){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		return 0;
	}else if ((uint32_t)dma == (uint32_t)DMA2){
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		return 0;
	}
		return 1;
}//eof

void clAssistant::enableI2CClock(I2C_TypeDef* i2cx){
	if (i2cx == I2C1){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	}else if (i2cx == I2C2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	}else if (i2cx == I2C3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
	}
}//eof


uint8_t clAssistant::enableUSARTClock(uint32_t peripheralAddress){
	if (peripheralAddress==(uint32_t)USART1){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)USART6){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)USART2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)USART3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)UART4){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		return 0;
	}else if (peripheralAddress==(uint32_t)UART5){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		return 0;
	}
		return 1;
}//eof

/**
 *
 * \brief Enables the clock of a TIMER peripheral.
 * \param timer - identifier of the TIMERx supposed to be enabled.
 */
void clAssistant::enableTIMERClock(TIM_TypeDef* timer){

	// Timers connected to APB1:
	if(timer==TIM1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	}
	else if(timer==TIM8){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	}
	else if(timer==TIM9){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	}
	else if(timer==TIM10){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	}
	else if(timer==TIM11){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	}
	//timers connected to APB2
	else if(timer==TIM2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	}
	else if(timer==TIM3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	}
	else if(timer==TIM4){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	}
	else if(timer==TIM5){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	}
	else if(timer==TIM6){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	}
	else if(timer==TIM7){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	}
	else if(timer==TIM12){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	}
	else if(timer==TIM13){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	}
	else if(timer==TIM14){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	}

	return;
}//eof

/**
 * Creates a delay by decrementing a variable x times.
 */
void clAssistant::waitCycles(uint32_t t)
{
	while(t>1){t--;}				//create a delay by decrementing a (a-1) times
}//eof

/**
 * \brief returns the length of a string by finding the '\0' termination symbol.
 * E.g. TODO add example
 */
uint16_t clAssistant::getStringLength(char *string)		//returns the length of a string
{
	u16 length=0;
	while(string[length]!='\0')
	{
		length++;
	}
return length;
}//eof

/**
 * \brief Reverses the Order of bits in a byte.
 * e.g.
 * \param char bits - the byte having the original bit order.
 * \return the reversed byte
 * TODO Test.
 */
char clAssistant::reverseBitOrder(char bits){
	static char BitReverseTable[256] =
	{
	0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
	0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
	0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
	0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
	0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
	0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
	0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
	0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
	0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
	0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
	0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
	0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
	0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
	0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
	0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
	0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
	0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
	0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
	0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
	0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
	0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
	0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
	0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
	0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
	0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
	0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
	0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
	0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
	0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
	0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
	0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
	0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff
	};
	return BitReverseTable[(uint8_t)bits];
}//eof

/**
 * \brief returns 1 if the given bit in the given 32-Bit register equals 1.
 * returns 0 if the given bit in the given register equals 0
 * @tested function tested, works as expected
 */
uint8_t clAssistant::isBitSet(uint32_t registrum, uint8_t bitnumber){
		uint32_t mask = 1;
		mask = mask << bitnumber;
		if( (registrum & mask) > 0){
			return 1;
		}
		else{
			return 0;
		}
}//eof

/**
 * \brief sets the bit of the specified number[0,31] in the given 32-Bit register to 1.
 * \param uint32_t* pointer to the register
 * \param uint8_t index of the bit to be set
 * TODO this function should be used only in e.g. initialization routines since it
 * needs much more cycles than setting a bit the direct way ("GPIOx->BSRR/GPIOx->BRR |= BITx")
 */
void clAssistant::setBit(uint32_t* registerAddress , uint8_t bitnumber){
	uint32_t mask = 1;
	mask = mask << bitnumber;
	*registerAddress |= mask;
}//eof

/**
 * \brief sets the bit of the specified number[0,31] in the given 32-Bit register to 0.
 * \param uint32_t* pointer to the register
 * \param uint8_t number of the bit to be cleared
 * TODO this function should be used only in e.g. initialization routines since it
 * needs much more cycles than clearing a bit in a direct way ("register &= ~BITx")
 */
void clAssistant::clearBit(uint32_t* registerAddress, uint8_t bitnumber){
	uint32_t mask = 1;
	mask = mask << bitnumber;
	*registerAddress &= ~mask;
}//eof

/**
 * \brief Starts the STM32's clock cycle counter beginning from 0.
 */
void clAssistant::startCycleCounter(){
if(clAssistant::CoreCycleCounterLocked == 0){
	*(clAssistant::SCB_DEMCR) |= 0x01000000;
	*(clAssistant::DWT_CYCCNT) = 0;						 	// reset the counter
	*(clAssistant::DWT_CONTROL) |= 1; 			// enable the counter
	clAssistant::CoreCycleCounterLocked = 1;
	*(clAssistant::DWT_CYCCNT) = 0; 							// reset the counter
//cycles are counted beginning from here
}
else{
	;
}
return;
}

/**
 * \brief Returns the current value of the STM32's cycle counter .
 */
uint32_t clAssistant::getCycleCounterValue(){
	clAssistant::CoreCycleCounterLocked = 0;
	//TODO adjust the cycle counter value by the number of cycles it takes to enter and to leave this function and to execute the line above
	return (*(clAssistant::DWT_CYCCNT) - 2);
}//eof

/**
* \brief Converts an array of PWM values [1000, 2000] into float values [-1.0, 1.0].
* \param[in] pointer to an array of pwm values.
* \param[out] pointer to an float array of the same length the results of the conversion are written to.
* \param[in] length of both arrays.
*/
void clAssistant::pwm2float(uint16_t* pwm, float* flt, uint8_t length){
  for(uint8_t i=0; i<length; i++){
  		flt[i] =  ( (float)(pwm[i]-1500) ) / 500;
  	}
  return;
}

/**
* \brief Converts an array of float values [-1.0, 1.0] into PWM values [1000, 2000].
* \param[in] pointer to an array of float values.
* \param[out] pointer to an array of the same length the results of the conversion are written to.
* \param[in] length of both arrays.
*
*/
void clAssistant::float2pwm(float* flt, uint16_t* pwm, uint8_t length){
	for(uint8_t i=0; i<length; i++){
		pwm[i] = 1500 + flt[i] * 500;
	}
	return;
}
