 /*************************************
 * \file clAssistant.h
 * \author: jan
 **************************************/

#ifndef CLASSISTANT_H_
#define CLASSISTANT_H_

#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"

 /*Definitions******************************/
class clAssistant
{
public:
	static void Init();
	static void waitCycles(uint32_t t);
	static uint16_t getStringLength(char* string);
	static uint8_t isBitSet(uint32_t registrum, uint8_t bitnumber);
	static void setBit(uint32_t* registerAddress , uint8_t bitnumber);
	static void clearBit(uint32_t* registerAddress , uint8_t bitnumber);
	static uint8_t enableGPIOClock(GPIO_TypeDef* periphAddress);
	static uint8_t enableDMAClock(DMA_TypeDef* periphAddress);
	static uint8_t enableUSARTClock(uint32_t periphAddress);
	static void enableTIMERClock(TIM_TypeDef* timer);
	static void enableI2CClock(I2C_TypeDef* i2cx);
	static char reverseBitOrder(char bits);
	static void startCycleCounter();
	static uint32_t getCycleCounterValue();
	static void float2pwm(float* flt, uint16_t* pwm, uint8_t lenght);
	static void pwm2float(uint16_t* pwm, float* flt, uint8_t lenght);
	//indicates whether the core clock cycle counter has been started and hasn't been stopped yet.
		//used by service_startCycleCounter() and service_getCycleCounterValue() to guarantee that the cycle counter
		//can't be started twice without being evaluated in the meantime since that would cause erroneous values.
	static uint8_t CoreCycleCounterLocked;
	static uint32_t cyclesPerMicrosecond;
	static uint32_t nanosecondsPerAction;
	//CM3 cycle counter register addresses used by service_startCycleCounter() and service_getCycleCounterValue()
	static uint32_t* DWT_CYCCNT;
	static uint32_t* DWT_CONTROL;
	static uint32_t* SCB_DEMCR;
private:
	//constructor is private because there should not exist an instance of this class, since all members are static.
	clAssistant();
	~clAssistant();


};

// BIT DEFINITION: e.g. 0x00000004 = 0b00000000000000000000000000000100

#define   BIT0        0x00000001
#define   BIT1        0x00000002
#define   BIT2        0x00000004
#define   BIT3        0x00000008
#define   BIT4        0x00000010
#define   BIT5        0x00000020
#define   BIT6        0x00000040
#define   BIT7        0x00000080
#define   BIT8        0x00000100
#define   BIT9        0x00000200
#define   BIT10       0x00000400
#define   BIT11       0x00000800
#define   BIT12       0x00001000
#define   BIT13       0x00002000
#define   BIT14       0x00004000
#define   BIT15       0x00008000
#define   BIT16       0x00010000
#define   BIT17       0x00020000
#define   BIT18       0x00040000
#define   BIT19       0x00080000
#define   BIT20       0x00100000
#define   BIT21       0x00200000
#define   BIT22       0x00400000
#define   BIT23       0x00800000
#define   BIT24       0x01000000
#define   BIT25       0x02000000
#define   BIT26       0x04000000
#define   BIT27       0x08000000
#define   BIT28       0x10000000
#define   BIT29       0x20000000
#define   BIT30       0x40000000
#define   BIT31       0x80000000


#endif /* CLASSISTANT_H_ */


