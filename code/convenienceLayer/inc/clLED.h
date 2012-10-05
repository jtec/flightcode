/*
 * \file clLED.h
 * \author jan
 */

#ifndef CLLED_H_
#define CLLED_H_

#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"
#include "clAssistant.h"
#include "clGPIOPin.h"

class clLED : public clGPIOPin{
public:
	clLED(GPIO_TypeDef* port, uint16_t pin, bool initialStatus);							//constructor
private:
	clLED();
};

#endif /* CLLED_H_ */
