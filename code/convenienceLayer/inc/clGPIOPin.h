/*
 *
 * \file clGPIOPin.h
 * \brief This driver provides functions to control a GPIO pin.
 * \author: jan bolting
 */
#ifndef CLGPIOPIN_H_
#define CLGPIOPIN_H_

#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"
#include "clAssistant.h"

class clGPIOPin
{
public:
	clGPIOPin(GPIO_TypeDef* port, uint16_t pin, bool initialStatus);							//constructor
	~clGPIOPin(void);									//destructor
	void toggle();
	void set(bool newState);
	static const bool Pinstate_HIGH = true;
	static const bool Pinstate_LOW = false;
	clGPIOPin();
protected:
	bool status;					// Indicates if the Pin currently is high or low.
	GPIO_TypeDef* GPIOPort;				// GPIO port.
	uint16_t GPIOPin;					// GPIO pin.
};//eoc




#endif /* CLGPIOPIN_H_ */
