/*************************************
 * \file clGPIOPin.cpp
 * \brief This driver provides functions to control the state of a GPIO output pin.
 * \author: jan bolting
 *
 **************************************/

#include "../inc/clGPIOPin.h"

/**
 * \brief Constructor, instantiates and initializes a GPIO pin.
 */
clGPIOPin::clGPIOPin(GPIO_TypeDef* port, uint16_t pin, bool initialStatus){

	this->GPIOPort = port;
	this->GPIOPin = pin;

	// Enable the GPIO port Clock
	clAssistant::enableGPIOClock(port);

	// Configure the pin:
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(this->GPIOPort, &GPIO_InitStructure);

	this->set(initialStatus);
}//eof

/**
 * \brief Destructor.
 */
clGPIOPin::~clGPIOPin(){
	// TODO free memory
	return;
}//eof


/**
 * \brief Toggles the pin state.
 */
void clGPIOPin::toggle(){
	this->set(!this->status);
	return;
}//eof

/**
 * \brief Sets the pin's state: high or low.
 * \param[in] state - the desired new state of the pin (states defined in the header file)
 */
void clGPIOPin::set(bool state)
{
	if (state==clGPIOPin::Pinstate_HIGH)
	{
		this->GPIOPort->BSRRL |= this->GPIOPin;
		this->status = clGPIOPin::Pinstate_HIGH;
	}
	else
	{
		this->GPIOPort->BSRRH |= this->GPIOPin;
		this->status = clGPIOPin::Pinstate_LOW;
	}
	return;
}//eof
