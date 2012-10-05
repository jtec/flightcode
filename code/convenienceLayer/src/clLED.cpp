/*
 * \file clLED.cpp
 * \brief An instance of this class can be used to control an LED
 * connected to a GPIO pin.
 * \author jan
 */

#include "../inc/clLED.h"

/**
 * \brief Constructor, instantiates and initializes an LED.
 */
clLED::clLED(GPIO_TypeDef* port, uint16_t pin, bool initialStatus)
: clGPIOPin(port, pin, initialStatus)	// Call base class constructor with the same arguments.
{

}
