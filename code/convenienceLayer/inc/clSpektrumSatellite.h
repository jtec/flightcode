/*
 * \file deviceDriver_clSpektrumSatellite.h
 * \brief This device driver provides functions to receive serial data from
 * 			a Spektrum satellite / remote receiver.
 * \author Jan
 */

#ifndef CLSPEKTRUMSATELLITE_H_
#define CLSPEKTRUMSATELLITE_H_


#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"


class clSpektrumSatellite{
public:
	clSpektrumSatellite(USART_TypeDef* usartToUse, GPIO_TypeDef* rxPort, uint16_t rxPin);
	~clSpektrumSatellite();
	void getRcChannels(int16_t* target);
	void getRcChannels(float* target);
	void getRawData(uint8_t* target);
	uint32_t getTimeOfLastValidFrame();
	void usart_IRQHandler();
	static clSpektrumSatellite* satelliteUsingUSART1;
	static clSpektrumSatellite* satelliteUsingUSART2;
	static clSpektrumSatellite* satelliteUsingUSART3;
	static const uint8_t numberOfChannels = 7;
private:
	void calculate();												// Turns raw data bytes into channel values.
	void setUp(USART_InitTypeDef usartInitStruct);					// This method configures the peripherals used by the driver.
	USART_TypeDef* usart;											// The USART chip peripheral the driver uses for communication.
	GPIO_TypeDef* RXPin_GPIOPort;									//which GPIO port is the USART RX line connected to?
	uint16_t RXPin;													//and to which pin?
	static const uint8_t bufferSize = 18;
	volatile uint8_t inBuffer[clSpektrumSatellite::bufferSize];			// Received bytes are stored in this buffer array.

	volatile int16_t channels[clSpektrumSatellite::numberOfChannels];		// Array holds the current received RC channel values [-1000, +1000]
	volatile uint8_t numberOfFramebytesReceived;							// Indeicates how many bytes of a frame have been received.
	volatile uint32_t timeOfLastValidFrame;								// Indicates when the last valid data frame has been
																		// received from the Spektrum remote receiver
	// Interrupts
	uint8_t USART_global_interrupt;
	// Fields used for frame synchronization
																	// to the receivers data frames.
	volatile uint32_t timeOfLastFrame;								// The time when the last byte of the last frame has been received
	volatile bool timeOfFirstByteStored;							// Indicates whether the reception time of
																	// the last frame has been stored yet.
};

extern "C" void USART3_IRQHandler_cute();
extern "C" void USART2_IRQHandler_cute();
extern "C" void USART1_IRQHandler_cute();


#endif /* CLSPEKTRUMSATELLITE_H_ */
