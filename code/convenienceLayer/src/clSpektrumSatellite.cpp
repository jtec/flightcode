/**
 *\file deviceDriver_clSpektrumSatellite.cpp
 *\brief This device driver provides functions to get serial data from a Spektrum satellite / remote receiver.
 *		 	It uses a USART and waits for incoming data bytes from the receiver (frames arrive every 22 ms)
 *		 	by providing an interrupt handler function for the USART 'received data ready to be read' interrupt
 *		 	(actually it is a USART global interrupt but the driver only enables this interrupt source).
 *			Have a look at the interrupt handler to see how the driver detects the beginning of a data
 *			frame. The channel values (0-6) are calculated by the calculate() method.
 *			To use this driver proceed as following:
 *			1) create an instance of clSpektrumSatellite, passing the USART peripheral pointer
 *				(possible USARTs: see constructor) defined in the STM32 StandardPeripheralDriver library by ST  to the constructor.
 *				Once the clSpektrumSatellite has been instantiated, it automatically reads data frames from the remote receiver
 *				connected to this USART's Rx pin.
 *			2) The current channel values may now be accessed by using the getRcChannels([...]) methods.
 *\author Jan
 *
 * dependencies: - Uses a TimeBase to get the current system time.
 * 				 - uses the clAssistant class
 * 				 - Uses parts of the ST STM32 StandardPeripheralDriver firmware library.
*/

//Inclusions
#include "../inc/clSpektrumSatellite.h"
#include "../inc/clAssistant.h"
#include "../inc/clInterruptManager.h"
#include "../inc/clTimebase.h"

//Static fields
// TODO Initialize to some safe value? Because if it's e.g. 0, calling a function
// of such an uninitialized pointer would cause a crash.
clSpektrumSatellite* clSpektrumSatellite::satelliteUsingUSART1;
clSpektrumSatellite* clSpektrumSatellite::satelliteUsingUSART2;
clSpektrumSatellite* clSpektrumSatellite::satelliteUsingUSART3;

/**
 * Constructor.
 * \param[in] usartToUse - pointer to the USART peripheral the driver is supposed to use (e.g. USART2), as defined by the ST Standard Peripheral Library.
 * \param[in] rxPort - the GPIO port the spektrum receivers TX pin is connected to as defined by the ST Standard Peripheral Library.
 * \param[in] rxPort - the GPIO pin the spektrum receivers TX pin is connected to as defined by the ST Standard Peripheral Library.
 *
 */
clSpektrumSatellite::clSpektrumSatellite(USART_TypeDef* usartToUse, GPIO_TypeDef* rxPort, uint16_t rxPin)
{
	this->timeOfFirstByteStored = false;
	this->timeOfLastFrame = 0;
	this->numberOfFramebytesReceived = 0;
	// use the given USART peripheral
	this->usart = usartToUse;
	this->RXPin_GPIOPort = rxPort;
	this->RXPin = rxPin;

	// Different USART use different interrupts:
	if(usartToUse == USART1){
		clSpektrumSatellite::satelliteUsingUSART1 = this;	// Used by interrupt handlers
		this->USART_global_interrupt = USART1_IRQn;
		// register the USART1 interrupt handler
		InterruptManager::registerInterruptHandler(&USART1_IRQHandler_cute, InterruptManager::USART1_Handler);
	}
	else if(usartToUse == USART2){
		clSpektrumSatellite::satelliteUsingUSART2 = this;
		this->USART_global_interrupt = USART2_IRQn;
		// register the USART1 interrupt handler
		InterruptManager::registerInterruptHandler(&USART2_IRQHandler_cute, InterruptManager::USART2_Handler);
	}
	else if(usartToUse == USART3){
		clSpektrumSatellite::satelliteUsingUSART3 = this;
		this->USART_global_interrupt = USART3_IRQn;
		// register the USART3 interrupt handler
		InterruptManager::registerInterruptHandler(&USART3_IRQHandler_cute, InterruptManager::USART3_Handler);
	}

	// write the USART's properties in a USART init structure
	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	// TODO Any advantage by using only USART_Mode_Rx ?

	this->setUp(USART_InitStructure);
}//eof

/**
 * Destructor
 */
clSpektrumSatellite::~clSpektrumSatellite()
{
	// TODO Free memory?
}//eof

/**
 * \brief Internal method that configures GPIO pins and the USART.
 */
void clSpektrumSatellite::setUp(USART_InitTypeDef usartInitStruct)
{
	//initialize USART input buffer
	for(uint8_t i=0; i<clSpektrumSatellite::bufferSize; i++){
		// set every byte to a particular value to make recognizing errors easier.
		this->inBuffer[i] = 77;
	}
	//initialize RC channel buffer
	for(uint8_t i=0; i<clSpektrumSatellite::numberOfChannels; i++){
		// set every byte to a a particular value to make recognizing errors more easier.
		this->channels[i] = 88;
	}

	//set up GPIO pins
	clAssistant::enableGPIOClock(this->RXPin_GPIOPort);
	GPIO_InitTypeDef GPIO_InitStructure;
	// Configure USART RX pin as alternate function pin
	GPIO_InitStructure.GPIO_Pin = this->RXPin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(this->RXPin_GPIOPort, &GPIO_InitStructure);

	//set up USART
	clAssistant::enableUSARTClock((uint32_t)this->usart);


	USART_DeInit(this->usart);
	USART_Init(this->usart, &usartInitStruct);

	USART_ITConfig(this->usart, USART_IT_RXNE, ENABLE);						//enable USART's 'Rx register not empty' interrupt
	USART_Cmd(this->usart, ENABLE);

	//set up NVIC
	NVIC_InitTypeDef NVIC_InitStructure;
	// Enable the USART's 'Received data ready to be read' interrupt / USART global interrupt
	NVIC_InitStructure.NVIC_IRQChannel = this->USART_global_interrupt;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	return;
}//eof

/**
 * The following wrapper functions are necessary because object methods cannot be used as
 * interrupt handler functions.
 */

/**
 * Interrupt handler wrapper for USART3.
 */
extern "C" void USART3_IRQHandler_cute()
{
	clSpektrumSatellite::satelliteUsingUSART3->usart_IRQHandler();
	return;
}//eof
/**
 * Interrupt handler wrapper for USART2.
 */
extern "C" void USART2_IRQHandler_cute()
{
	clSpektrumSatellite::satelliteUsingUSART2->usart_IRQHandler();
	return;
}//eof
/**
 * Interrupt handler wrapper for USART1.
 */
extern "C" void USART1_IRQHandler_cute()
{
	clSpektrumSatellite::satelliteUsingUSART1->usart_IRQHandler();
	return;
}//eof

/**
 * \brief This handler function is called every time the USART receives one byte from the
 * Spektrum remote receiver.
 */
void clSpektrumSatellite::usart_IRQHandler(){
	// Check the interrupt's source.
	// a) Is it a 'Received data ready to be read' interrupt?
	if((this->usart->SR && BIT5) > 0){
			uint8_t byte = this->usart->DR;
			// This is only done once receiving the very first byte.
			if(this->timeOfFirstByteStored == false)
			{
				this->timeOfLastFrame = TimeBase::getSystemTimeMs();
				this->timeOfFirstByteStored = true;
			}
			// A) If we are expecting the first byte of a frame: Check the time having passed since the last byte
			// has been received. If it is greater than 10 ms we have just received the first
			// byte of a Spektrum remote receiver data frame.
			if(this->numberOfFramebytesReceived == 0)
			{
				// If the byte is too close to the end of the last frame to be the first byte of a frame:
				if( (TimeBase::getSystemTimeMs() - this->timeOfLastFrame) < 10 )
				{
					return;
				}
				// Otherwise:
				else{
					this->inBuffer[this->numberOfFramebytesReceived] = byte;
					this->numberOfFramebytesReceived++;
				}
			}
			//B) If we're receiving one of the following bytes:
			else
			{
				// Store this byte to buffer
				this->inBuffer[this->numberOfFramebytesReceived] = byte;

				this->numberOfFramebytesReceived++;
				// C) At the end of a frame:
				if(this->numberOfFramebytesReceived >= 16){
					this->timeOfLastFrame = TimeBase::getSystemTimeMs();
					this->timeOfLastValidFrame = this->timeOfLastFrame;
					this->numberOfFramebytesReceived = 0;
					// calculate channel values
					this->calculate();
				}
			}
	}

	return;
}//eof

/**
 * \brief Calculates the RC channel values using the last received frame.
 * Each channel is encoded as follows:
 * first byte:  R1 R0 C3 C2 C1 C0 D9 D8
 * second byte: D7 D6 D5 D4 D3 D2 D1 D0
 * D: channel value bits
 * C: channel number bits
 * R: not used
 */
void clSpektrumSatellite::calculate()
{
	uint8_t firstByte 	= 0;
	uint8_t secondByte 	= 0;
	uint8_t channel		= 0;
	uint16_t value 		= 0;
	for(uint8_t i=0; i<clSpektrumSatellite::numberOfChannels; i++)
	{
		// channels start at byte index 2 (third byte)
		firstByte 	= this->inBuffer[(i+1)*2];		// byte 2, 4, 6, 8, 10, 12, 14
		secondByte 	= this->inBuffer[(i+1)*2+1];	// byte 3, 5, 7, 9, 11, 13, 15
		channel = (firstByte & 0b00111100) >> 2;
		value = (firstByte & 0b00000011);
		value = value <<8;
		value = value | secondByte;
		// Check if RC value and channel in range of reasonable values (7 channels, 10 bit/channel).
		if(value > 1025 || channel > 7)
		{
			return;
		}
		this->channels[channel] = (((int16_t)value) - 512)*2;			// FIXME channel interval: [-1000, 1000]
//		this->channels[channel] = value;					//FIXME hack
	}
}//eof

/**
 * \brief Writes the channel values [-1000, 1000] to the given integer array.
 * \param[out] target - pointer to an array of at least 7 16-bit signed integers
 */
void clSpektrumSatellite::getRcChannels(int16_t* target)
{
	for(uint8_t i=0; i<clSpektrumSatellite::numberOfChannels; i++)
	{
		target[i] = this->channels[i];
	}
}//eof

/**
 * \brief Writes the channel values [-1.0, 1.0] to the given integer array.
 * \param[out] target - pointer to an array of at least 7 floats.
 */
void clSpektrumSatellite::getRcChannels(float* target)
{
	for(uint8_t i=0; i<clSpektrumSatellite::numberOfChannels; i++)
	{
		target[i] = ((float)this->channels[i])/1000.0;
	}
}//eof



/**
 * \brief Writes the raw data frame received from the Spektrum remote receiver to the given array of 16 bytes.
 */
void clSpektrumSatellite::getRawData(uint8_t* target)
{
	static uint16_t frameLength = 16;
	for(uint8_t i=0; i<frameLength; i++)
	{
		target[i] = this->inBuffer[i];
	}
}//eof

/**
 * \brief Returns the timestamp (system time in ms) of the last valid frame. May be used to detect
 * if the receiver is still receiving fresh data or maybe out of range.
 */
uint32_t clSpektrumSatellite::getTimeOfLastValidFrame()
{
	return this->timeOfLastValidFrame;
}
