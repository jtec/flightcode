/*************************************
 * \file serialPort.cpp
 * \brief 	This file provides functions to send and receive data by serial ports (physical device: USART).
 *	 		A serialPort is kind of a USART port with extended capabilities.
 *
 * \author: jan bolting
 *
 **************************************/

#include "../inc/clSerialPort.h"
#include "../../hardwareAccessLayer/stm32_bits.h"

// Initialize static members.
// FIXME: Pointers shouldn't be initialized to zero.
clSerialPort* clSerialPort::serialPortUsingUSART1 = 0;
clSerialPort* clSerialPort::serialPortUsingUSART2 = 0;
clSerialPort* clSerialPort::serialPortUsingUSART3 = 0;

/**
 * \brief Constructor. Initializes a serial I/O port (USART).
 * \param[in] usartToUse - pointer to the USART peripheral (USART1, USART2 etc) the SerialPort is supposed
 * to use.
 * \param[in] baudrate - the desired baudrate [Bit/s], e.g. 115200 or 2500000.
 * \param[in] gpio_tx - the GPIO port the USART's Tx pin is part of, e.g. GPIOB.
 * \param[in] gpio_rx - the GPIO port the USART's Rx pin is part of.
 * \param[in] pin_tx - the USARt Tx pin, e.g. GPIO_Pin_12.
 * \param[in] pin_rx - the USARt Rx pin.
 *
 */
clSerialPort::clSerialPort(USART_TypeDef* usartToUse, uint32_t baudrate, GPIO_TypeDef* gpio_tx, GPIO_TypeDef* gpio_rx, uint16_t pin_tx, uint16_t pin_rx){

	// Initialize variables:
	this->usartBusyTransmitting = false;
	this->DMABusyTransmitting = false;

	//set up GPIO pins
	clAssistant::enableGPIOClock( gpio_rx );
	clAssistant::enableGPIOClock( gpio_tx);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure USART TX pin as alternate function push-pull:
	GPIO_InitStructure.GPIO_Pin = pin_tx;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(gpio_tx, &GPIO_InitStructure);

	// Configure USART RX pin as input:
	GPIO_InitStructure.GPIO_Pin = pin_rx;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF;
	GPIO_Init(gpio_rx, &GPIO_InitStructure);

	// Initialize Rx and Tx buffers:
	for(uint16_t i=0; i<this->rxBuffer.buffersize; i++){
		this->rxBuffer.dataBuffer[i] = 0;
	}
	//The buffer read position is the buffer index we will read the next byte from.
	this->rxBuffer.bufferReadPosition = 0;
	for(uint16_t i=0; i<this->txBuffer.buffersize; i++){
		this->txBuffer.dataBuffer[i] = 0;
	}
	this->txBuffer.nextWritePosition = 0;
	this->txBuffer.overflow = false;
	this->txBuffer.dmaJumpBack = false;


	if(usartToUse == USART1){
		clSerialPort::serialPortUsingUSART1 = this;
		this->hardware = usartToUse;							//uses #usartToUse as physical device to send and receive data
		// Define to which DMA, DMA streams and DMA channels the USART is connected.
		this->dma_rx = DMA2;
		this->dmaStream_rx = DMA1_Stream5;
		this->dmaChanel_rx = DMA_Channel_4;
		this->dma_tx = DMA2;
		this->dmaStream_tx = DMA2_Stream7;
		this->dmaChanel_tx = DMA_Channel_4;
		this->dmaTCInterruptID_tx = DMA_IT_TCIF7;
		this->DMAStreamTx_interruptNVICID = DMA2_Stream7_IRQn;
		// Register DMA 'Transmission Complete' interrupt handler.
		InterruptManager::registerInterruptHandler(&USART1_DMA_TransmissionCompleteHandlerTx, InterruptManager::DMA2_Stream7_Handler);
	}
	else if(usartToUse == USART2){
		clSerialPort::serialPortUsingUSART2 = this;
		this->hardware = USART2;
		// Define to which DMA, DMA streams and DMA channels the USART is connected.
		this->dma_rx = DMA1;
		this->dmaStream_rx = DMA1_Stream5;
		this->dmaChanel_rx = DMA_Channel_4;
		this->dma_tx = DMA1;
		this->dmaStream_tx = DMA1_Stream6;
		this->dmaChanel_tx = DMA_Channel_4;
		this->dmaTCInterruptID_tx = DMA_IT_TCIF6;
		this->DMAStreamTx_interruptNVICID = DMA1_Stream6_IRQn;
		// Register DMA 'Transmission Complete' interrupt handler.
		InterruptManager::registerInterruptHandler(&USART2_DMA_TransmissionCompleteHandlerTx, InterruptManager::DMA1_Stream6_Handler);
	}
	else if(usartToUse == USART3){
		clSerialPort::serialPortUsingUSART3 = this;
		this->hardware = USART3;
		// Define to which DMA, DMA streams and DMA channels the USART is connected.
		this->dma_rx = DMA1;
		this->dmaStream_rx = DMA1_Stream1;
		this->dmaChanel_rx = DMA_Channel_4;
		this->dma_tx = DMA1;
		this->dmaStream_tx = DMA1_Stream3;
		this->dmaChanel_tx = DMA_Channel_4;
		this->dmaTCInterruptID_tx = DMA_IT_TCIF3;
		this->DMAStreamTx_interruptNVICID = DMA1_Stream3_IRQn;
		// Register DMA 'Transmission Complete' interrupt handler.
		InterruptManager::registerInterruptHandler(&USART3_DMA_TransmissionCompleteHandlerTx, InterruptManager::DMA1_Stream3_Handler);
	}

	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;							//use 8-Bit words
	USART_InitStructure.USART_StopBits = USART_StopBits_1;								//use 1 stopbit
	USART_InitStructure.USART_Parity = USART_Parity_No;									//don't use parity
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//don't use hardware flow control
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;						//use bidirectional mode

	this->setUpHardware(USART_InitStructure);
	return;
}//eof

/**
 * \brief Destructor.
 */
clSerialPort::~clSerialPort(){
	// FIXME free memory
}//eof

/**
 * \brief Internal method that configures the USART, DMA and GPIO pins.
 */
void clSerialPort::setUpHardware(USART_InitTypeDef usartInitStruct){

	//Set up DMA for data reception. //
	//Enable the DMA clock.
	clAssistant::enableDMAClock(this->dma_rx);
	DMA_InitTypeDef  DMA_InitStructure;
	DMA_DeInit(this->dmaStream_rx);
	// Configure DMA controller to manage RX DMA request //
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(this->hardware->DR);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_InitStructure.DMA_Channel = this->dmaChanel_rx;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&(this->rxBuffer.dataBuffer[0]);
	DMA_InitStructure.DMA_BufferSize = (uint16_t)this->rxBuffer.buffersize;

	DMA_Init(this->dmaStream_rx, &DMA_InitStructure);
	// Enable the DMA RX Stream //
	DMA_Cmd(this->dmaStream_rx, ENABLE);

	// Set up DMA Stream for data transmission //
	//Enable the DMA clock.
	clAssistant::enableDMAClock(this->dma_tx);
	DMA_DeInit(this->dmaStream_tx);

	DMA_StructInit(&DMA_InitStructure);
	// Configure DMA controller to manage TX DMA request
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(this->hardware->DR);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_Channel = this->dmaChanel_tx;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;

	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&(this->txBuffer.dataBuffer[0]);
	DMA_InitStructure.DMA_BufferSize = (uint16_t)0;	// Since we have got no bytes to transmit yet.
	DMA_Init(this->dmaStream_tx, &DMA_InitStructure);

	// Enable Tx DMA stream 'Transmission Complete' interrupt.
	DMA_ITConfig(this->dmaStream_tx, DMA_IT_TC, ENABLE);

	// Disable the DMA Tx Stream //
	DMA_Cmd(this->dmaStream_tx, DISABLE);

	// Set up USART. //
	clAssistant::enableUSARTClock((uint32_t)this->hardware);
	// Configure USART.
	USART_Init(this->hardware, &usartInitStruct);

	// Enable the USART Tx DMA request //
	USART_DMACmd(this->hardware, USART_DMAReq_Tx, ENABLE);
	// Enable the USART Rx DMA request.
	USART_DMACmd(this->hardware, USART_DMAReq_Rx, ENABLE);


	// Disable the USART OverSampling by 8 to try 10 MBit.
	//USART_OverSampling8Cmd(this->hardware, ENABLE);

	// Set up NVIC. //
	// For DMA based data transmission.
	NVIC_InitTypeDef NVIC_InitStructure;
	// Enable the USART's 'Received data ready to be read' interrupt / USART global interrupt:
	NVIC_InitStructure.NVIC_IRQChannel = this->DMAStreamTx_interruptNVICID;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Switch on USART.
	USART_Cmd(this->hardware, ENABLE);

	return;
}//eof

/**
 * \brief Indicates whether there are unread bytes in the input buffer.
 * \return boolean indicating whether there are in the input buffer that have not been read yet.
 */
bool clSerialPort::newBytesAvailable(){
	//We are assuming that there are only new bytes available in case both the "dmaIndex" i.e. the next write
	//position the dma goes to and the "bufferReadPosition" point to the same byte.
	// Calculate buffer index up to which the DMA has written bytes into the buffer.
	uint16_t dmaCounter = DMA_GetCurrDataCounter(this->dmaStream_rx);
	uint16_t dmaIndex = this->rxBuffer.buffersize - dmaCounter;
	if(dmaIndex == this->rxBuffer.bufferReadPosition){
		return false;
	}
	return true;
}

/**
 * \brief Returns one byte of the rx buffer.
 * \return The next byte from the input buffer.
 */
uint8_t clSerialPort::readByte(){

	uint8_t returnValue = this->rxBuffer.dataBuffer[this->rxBuffer.bufferReadPosition];
	this->rxBuffer.bufferReadPosition += 1;
	if(this->rxBuffer.bufferReadPosition > ( this->rxBuffer.buffersize - 1)){
		this->rxBuffer.bufferReadPosition = 0;
	}
	return returnValue;
}

/**
 * \brief Sends a text message, sometime suseful to see if the serial connection wroks properly since
 * errors in written text are easy to spot.
 */
void clSerialPort::sendTestMessage()
{
	char testMessage[42] = "sending serialPort.c module Teststring\n\r";
	this->writeArray((uint8_t*)testMessage, 42);
	return;
}//eof

/**
 * Writes an array of bytes to the output buffer and sets up the DMA to transmit them.
 * \param[in] array - the data array to be sent.
 * \param[in] length - the length of the array in bytes (max length: length of the tx buffer, defined in the header file)
 */
void clSerialPort::writeArray(uint8_t* array, uint16_t length){

	// Stop DMA.
	DMA_Cmd(this->dmaStream_tx, DISABLE);
	// The DMA possibly is still busy transmitting a byte, so wait for it to finish the last transaction.
	bool isENBitSet = ((this->dmaStream_tx->CR & BIT0) > 0);
	while(isENBitSet){
		// FIXME Add timeout.
		isENBitSet = ((this->dmaStream_tx->CR & BIT0) > 0);
	}
	// Get the number of remaining bytes to transfer by the DMA:
	uint16_t numberOfBytesNotTransmittedYet = DMA_GetCurrDataCounter(this->dmaStream_tx);
	// Store tx buffer index where the DMA is currently reading from:
	uint16_t currentDMAIndex = this->txBuffer.nextWritePosition - numberOfBytesNotTransmittedYet;
	// Check
	if(this->txBuffer.nextWritePosition > this->txBuffer.buffersize-1 && numberOfBytesNotTransmittedYet == 0 ){
		currentDMAIndex = 0;
	}

	// lengthOfArraySectionThatFitsIntoTheBuffer = number of bytes of those part of the data array that fits into the buffer.
	uint16_t lengthOfArraySectionThatFitsIntoTheBuffer = 0;
	// Calculate the remaining free space in the buffer:
	// E.g. if the next byte will be written to position/index 2 and the buffer size is 4, the remaining space is: 4 - 2  =  2.
	uint16_t remainingSpaceInBytes = this->txBuffer.buffersize - this->txBuffer.nextWritePosition;
	if(remainingSpaceInBytes == 0){
		remainingSpaceInBytes = this->txBuffer.buffersize;
	}
	// Figure out whether there array will exactly fit into the remaining space or if there will be an overflow:
	if(remainingSpaceInBytes == length){
		// The dma will have to return to the beginning of the buffer after having transmitted this array.
		this->txBuffer.dmaJumpBack = true;
	}else if(remainingSpaceInBytes < length){
		// The dma will have to return to the beginning of the buffer after having transmitted this array AND there will be an overflow.
		lengthOfArraySectionThatFitsIntoTheBuffer = remainingSpaceInBytes;
		this->txBuffer.dmaJumpBack = true;
		this->txBuffer.overflow = true;
	}
	// Detect if we had a jumpback situation and at the same time the DMA has still been transmitting data:
	if(numberOfBytesNotTransmittedYet > 0 && this->txBuffer.dmaJumpBack){
		this->txBuffer.overflow = true;
	}

	// Write bytes to tx buffer:
	for(uint16_t i=0; i< length; i++){
		// If we are about to write to the byte after the last position in the buffer, return to the first position:
		if( this->txBuffer.nextWritePosition > (this->txBuffer.buffersize-1)){
			this->txBuffer.nextWritePosition = 0;
		}
		this->txBuffer.dataBuffer[this->txBuffer.nextWritePosition] = array[i];
		this->txBuffer.nextWritePosition += 1;
	}
	// Add the new bytes to the data counter.
	// a) If we wrote just a fraction of the array to the upper part of the buffer, then hit
	// the buffer limit and returned to the beginning of the buffer:
	if(this->txBuffer.overflow){
		DMA_SetCurrDataCounter(this->dmaStream_tx, numberOfBytesNotTransmittedYet + lengthOfArraySectionThatFitsIntoTheBuffer);
	}
	// b) If we were able to write 100% of the array to the buffer without an overflow:
	else{
		DMA_SetCurrDataCounter(this->dmaStream_tx, numberOfBytesNotTransmittedYet + length);
	}
	// Since the DMA jumps back to the initial source address (here: the first byte of the tx buffer) when disabled and then enabled,
	// as a workaround the source address is updated manually.
	// TODO Figure out if this is a device limitation (not in the Errata sheet) or if we are doing anything wrong configuring or using the DMA.
	uint32_t newTarget = (uint32_t)&(this->txBuffer.dataBuffer[currentDMAIndex]);
	DMA_MemoryTargetConfig(this->dmaStream_tx, newTarget, DMA_Memory_0);
	// Reenable DMA stream;
	// NOTE: re- - enabling the stream causes a Transfer Complete interrupt according to the reference manual, RM0090.
	DMA_Cmd(this->dmaStream_tx, ENABLE);
	this->DMABusyTransmitting = true;

	return;
}//eof

/**
 * \brief Interrupt handler for the tx DMA 'Transfer Complete' interrupt.
 */
void clSerialPort::handleDMAInterruptTx(){
	// Figure out whether this interrupt has been caused by a 'Transfer Complete' interrupt.
	bool isTCInterrupt = DMA_GetITStatus(this->dmaStream_tx, this->dmaTCInterruptID_tx);
	if(isTCInterrupt){
		this->DMABusyTransmitting = false;
		// Clear interrupt flag.
		DMA_ClearITPendingBit(this->dmaStream_tx, this->dmaTCInterruptID_tx);
		// Check if when the last bunch of data have been written to the buffer, we hit the buffer limit and thus
		// have to return to the beginning of the buffer. Since stopping and re-enabling the DMA causes an immediate
		// TC interrupt (which we are not interested in), check if it is an interrupt caused by all bytes having been transmitted,
		// i.e. a "real" Transfer Complete interrupt.
		if(this->txBuffer.dmaJumpBack && this->dmaStream_tx->NDTR == 0){
			// Disable the DMA stream for reconfiguration:
			DMA_Cmd(this->dmaStream_tx, DISABLE);
			// Reset jump back flag since we jumped back.
			this->txBuffer.dmaJumpBack = false;
			// Update the DMA's address to read from.
			uint32_t newTarget = (uint32_t)&(this->txBuffer.dataBuffer[0]);
			// Use 'DMA_Memory_0' since be are not operating in double buffer mode.
			DMA_MemoryTargetConfig(this->dmaStream_tx, newTarget, DMA_Memory_0);
			if(this->txBuffer.overflow){
				this->txBuffer.overflow = false;
				// Update the DMA's data counter to make it transmit the bytes at the beginning of the buffer.
				DMA_SetCurrDataCounter(this->dmaStream_tx, this->txBuffer.nextWritePosition);
				// Reenable the DMA stream to make it do its job.
				DMA_Cmd(this->dmaStream_tx, ENABLE);
				this->DMABusyTransmitting = true;
			}
		}
	}
}

/**
 * \brief Returns true if the port's USART is still transmitting data, false if transmission has finished.
 * \return boolean answer.
 */
bool clSerialPort::isTransmitting(){
	// Check whether the USART is transmitting or not.
	if(USART_GetFlagStatus(this->hardware, USART_FLAG_TC)==RESET){
		this->usartBusyTransmitting = true;
	}else{
		usartBusyTransmitting = false;
	}
	if(this->usartBusyTransmitting || this->DMABusyTransmitting){
		return true;
	}
	else{
		return false;
	}
}

/**
 * The following wrapper functions are necessary because object methods cannot be used as
 * interrupt handler functions.
 */

/**
 * \brief Static wrapper function for DMA serving USART1 'Transmission Complete' interrupt.
 */
extern "C" void USART1_DMA_TransmissionCompleteHandlerTx(){
	// Call interrupt handler method.
	clSerialPort::serialPortUsingUSART1->handleDMAInterruptTx();
}

/**
 * \brief Static wrapper function for DMA serving USART2 'Transmission Complete' interrupt.
 */
extern "C" void USART2_DMA_TransmissionCompleteHandlerTx(){
	// Call interrupt handler method.
	clSerialPort::serialPortUsingUSART2->handleDMAInterruptTx();

}

/**
 * \brief Static wrapper function for DMA serving USART3 'Transmission Complete' interrupt.
 */
extern "C" void USART3_DMA_TransmissionCompleteHandlerTx(){
	// Call interrupt handler method.
	clSerialPort::serialPortUsingUSART3->handleDMAInterruptTx();
}
