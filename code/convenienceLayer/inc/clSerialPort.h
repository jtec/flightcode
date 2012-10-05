 /*************************************
 * \file serialPort.h
 * \brief 	This file provides functions to send and receive data by serial ports (USART).
 *	 		A serialPort is kind of a USART port with extended capabilities.
 * 			Until now the parameters of a SerialPort (e.g. 8/9 bit, stopbit, parity etc.) are
 * 			still hard-coded in the constructor.
 *
 * TODO A lot of documentation
 *
 * \author: jan bolting
 *
 **************************************/

#ifndef CLSERIALPORT_H_
#define CLSERIALPORT_H_

#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"
#include "clAssistant.h"
#include "clInterruptManager.h"
#include "clCircularBuffer.h"
#include "clTimebase.h"

struct dmaRxBuffer{
	const static uint16_t buffersize = 500;
	volatile char dataBuffer[buffersize];			// The actual buffer array holding the buffered data. 'Volatile', since it's changed by an interrupt handler..
	volatile uint16_t bufferReadPosition;
};

struct dmaTxBuffer{
	const static uint16_t buffersize = 499;
	volatile char dataBuffer[buffersize];			// The actual buffer array holding the buffered data. 'Volatile', since it's changed by an interrupt handler..
	volatile uint16_t nextWritePosition;			// Indicates up to which array index we have written data to the buffer.
	volatile bool overflow;							// Indicates that at the last call to the writeArray([...]) function
													// the data have hit the buffer limits, causing the write index to return
													// to the beginning of the buffer.
	volatile bool dmaJumpBack;							// Indicates that at the last call to the writeArray function we have hit
													// the end of the tx buffer array, either because we had exactly as many bytes
													// as fit into the buffer or we had more and had to write part of them at the beginning
													// of the buffer.
};


class clSerialPort{
public:
	clSerialPort(USART_TypeDef* usartToUse, uint32_t baudrate, GPIO_TypeDef* gpio_tx, GPIO_TypeDef* gpio_rx, uint16_t pin_tx, uint16_t pin_rx);
	~clSerialPort();

	bool newBytesAvailable();	// Indicates whether there are unread bytes in the input buffer.
	uint8_t readByte();
	void sendTestMessage();
	void writeArray(uint8_t* data, uint16_t length);
	void writeString(char* data, uint16_t numberOfChars);
	void writeU8(uint8_t data);

	// FIXME None of the fields and methods below should be public.
	void handleDMATransferCompleteInterrupt();
	void handleDMAInterruptTx();
	bool isTransmitting();

	static clSerialPort* serialPortUsingUSART1;
	static clSerialPort* serialPortUsingUSART2;
	static clSerialPort* serialPortUsingUSART3;
	USART_TypeDef* hardware;										//pointer to the peripheral the port uses; e.g. USART1
	DMA_Stream_TypeDef * dmaStream_rx;
	DMA_Stream_TypeDef * dmaStream_tx;
private:																		//E.g. if "xxsabc" has been received and the syncChars are "abcd" this variable is 3.																	//if the received chars continue as "xxsabcp" the variable should go back to 0

	void setUpHardware(USART_InitTypeDef usartInitStruct);
	void putNewDataInTxBuffer();									// Called by writeXYZ({[...]) methods to start transmission.
	struct dmaTxBuffer txBuffer;										// TODO Should it be volatile, since changed by interrupts?
	struct dmaRxBuffer rxBuffer;
	volatile bool DMABusyTransmitting;
	volatile bool usartBusyTransmitting;														// Indicates that the USART is transmitting.
	uint32_t DMAStreamTx_interruptNVICID;
	DMA_TypeDef* dma_rx;
	DMA_TypeDef* dma_tx;
	uint32_t dmaChanel_rx;
	uint32_t dmaChanel_tx;
	uint32_t dmaTCInterruptID_tx;
};

extern "C" void USART1_DMA_TransmissionCompleteHandlerTx();
extern "C" void USART2_DMA_TransmissionCompleteHandlerTx();
extern "C" void USART3_DMA_TransmissionCompleteHandlerTx();

#endif /* SERIALPORT_H_ */
