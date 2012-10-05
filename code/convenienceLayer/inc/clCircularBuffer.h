/*
 * \file CircularBuffer.h
 * \brief This is a circular buffer, may be used for serial communications for example.
 * \author jan
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"


class CircularBuffer{
public:
	CircularBuffer();
	~CircularBuffer();
	void addByte(char c);			// Writes 1 byte to the buffer.
	char getByte();					// Returns 1 byte FIFO - style.
	volatile char* getReadpointer();			// Indicates until which memory address data have been read from the buffer
									// i.e. the next byte will be read from this address.
	volatile char* getWritepointer();		// Indicates at which memory address the next byte will be written.
	uint16_t getReadIndex();
	uint16_t getWriteIndex();
	uint16_t getNumberOfUnreadBytes();		// Returns the number of bytes that have been put into the buffer but not read yet.
private:
	// Note: Some variables are 'volatile' because the buffer may be accessed by interrupt handlers or a DMA.
	static const uint16_t bufferSize = 500;
	volatile uint16_t readIndex;				// Indicates which index of the buffer array the next byte should be read from.
	volatile uint16_t writeIndex;			// Indicates at which index the next byte should be written.
	volatile uint16_t unreadbytes;			// Indicates how many bytes have been put into the buffer but not read yet.
	volatile uint16_t buffersize;
	volatile char buffer[bufferSize];			// The actual buffer array holding the buffered data. TODO make capacity a constructor parameter.

};

#endif /* CIRCULARBUFFER_H_ */
