/*
 * \file CircularBuffer.cpp
 * \brief This is a simple circular buffer, may be used for serial communications for example.
 * \author jan
 */

#include "../inc/clCircularBuffer.h"

/**
 * \brief Constructor.
 */
CircularBuffer::CircularBuffer(){
	this->readIndex = 0;
	this->writeIndex = 0;
	this->unreadbytes = 0;
	this->buffersize = CircularBuffer::bufferSize;
	// Initialize buffer to some defined value, helpful for debugging:
	for(uint16_t i=0; i<this->buffersize; i++){
		this->buffer[i] = 77;
	}

}

/**
 * \brief Destructor.
 */
CircularBuffer::~CircularBuffer(){
}

/**
 * \brief Puts one byte into the buffer.
 */
void CircularBuffer::addByte(char c){
	this->buffer[this->writeIndex] = c;
	this->unreadbytes += 1;
	this->writeIndex += 1;
	// Check for potential overflow:
	if(this->writeIndex >= this->buffersize){
		this->writeIndex = 0;
	}
}

/**
 * \brief Pulls one byte from the buffer.
 * \return The byte after the byte that has been returned when this function has been
 * called the last time. If a byte is requested after the last byte in the buffer has been
 * returned, this function returns 0;
 */
char CircularBuffer::getByte(){
	// If all bytes have been read from the buffer, return 0.
	if(this->unreadbytes == 0){
		return 0;
	}
	// Otherwise return the next byte.
	else{
		char c = this->buffer[this->readIndex];
		this->readIndex += 1;
		// Check for overflow:
		if(this->readIndex >= this->buffersize){
			this->readIndex = 0;
		}
		this->unreadbytes -= 1;
		return c;
	}
}

/**
 * \brief Use this function to find out how many bytes have been put into the buffer but haven't been read yet.
 * \return number of unread bytes.
 */
uint16_t CircularBuffer::getNumberOfUnreadBytes(){
	return this->unreadbytes;
}
