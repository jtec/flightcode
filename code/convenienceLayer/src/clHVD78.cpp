/*
 * \file clHVD78.cpp
 *
 * \date Created on: 01.09.2012
 * \author Stealer
 * \brief Hardware driver for the SN65HVD78 RS485 Transceiver by Texas instruments.
 */

#include "../inc/clHVD78.h"

/**
 * \brief Constructor.
 * TODO: Make Pins and ports constructor parameters.
 */
clHVD78::clHVD78() {
	this->driverEnablePin = new clGPIOPin(GPIOC, GPIO_Pin_2, clGPIOPin::Pinstate_HIGH);
	this->receiverEnablePin = new clGPIOPin(GPIOC, GPIO_Pin_3, clGPIOPin::Pinstate_LOW);
	// TODO Auto-generated constructor stub
}

/**
 * Destructor.
 */
clHVD78::~clHVD78() {
	// TODO Anything to be done here?
}

/**
 * \brief Enables the transceiver's driver stage.
 */
void clHVD78::enableDriver(){
	this->driverEnablePin->set(clGPIOPin::Pinstate_HIGH);
}

/**
 * \brief Disables the transceiver's driver stage.
 */
void clHVD78::disableDriver(){
	this->driverEnablePin->set(clGPIOPin::Pinstate_LOW);

}

/**
 * \brief Enables the transceiver's revceiver stage.
 */
void clHVD78::enableReceiver(){
	this->receiverEnablePin->set(clGPIOPin::Pinstate_LOW);

}

/**
 * \brief Disables the transceiver's receiver stage.
 */
void clHVD78::disableReceiver(){
	this->receiverEnablePin->set(clGPIOPin::Pinstate_HIGH);
}