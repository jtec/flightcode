/*
 * \file alPeriodicPingNode.cpp
 * \brief This node just sends data every x milliseconds, may be used to verify that
 * the bus hardware works (e.g. to check the RS485 voltage levels).
 * \author jan
 */

#include "../inc/alPeriodicPingNode.h"
#include <math.h>

/**
 * \brief Constructor.
 */
alPeriodicPingNode::alPeriodicPingNode() {
	// Since we only use the bus, just use the hardware setup of the main node.
	clFactory* fac = clFactory::buildForMainNode();
	alMAVLink* mavlink = new alMAVLink(fac->getSerialPortBus());
	this->bus = new alBus(mavlink, periodicPingNode);
	this->sPort = fac->getSerialPortXBee();
	this->LED = fac->getStatusLed();
	this->RS485Transceiver = fac->getRS485Transceiver();
	// Switch on RS485 receiver, keep driver silent.
	this->RS485Transceiver->disableDriver();
	this->RS485Transceiver->enableReceiver();}

/**
 * \brief Destructor.
 */
alPeriodicPingNode::~alPeriodicPingNode() {
}

/**
 * \brief Infinite loop, sends stuff via the bus or directly via the clSerialPort the bus uses,
 * have a look at the method to see what it is actually doing, since it is used for testing and thus
 * changes all the time.
 */
void alPeriodicPingNode::runLoop(){
	int a = 0;
	alStopwatch*  timer = new alStopwatch();
	a = 1;
	timer->restart();
	alStopwatch*  LEDtimer = new alStopwatch();
	LEDtimer->restart();

	alMAVLink* testmavlink = new alMAVLink(this->sPort);
	float sinusan = 0;
	float sinArgument = 1.515;
	uint8_t const length = 250;
	uint8_t const limit = 25;
	uint8_t test[length];
	for(int i=0; i<length; i++){
		test[i] = i;
	}
	while(true){
		//this->bus->tick();
		if(timer->getTime() > 10){
			sinArgument += 0.02;
			sinusan = sin(sinArgument);

			//this->sPort->sendTestMessage();
			//this->sPort->writeArray(test, length);
			testmavlink->sendHeartbeat(MAV_STATE_STANDBY);
			testmavlink->sendFloat(sinusan, "sinusan");
			timer->restart();
		}

		if(LEDtimer->getTime() > 100){
			LEDtimer->restart();
			this->LED->toggle();
		}
	}
}


