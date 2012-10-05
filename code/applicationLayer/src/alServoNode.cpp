/*
 * \file alServoNode.cpp
 * \brief An alServoNode receives messages via the bus and controls up to 16 PWM outputs.
 * \author jan
 */

#include "../../convenienceLayer/inc/clFactory.h"
#include "../inc/alServoNode.h"

/**
 * \brief Constructor. Retrieves readily configured hardware drivers from a clFactory and stores pointers
 * to them as class fields.
 */
alServoNode::alServoNode(){
	// Get hardware drivers.
	this->factory = clFactory::buildForServoNode();
	this->pwmOutput = this->factory->getPWMServoBank();
	// Create alMAVLink for alBus.
	clSerialPort* sPort = this->factory->getSerialPortBus();
	alMAVLink* mavlink = new alMAVLink(sPort);
	this->bus = new alBus(mavlink, servoNode);
	this->RS485Transceiver = this->factory->getRS485Transceiver();
	this->LED = factory->getStatusLed();

	this->RS485Transceiver->disableDriver();
	this->RS485Transceiver->enableReceiver();
}

/**
 * \brief Destructor
 */
alServoNode::~alServoNode(){}

/**
 * This method runs an infinite loop, performing all the tasks of a servo node. It waits for new servo
 * command messages on the bus and updates the PWM output.
 */
void alServoNode::runLoop(){
	float servoCommands[12] = {0};
	int32_t nPackets = 0;

	while(true){
		// Make bus node check for new messages etc.
		this->bus->tick();
		if(this->bus->newRCCommandsAvailable()){
			// Get latest commands received on the bus.
			this->bus->getRCCommands(servoCommands);
			this->pwmOutput->setPositions(servoCommands);
			nPackets += 1;
		}
		if(nPackets > 100){
			this->LED->toggle();
			nPackets = 0;
		}
	}
}
