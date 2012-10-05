/*
 * \file almainNode.cpp
 *
 * \date Created on: Jul 14, 2012
 * \author: Stealer, Jan
 * \brief You can imagine an alMainNode as the actual autopilot. It manages the data flow on the by, so it is the bus master.
 * The other nodes are only allowed to send data if they have been requested by the main node.
 *
 * For now, the main node just requests RC commands from the RF node and forwards them to the servo node to enable manual flight.
 */

#include "../inc/alMainNode.h"
#include "../inc/alBus.h"

/**
 * \brief Constructor.
 */
alMainNode::alMainNode(){
	//Zeiger auf Objekte, die runloop benötigt um ordentlich zu arbeiten
	//Initialisation of array "receivedServoCommands"
	for(int i=0;i<10;i++){
		this->receivedServoCommands[i]=0;
	}

	this->RCRequestHasBeenServed = false;
	//Bauen einer neuen Factory
	//Factory ERZEUGT (fabriziert) unter anderen zwei SerialPorts.
	clFactory* factory = clFactory::buildForMainNode();
	//Benennen des einen Ports und
	clSerialPort* port = factory->getSerialPortBus();

	this->mavlinkBus = new alMAVLink(port);
	//ein Bus kann nur mit einem Mavlink gebaut werden- deshalb bus(mavlink)
	this->bus = new alBus(this->mavlinkBus, mainNode);
	//Nur Objekte, die nicht von Factory gebildet werden müssen neu erzeugt Werden (bei alBuS ist dies der Falls)

	this->RS485Transceiver = factory->getRS485Transceiver();
	this->LED = factory->getStatusLed();

	factory->getRS485Transceiver()->disableDriver();
	factory->getRS485Transceiver()->enableReceiver();
}

alMainNode::~alMainNode(){
	return;
}

/**
 * \brief This method goes into an infinite loop and periodically requests information from the other nodes,
 * calculates the control answer (for now this means just forwarding the RC input commands to test in manual flight)
 * and sends the control commands to the servo node.
 */
void alMainNode::runloop(){
	alStopwatch*  mainNodeSystemTime = new alStopwatch();
	mainNodeSystemTime->restart();
	int32_t nPackets = 0;

	while(true){
		// Make bus node check for new messages etc.
		this->bus->tick();
		uint32_t time = mainNodeSystemTime->getTime();
		if(time > 1){
			//force stopwatch to reset and start again after 10ms
			mainNodeSystemTime->restart();
			// send RCRequest.
			this->RS485Transceiver->enableDriver();
			// TODO do we have to wait for the driver to be ready (needs 30 ns max according to the datasheet)?
			//this->bus->sendHeartbeat();
			this->bus->requestServoCommands();
			while(this->bus->isTransmitting()){
				int a = 0;
			}
			nPackets += 1;
			this->RS485Transceiver->disableDriver();
		}
		// Check if new RC commands from the RF node have been received:
		if(this->bus->newRCCommandsAvailable()){
			this->bus->getRCCommands(this->receivedServoCommands);
			this->RCRequestHasBeenServed = true;
		}
		if(this->RCRequestHasBeenServed){
			// Forward RC commands to the servo node.
			this->RS485Transceiver->enableDriver();
			// TODO do we have to wait for the driver to be ready (needs 30 ns max according to the datasheet)?
			this->bus->sendServoCommands(this->receivedServoCommands, servoNode);
			while(this->bus->isTransmitting()){
				int a = 0;
			}
			this->RS485Transceiver->disableDriver();
			this->RCRequestHasBeenServed = false;
		}
		//  Blink LED every 100 requests:
		if(nPackets > 100){
			this->LED->toggle();
			nPackets = 0;
		}
	}
}
