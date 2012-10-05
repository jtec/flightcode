/*
 * \file alGroundLinkNode.cpp
 * \brief An alGroundLinkNode is responsable for communications between UAV and GCS. It receives messages
 * from the ground and forwards them to the bus (not implemented yet).
 * It also reads a spektrum satellite receiver to get RC commands.
 * \author jan
 */

#include "../../convenienceLayer/inc/clFactory.h"
#include "../inc/alGroundLinkNode.h"

/**
 * \brief Constructor.
 */
alGroundLinkNode::alGroundLinkNode(){
	this->factory = clFactory::buildForGroundLinkNode();

	this->mavlink = new alMAVLink(this->factory->getSerialPortXBee());
	alMAVLink*  busmavlink = new alMAVLink(this->factory->getSerialPortBus());

	this->bus = new alBus(busmavlink, RFNode);
	this->rcReceiver = this->factory->getSpektrumSatellite();
	this->RS485Transceiver = this->factory->getRS485Transceiver();

	// Switch on RS485 receiver, keep driver silent.
	this->RS485Transceiver->disableDriver();
	this->RS485Transceiver->enableReceiver();

	//LED to visualize when data has been transmitted.
	this->LED = factory->getStatusLed();
}

/**
 * \brief Destructor
 */
alGroundLinkNode::~alGroundLinkNode(){

}

/**
 * This method runs an infinite loop, performing all the tasks of a groundlink node.
 */
void alGroundLinkNode::runLoop(){
	float rcCommands[10] = {0};
	clSerialPort* port = this->factory->getSerialPortBus();
	alStopwatch*  requestTimer = new alStopwatch();
	requestTimer->restart();
	int32_t nPackets = 0;


	while(true){
		this->bus->tick();
		if(this->bus->newRCCommandRequestAvailable()){
			this->rcReceiver->getRcChannels(rcCommands);
			requestTimer->restart();
			this->RS485Transceiver->enableDriver();
			this->bus->sendServoCommands(rcCommands, mainNode);
			//Wait until the bus is done transmitting:
			while(this->bus->isTransmitting()){

			}
			this->RS485Transceiver->disableDriver();
			nPackets += 1;
			this->bus->RCCommandRequestHandled();
		}
		// To detect mavlink packet losses:
		if(requestTimer->getTime() >200){
			int a = 0;
		}
		//  Blink LED every 100 packets:
		if(nPackets > 100){
			nPackets = 0;
			this->LED->toggle();

		}

	}
}

