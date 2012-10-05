/*
 * \file alGroundStation.cpp
 * \brief This class provides the functionality of a ground control station adapter. It is
 * connected to the GCS by an USART-USAB adapter. It
 * a) receives mavlink messages from QGroundControl and forwards them to the UAV using a serial link connected to an XBee RF modem.
 * b) reads the output of a spektrum satellite receiver and sends it to the UAV
 * c) receives messages sent by the UAV and forwards them to QGroundControl.
 * To turn your system into a groundstation adapter, just create an instance of this class
 * and call its runloop() method.
 *
 *  Created on: Jul 14, 2012
 *  \author: Jan Bolting
 */
#include "../inc/alGroundStation.h"

alGroundStation::alGroundStation(){
	this->factory = clFactory::buildForGroundStation();
	// Get serial port connected to XBee modem.
	clSerialPort* sPort = this->factory->getSerialPortXBee();
	// Create mavlink port for connection to UAV.
	this->mavlink = new alMAVLink(sPort);
	// Get spektrum satellite receiver for RC commands.
	this->receiver = this->factory->getSpektrumSatellite();
}

alGroundStation::~alGroundStation(){
	return;
}

void alGroundStation::runLoop(){

	alStopwatch rcTimer;
	alStopwatch heartbeatTimer;
	float receivedRCCommands[10] = {0.0};
	uint16_t pwmCommands[10] = {0.0};
	rcTimer.restart();
	heartbeatTimer.restart();

	while(true){
		this->mavlink->tick();
		if(rcTimer.getTime() > 50){
			// Read RC receiver output.
			this->receiver->getRcChannels(receivedRCCommands);
			// Convert to pwm;
			clAssistant::float2pwm(receivedRCCommands, pwmCommands, 10);
			// Send it to the UAV.
			this->mavlink->sendPWMCommands(pwmCommands, 0);	// FIXME Define proper target ID.
			//char variableName[16] = "channel_1";
			//this->mavlink->sendFloat(receivedRCCommands[0], variableName);
			rcTimer.restart();
		}
		if(heartbeatTimer.getTime() > 1000){
			// FIXME Just here to make QGroundControl display values.
			this->mavlink->sendHeartbeat(MAV_STATE_STANDBY);
			this->factory->getStatusLed()->toggle();
			heartbeatTimer.restart();
		}
	}
}

//! This function listens to all incoming messages. Using the message ID it checks if the incoming message-type is
//! a servo command. In case the incoming message (mavlink_message_t) is a servo command, all containing servo
//! commands are first stored in a structure (name = rcOverrideMessage). Afterwards the servoCommands are copied
//! to an array called "lastCommands" enabling easier access and handling.
    /*!
      \param mavlink_message_t* msg.
      \return void.
      \sa Test(), ~Test(), testMeToo() and publicVar()
    */
void alGroundStation::mavlinkMessageHandler(mavlink_message_t* msg){

}

