/**
 ******************************************************************************
 * \file    alBus.cpp
 * \author  Jan Bolting, Martin Stolle
 * \date    15-July-2012
 * \brief   This class defines objects of type alBus that may be used to transmit and receive
 * messages via the bus connecting the various nodes of the autopilot system. It uses mavlink as a protocol layer.
 * This way the alBus does not have to now a lot about the protocol which should be fairly easy to be replaced
 * by a different one.
 * If there are physical bus drivers (like RS485 Transceivers), they have to be enabled before calling one of this classe's
 * sendX(...) methods and disabled after the alBus is done transmitting (can be checked by the isTransmitting(...) method)
 ******************************************************************************
 * @attention
 * <h2><center>&copy; COPYRIGHT 2012 The Institution</center></h2>
 ******************************************************************************
 */

#include "../../convenienceLayer/inc/clTimeBase.h"
#include "../inc/alBus.h"
alBus* alBus::instance = 0;

/**
 * \brief Constructor.
 * \param[in] mavlinkToUse - a pointer to an alMAVLink object. The bus uses mavlink as protocol layer and this object
 * is responsible for sending and receiving mavlink messages.
 * \param[in] ID - This ID tells you in what kind of node this alBus is working; IDs are defined in alBus.h
 */
alBus::alBus(alMAVLink* mavlinkToUse, nodeIDs ID)
{
	this->mavlink = mavlinkToUse;
	this->nodeID = ID;
	//in case of no received Servo Commands: all channels are initially set to 0
	for(int i=0;i<numchannels;i++){
		this->lastCommands[i] = 0;
	}
	this->mavlink->startCallingMessageHandler(&alBus::mavlinkMessageHandler);
	alBus::instance = this;
	this->hasNewRCCommandRequest = false;
	this->hasNewRCCommands = false;
}
/** Default destructor.
 */
alBus::~alBus(){}

/** The function receives 10 servo commands ([-1.0, 1.0]) and writes them to the bus.
 *
 * \param[in] commands - pointer to an array of 10 float variables.
 * \param[in] targetID - the node id of the receiver of this message.
 * \return void.
 *
 */
void alBus::sendServoCommands(float* commands, nodeIDs targetID){
	uint16_t pwmcommands[8];
	for(int i=0;i<8;i++){
		pwmcommands[i]=1500+commands[i]*500;
	}
	this->mavlink->sendPWMCommands(pwmcommands, (uint8_t)targetID);
}

//! This function stores the latest commands in an private array called commands.
//! It thus protects the incomming servo commands.
/*!
      \param commands pointer to an array of int16_t variables.
      \return void.
      \sa Test(), ~Test(), testMeToo() and publicVar()
 */
void alBus::sendHeartbeat(){
	// FIXME status has nothing to do with actual UAV status.
	this->mavlink->sendHeartbeat(MAV_STATE_STANDBY);
}


/**
 * \brief Keeps the object checking for new incoming messages. Should be called as frequently as possible.
 */
void alBus::tick(){
	this->mavlink->tick();
}


//! This function listens to all incoming messages. Using the message ID it checks if the incoming message-type is
//! a servo command. In case the incoming message (mavlink_message_t) is a servo command, all containing servo
//! commands are first stored in a structure (name = rcOverrideMessage). Afterwards the servoCommands are copied
//! in an array called "lastCommands" enabling easier access and handling.
/*!
      \param mavlink_message_t* msg.
      \return void.
      \sa Test(), ~Test(), testMeToo() and publicVar()
 */
void alBus::mavlinkMessageHandler(mavlink_message_t* msg){
	//The method "mavlink_msg_rc_channels_override_decode" decodes incoming bus messages and stores them into a
	//message-type specific structure.
	static __mavlink_rc_channels_override_t rcCommandMessage;
	static __mavlink_mission_request_t messageRequestMessage;
	//Before decoding the message, the message type is determined.

	// a) If it is an servo commands message:
	if(msg->msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE){
		// Decode message, i.e. convert message byte stream to message structure.
		mavlink_msg_rc_channels_override_decode(msg, &rcCommandMessage);
		// Check if the message is for us:
		if(rcCommandMessage.target_system== (uint8_t)alBus::instance->nodeID)
		{
			// Now copy commands to bus servo command array.
			alBus::instance->lastCommands[0] = (((float)rcCommandMessage.chan1_raw) - 1500)/500;
			alBus::instance->lastCommands[1] = (((float)rcCommandMessage.chan2_raw) - 1500)/500;
			alBus::instance->lastCommands[2] = (((float)rcCommandMessage.chan3_raw) - 1500)/500;
			alBus::instance->lastCommands[3] = (((float)rcCommandMessage.chan4_raw) - 1500)/500;
			alBus::instance->lastCommands[4] = (((float)rcCommandMessage.chan5_raw) - 1500)/500;
			alBus::instance->lastCommands[5] = (((float)rcCommandMessage.chan6_raw) - 1500)/500;
			alBus::instance->lastCommands[6] = (((float)rcCommandMessage.chan7_raw) - 1500)/500;
			alBus::instance->lastCommands[7] = (((float)rcCommandMessage.chan8_raw) - 1500)/500;
			// If so, call the captain to tell him that we got new RC commands.
			alBus::instance->hasNewRCCommands = true;
		}
	}
	// b) If it is a message request message:
	else if(msg->msgid == MAVLINK_MSG_ID_MISSION_REQUEST){
		// Decode message, i.e. convert message byte stream to message structure.
		mavlink_msg_mission_request_decode(msg, &messageRequestMessage);
		// Check if the message is for us:
		if(messageRequestMessage.target_system == (uint8_t)alBus::instance->nodeID)
		{
			//Check which message type (e.g. RC channels, IMU data etc.) has been requested:
			if(messageRequestMessage.target_component == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE){
				alBus::instance->hasNewRCCommandRequest = true;
			}
		}
	}
}

/*
 * \brief Requests a Servo Command message from a certain node identified by it's ID (see enumeration).
 */
void alBus::requestServoCommands(){
	this->mavlink->sendRequestServoCommands(RFNode);
}

/**
 * \brief Returns if the bus currently is transmitting data.
 */
bool alBus::isTransmitting(){
	return this->mavlink->isTransmitting();
}

/**
 * \brief Return value indicates if there have been received new RC commands
 *  since the last have been read.
 */
bool alBus::newRCCommandsAvailable(){
	return this->hasNewRCCommands;
}

/**
 * \brief Writes the last received RC commands to the given array (float, size of 10)
 */
void alBus::getRCCommands(float* cmds){
	for(uint8_t i=0; i<10; i++){
		cmds[i] = this->lastCommands[i];
	}
	this->hasNewRCCommands = false;
}

/**
 * \brief Return value indicates if there have been received a new request for
 *  RC commands since the last has been read.
 */
bool alBus::newRCCommandRequestAvailable(){
	return this->hasNewRCCommandRequest;
}

/**
 * \brief TODo doku.
 */
void alBus::RCCommandRequestHandled(){
	this->hasNewRCCommandRequest = false;
}
