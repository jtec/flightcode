/**
 * \file GroundLink.cpp
 * \brief Manages communications with an adapter merging the RC inputs and data from QGroundControl.
 *
 * TODO Decide wether to talk about 'mission items' (more general terminology used by mavlink 1.0) or 'waypoints' (less abstract)
 * and adapt comments and variable names for the mission item transaction code accordingly.
 */

#include "../inc/GroundLink.h"
#include "clTimeBase.h"

GroundLink* GroundLink::instance = 0; // Used by the static MAVlink message handler method to access instance variables. TODO Should not be initialized to zero.

/**
 * \brief Constructor, returns an instance of GroundLink.
 * \param[in] portNumber the number of serial port to be used.
 */
GroundLink::GroundLink(clSerialPort* serialPortXBEe):mavLink(serialPortXBEe)
{
	this->hasParameterManager = false;

	// Initialize link state.
	this->linkState.txWindowOpen = false;
	this->linkState.txTimeLeft = 0;
	this->linkState.prescaler = 10;
	this->linkState.prescalerCounter = 0;

	// Initialize parameter send/receive state machine.
	this->linkState.parameterTransactionManager.state = PAR_IDLE;
	this->linkState.parameterTransactionManager.txCounter = 0;
	this->linkState.parameterTransactionManager.indexOfLastReceivedParameter = 0;

	GroundLink::instance = this;
	//GroundLink::UAVDebug = this;
	this->mavLink.startCallingMessageHandler(&GroundLink::mavlinkMessageHandler);
	// For debugging: echo data.
	//this->mavLink.startForwardingMessages(&this->mavLink);
	this->timeOfLastMessage = 0;
	this->numberOfConnectionTimeouts = 0;
	this->connectionTimeoutTime = 3000.0;   // Consider the connection as lost after 3 s.
}

/**
 * \brief Destructor.
 */
GroundLink::~GroundLink()
{
}

/*
 * \brief Makes the GroundLink update its inner state, check for new received bytes, send data etc.
 */
void GroundLink::tick(){
	static uint32_t timeOfLastSend = 0;
	this->mavLink.tick();
	// Check for connection timeout.
	if((TimeBase::getSystemTimeMs() - this->timeOfLastMessage) > (uint32_t)this->connectionTimeoutTime){
		this->numberOfConnectionTimeouts += 1;
		this->timeOfLastMessage = TimeBase::getSystemTimeMs(); 	//Since the variable 'this->numberOfConnectionTimeouts' reflects the number of timeout periods,
		// after each timeout event, the timer has to be reset to avoid incrementing 'this->numberOfConnectionTimeouts'
		// each time this method is called after the first timeout occurred.
	}
	if((TimeBase::getSystemTimeMs() - timeOfLastSend) > 1000){
		timeOfLastSend = TimeBase::getSystemTimeMs();
		uint8_t mavlinkSystemState = MAV_STATE_UNINIT;
		// Send a heartbeat message to show that the system works at all.
		this->mavLink.sendHeartbeat(mavlinkSystemState);
	}

	this->evaluateParameterTransactionStateMachine();
	return;
}

/**
 * \brief Makes this GroundLink send an info message to the GCS. Enable other UAVComponents to
 *        make visible whats going on to the drone operator. Should not be used intensively,
 *        since it consumes a significant amount of bandwidth (51 payload bytes). Messages are buffered
 *        and sent with 1 Hz.
 */
void GroundLink::sendInfoMessage(char* text){
	// TODO We should not access the mavlink protocol here, since it is supposed to be hidden by the MAVlinkMAVion class.
	// Maybe add buffer functionality to MAVLinkMAVion class instead (methods: bufferMessage(packMessageX([...])), sendMessage(packMessageX([...])) ).
	static mavlink_message_t bufferMessage;
	uint16_t l = mavlink_msg_statustext_pack(this->mavLink.mavlink_system.sysid,
			this->mavLink.mavlink_system.compid,
			&bufferMessage,
			MAV_SEVERITY_INFO,
			text);
	this->mavLink.sendMessage(&bufferMessage);
}
/**
 * \brief Evaluates the state machine for parameter transactions between the GCS and the UAV.
 */
void GroundLink::evaluateParameterTransactionStateMachine(){
	static float value = 0.0; // Used below.
	static char textID[17];   // Used below.
	uint16_t totalNumberOfParams = 0; // Used below.

	switch(this->linkState.parameterTransactionManager.state){
	case PAR_IDLE:
		break;

	case PAR_LIST_REQUEST_RECEIVED:

		totalNumberOfParams = this->parameterManager->getNumberOfFloats();
		if(this->hasParameterManager){
			if(this->linkState.parameterTransactionManager.txCounter < totalNumberOfParams){
				this->parameterManager->getFloat(this->linkState.parameterTransactionManager.txCounter, &value, textID);
				this->mavLink.sendFloatParameter(value,
						totalNumberOfParams,
						this->linkState.parameterTransactionManager.txCounter,
						textID,
						MAVLINK_TYPE_FLOAT
				);
				this->linkState.parameterTransactionManager.txCounter += 1;
			}
			else{
				// When all parameters are sent, get back to idle state.
				this->linkState.parameterTransactionManager.state = PAR_IDLE;
			}
		}
		break;

	case PAR_GOT_PARAMETER:
		totalNumberOfParams = this->parameterManager->getNumberOfFloats();
		// Acknowledge parameter by sending it back to the GCS.
		this->parameterManager->getFloat(this->linkState.parameterTransactionManager.indexOfLastReceivedParameter, &value, textID);
		this->mavLink.sendFloatParameter(value,
				totalNumberOfParams,
				this->linkState.parameterTransactionManager.txCounter,
				textID,
				MAVLINK_TYPE_FLOAT
		);
		// Go back to idle state.
		this->linkState.parameterTransactionManager.state = PAR_IDLE;
		break;

	default:
		break;
	}
}


/**
 * \brief Gives the GroundLink a reference to a ParameterManager object it may use to
 *        change onboard parameters and send their values to the GCS.
 */
void GroundLink::introduceComponent(ParameterManager* p){
	this->parameterManager = p;
	this->hasParameterManager = true;
	// Make the connection timeout time criterion configurable by the GCS.
	this->parameterManager->checkInFloat(&(this->connectionTimeoutTime), "gLink_timeoutMS", 16);
}

/**
 * \brief Return value indicates how often the connection to the GCS has been lost for more than
 *        a certain timeout period.
 * \return Number of times the connection has timed out (no message received for x ms) since this GroundLink has been created.
 */
int16_t GroundLink::howManyConnectionTimeouts(){
	return this->numberOfConnectionTimeouts;
}

/**
\brief This static method is called by the GroundLink's MAVLink component whenever
 *      a new message has been received. It takes the appropriate actions depending on the message type.
 */
void GroundLink::mavlinkMessageHandler(mavlink_message_t* msg){
	// Structures to keep decoded messages. They are declared as static because this way
	// memory is allocated only once and not each time the method gets called.
	static mavlink_param_request_list_t parameterListRequestMessage;
	static mavlink_param_set_t parameterSetMessage;

	GroundLink* instance = GroundLink::instance;  // The instance of GroundLink this static method is operating on.
	instance->timeOfLastMessage = TimeBase::getSystemTimeMs(); // This may be used to detect connection timeouts.

	switch(msg->msgid)
	{
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		mavlink_msg_param_request_list_decode(msg, &parameterListRequestMessage);
		// Check if this message is for us:
		if(instance->mavLink.isMySystemID(parameterListRequestMessage.target_system)){
			// set up counter in order to transmit the first parameter first:
			instance->linkState.parameterTransactionManager.txCounter = 0;
			instance->linkState.parameterTransactionManager.state = PAR_LIST_REQUEST_RECEIVED;
		}
		break;

	case MAVLINK_MSG_ID_PARAM_SET:
		mavlink_msg_param_set_decode(msg, &parameterSetMessage);
		// Check if this message is for us:
		if(instance->mavLink.isMySystemID(parameterSetMessage.target_system)){
			// Assign value to onboard parameter.
			instance->linkState.parameterTransactionManager.indexOfLastReceivedParameter = instance->parameterManager->setFloat(parameterSetMessage.param_value, parameterSetMessage.param_id);
			instance->linkState.parameterTransactionManager.state = PAR_GOT_PARAMETER;
		}
		break;

	default:
		break;
	}
}
