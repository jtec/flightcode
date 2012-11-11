/**
 * \file alMAVLink.cpp
 * \brief This class may be used to exchange alMAVLink messages via a serial port.
 *        It can be configured to
 *        a) forward every message to another alMAVLink
 *        b) call a handler function each time a new message is received and pass
 *           it a pointer to this message.
 *
 */

#include "../inc/alMAVLink.h"
#include "../../convenienceLayer/inc/clTimeBase.h"
#include "../../convenienceLayer/inc/clSerialPort.h"

/**
 * \brief Constructor, returns an instance of alMAVLink.
 * \param
 */
alMAVLink::alMAVLink(clSerialPort* serialPort)
{
	this->mySerialPort = serialPort;
	this->systemID = 1;
	this->forwardEveryMessage = false;
	this->callMessageHandler = false;

	// TODO make mavlink system configurable
	this->mavlink_system.sysid = 1;
	this->mavlink_system.compid = MAV_COMP_ID_IMU;
	this->mavlink_system.type = MAV_TYPE_FIXED_WING;
}

/**
 * \brief Destructor.
 */
alMAVLink::~alMAVLink()
{
}

/*
 * \brief Sends the heartbeat message
 */
void alMAVLink::sendHeartbeat(uint8_t systemState)
{
	// Define the system type, in this case an airplane
	uint8_t system_type = MAV_TYPE_FIXED_WING;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

	uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
	uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter

	// Initialize the required buffers
	static mavlink_message_t msg;

	// Pack the message
	mavlink_msg_heartbeat_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			system_type,
			autopilot_type,
			system_mode,
			custom_mode,
			systemState);

	alMAVLink::sendMessage(&msg);
	return;
}

/*
 * \brief Sends the euler angles and angular rates in the body frame using the alMAVLink protocol.
 * \param[in] a attitudeEuler structure pointer
 */
void alMAVLink::sendAttitude(attitudeEuler* a)
{
	static mavlink_message_t msg;

	// Pack the message
	uint32_t time = TimeBase::getSystemTimeMs();
	uint16_t length = mavlink_msg_attitude_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			time,
			a->euler[0],
			a->euler[1],
			a->euler[2],
			a->bodyRotationRate[0],
			a->bodyRotationRate[1],
			a->bodyRotationRate[2]);

	alMAVLink::sendMessage(&msg);
	return;
}

/*
 * \brief Sends the attitude (quaternion) using the alMAVLink protocol.
 * \param[in] state attitudeEuler structure pointer
 */
void alMAVLink::sendAttitude(attitudeQuaternion* q)
{
	static mavlink_message_t msg;
	// Pack the message
	uint32_t time = TimeBase::getSystemTimeMs();
	uint16_t len = mavlink_msg_attitude_quaternion_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			time,
			q->w,
			q->x,
			q->y,
			q->z,
			q->bodyRotationRate[0],
			q->bodyRotationRate[1],
			q->bodyRotationRate[2]);

	alMAVLink::sendMessage(&msg);
	return;
}

/*
 * \brief Sends the inertial and magnetic field measurements in the body frame using the alMAVLink protocol.
 * \param[in] state measurementsInertial structure pointer
 */
void alMAVLink::sendAttitude(measurementsInertial* a)
{
	static mavlink_message_t msg;

	// Pack the message
	uint32_t time = TimeBase::getSystemTimeMs();
	uint16_t length = mavlink_msg_scaled_imu_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			time,
			(uint16_t)((a->acc[0]/9.81)*1000.0),	// new unit: [mg]	FIXME Use proper value for g.
			(uint16_t)((a->acc[1]/9.81)*1000.0),
			(uint16_t)((a->acc[2]/9.81)*1000.0),
			(uint16_t)(a->gyro[0]*1000.0),		// new unit: [millirad]
			(uint16_t)(a->gyro[1]*1000.0),
			(uint16_t)(a->gyro[2]*1000.0),
			(uint16_t)a->magField[0],		// keeps its unit: [milliTesla]
			(uint16_t)a->magField[1],
			(uint16_t)a->magField[2]
	);

	alMAVLink::sendMessage(&msg);
	return;
}

/**
 * \brief Sends 8 float commands [-1.0, 1.0] to a target system.
 * TODo Uses the PWM message.
 */
void alMAVLink::sendFloatCommands(float* commands, uint8_t targetID){
	static uint16_t pwm[8] = {0};
	// Convert float to pwm:
	float2pwm(8, commands, pwm);
	// Send:
	this->sendPWMCommands(pwm, targetID);
	return;
}


/**
 * \brief Sends 8 PWM values to a target system.
 */
void alMAVLink::sendPWMCommands(uint16_t* pulseWidths, uint8_t targetID){
	static mavlink_message_t msg;
	// Pack the message
	uint32_t time = TimeBase::getSystemTimeMs();
	uint16_t len = mavlink_msg_rc_channels_override_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			targetID,
			MAV_COMP_ID_ALL,
			pulseWidths[0],
			pulseWidths[1],
			pulseWidths[2],
			pulseWidths[3],
			pulseWidths[4],
			pulseWidths[5],
			pulseWidths[6],
			pulseWidths[7]);

	alMAVLink::sendMessage(&msg);
	return;
}

/**
 * Sends 10 actuator commands running from -1 to 1.
 */
void alMAVLink::sendActuatorCommands(float* cmds){
	static mavlink_message_t msg;
	// Pack the message
	mavlink_msg_actuator_commands_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			cmds);
	alMAVLink::sendMessage(&msg);
	return;
}

/*
 * \brief Sends a named float value. Maximum length of name: 10 characters.
 */
void alMAVLink::sendFloat(float value, char* name){
	static mavlink_message_t msg;

	// Pack the message
	uint32_t time = TimeBase::getSystemTimeMs();
	mavlink_msg_named_value_float_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			time,
			name,
			value);

	alMAVLink::sendMessage(&msg);
}

/*
 * \brief Sends a float value and an index to discriminate between values.
 */
void alMAVLink::sendFloat(float value, uint8_t index){
	static mavlink_message_t msg;
	// Pack the message
	uint32_t time = TimeBase::getSystemTimeMs();
	uint16_t length = mavlink_msg_debug_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			time,
			index,
			value);

	alMAVLink::sendMessage(&msg);
}

/*
 * \brief Sends a named signed 32 integer value. Maximum length of name: 10 characters.
 */
void alMAVLink::sendInt32(int32_t value, char* name){
	static mavlink_message_t msg;

	// Pack the message
	uint32_t time = TimeBase::getSystemTimeMs();
	mavlink_msg_named_value_int_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			time,
			name,
			value
	);

	alMAVLink::sendMessage(&msg);
}

/**
 * \brief Writes a message to the mavlink buffer and writes the buffer to the serial port.
 */
void alMAVLink::sendMessage(mavlink_message_t* msg){
	// Copy the message to the send buffer
	static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
	// Write message bufer to serial port.
	this->mySerialPort->writeArray(buf, len);
}

/**
 * \brief Sends a parameter value.
 *
 * \param[in] value - the value of the parameter, e.g. 1.23.
 * \param[in] totalNumberOfParameters - how many onboard parameters exist.
 * \param[in] indexOfParameter - the index of this parameter in the list of parameters.
 * \param[in] paramID - the String ID of this parameter, e.g. "Pgain_engineLaw", max. 16 characters.
 * \param[in] parameterType - one of the types defined in the mavlink_message_type_t typedef in mavlink's mavlink_types.h
 */
void alMAVLink::sendFloatParameter(float value, uint16_t totalNumberOfParameters, uint16_t indexOfParameter, char* paramID, uint8_t parameterType){
	static mavlink_message_t msg;
	// Pack message.
	uint16_t len = mavlink_msg_param_value_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg,
			paramID,
			value,
			parameterType,
			totalNumberOfParameters,
			indexOfParameter);
	// Send message:
	alMAVLink::sendMessage(&msg);
}

/*
 * \brief Sets the serial link this alMAVLink is supposed to use.
 * \param[in] port The LiaisonSerie this alMAVLink is supposed to use for data transfer.
 */
void alMAVLink::setPort(clSerialPort* port)
{
	this->mySerialPort = port;
	return;
}

/**
 * \brief Defines where the alMAVLink forwards messages to.
 *        Before this method has been called, no message is forwarded.
 */
void alMAVLink::startForwardingMessages(alMAVLink* target){
	this->forwardTo = target;
	this->forwardEveryMessage = true;
}

/**
 * \brief Makes the alMAVLink stop forwarding messages.
 */
void alMAVLink::stopForwardingMessages(){
	this->forwardEveryMessage = false;
}

/**
 * \brief Makes the MAVlink call the given message handler every time a mavlink message is received.
 * \param[in] messageHandler The function that is called. MAVlink passes a pointer to the received message to this function.
 * TODO Check if message handling takes too much time and may hence cause the serial port's input buffer to overflow.
 * TODO Make method accept an object pointer instead of a function pointer.
 */
void alMAVLink::startCallingMessageHandler(void (*handler)(mavlink_message_t*)){
	this->callMessageHandler = true;
	this->messageHandler = handler;
}
/**
 * \brief Makes the MAVlink stop calling the message handler function.
 */
void alMAVLink::stopCallingMessageHandler(){
	this->callMessageHandler = false;
}

/*
 * \brief Call this function at least as fast as bytes can arrive at the serial connection.
 *        It checks for new received bytes and decodes alMAVLink messages.
 */
void alMAVLink::tick(){
	this->checkForNewBytes();
}

/*
 * \brief This method checks for new received bytes and decodes alMAVLink messages.
 */
void alMAVLink::checkForNewBytes(){
	// TODO make function use a timeout in case there is an endless stream of bytes coming in.
	// TODO Maybe add message filter.
	// Buffer messages used for decoding.
	static mavlink_message_t msg;
	static mavlink_status_t status;
	static uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	while(this->mySerialPort->newBytesAvailable()){
		uint8_t c = this->mySerialPort->readByte();

		// Check if we have received a complete message yet:
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle messages.
			switch(msg.msgid){
			case MAVLINK_MSG_ID_HEARTBEAT:
				this->timeOfLastHeartBeat = TimeBase::getSystemTimeMs();
				break;
			default:
				break;
			}
			if(this->forwardEveryMessage){
				// Forward each message
				// Copy the message to the tx buffer.
				uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
				//send the buffer
				this->forwardTo->sendRawData(len, buf);
			}
			if(this->callMessageHandler){
				this->messageHandler(&msg);
			}
		}

	}
}

/*
 * \brief Returns how much time has passed since the last heartbeat message has been received by this alMAVLink.
 * TODO Test.
 */
uint32_t alMAVLink::getTimeSinceLastHeartBeat(){
	return TimeBase::getSystemTimeMs() - this->timeOfLastHeartBeat;
}

/*
 * \brief Sends an array of bytes, the user has to take care of the data structure,
 *        so it is not guaranteed to send a valid mavlink message.
 */
void alMAVLink::sendRawData(uint16_t length, uint8_t* buf){
	this->mySerialPort->writeArray(buf, length);
}

/*
 * \brief Sets all fields of a given Structure to their default values.
 * \param[in] s The attitudeEuler structure to be initialized.
 */
void alMAVLink::initStructure(attitudeEuler *s){
	for(uint8_t i=0; i<3; i++){
		s->euler[i] = 0;
		s->bodyRotationRate[i] = 0;
	}
}

/*
 * \brief Sets all fields of a given Structure to their default values.
 * \param[in] s The attitudeEuler structure to be initialized.
 */
void alMAVLink::initStructure(attitudeQuaternion *q){
	q->w = 0;
	q->x = 0;
	q->y = 0;
	q->z = 0;
	for(uint8_t i=0; i<3; i++){
		q->bodyRotationRate[i] = 0;
	}
}

/*
 * \brief Sets all fields of a given Structure to their default values.
 * \param[in] s The measurementsInertial structure to be initialized.
 */
void alMAVLink::initStructure(measurementsInertial* s){
	for(uint8_t i=0; i<3; i++){
		s->acc[i] = 0;
		s->gyro[i] = 0;
		s->magField[i] = 0;
	}
}

/*
 * \brief Requests a certain message (messageID) from a certain (targetID) node.
 * \param[in] targetID i.e the address of the bus node the requested data come from.
 */
void alMAVLink::sendRequestServoCommands(uint8_t targetID)
{
	static mavlink_message_t msg;

	// Pack the message
	mavlink_msg_mission_request_pack(this->mavlink_system.sysid,
			this->mavlink_system.compid,
			&msg, targetID, MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE, 0);
	alMAVLink::sendMessage(&msg);
	return;
}

/**
 * \brief Use this method to check whether the alMAVlink is currently transmitting data or not.
 * \return Boolean indicating wheter this is transmitting or not.
 */
bool alMAVLink::isTransmitting(){
	return this->mySerialPort->isTransmitting();
}

/**
 * \brief Checks wether a given ID is the ID of this MAVLinkMAVion.
 * \param[in] idToCheck - The ID to be checked for identity with this MAVLinkMAVion'S ID.
 * \return boolean, true if the given ID is the ID of this MAVLinkMAVion, false if not.
 */
bool alMAVLink::isMySystemID(uint8_t idToCheck){
	return idToCheck == this->mavlink_system.sysid;
}

/**
 * \brief Converts an array of PWM values [1000, 2000] into float values [-1.0, 1.0]and writes them to a second array.
 * \param[in] n - Number of values to convert, i.e. the both arrays' lengths.
 * \param[in] pwm - Pointer to an aray of pwm values.
 * \param[out] flt - Pointer to an aray of float values.
 */
void alMAVLink::pwm2float(int32_t n, uint16_t* pwm, float* flt){
	for(int i=0; i<n; i++){
		flt[i] = (((float)pwm[i])-1500)/500;
	}
	return;
}

/**
 * \brief Converts an array of float values [-1.0, 1.0] into an array of PWM values [1000, 2000].
 * \param[in] n - Number of values to convert, i.e. both arrays' lengths.
 * \param[in] flt - Pointer to an aray of float values.
 * \param[out] pwm - Pointer to an aray of pwm values.
 */
void alMAVLink::float2pwm(uint16_t n, float* flt, uint16_t* pwm){
	for(int i=0; i<n; i++){
		pwm[i] = (flt[i]*500+1500);
	}
	return;
}

