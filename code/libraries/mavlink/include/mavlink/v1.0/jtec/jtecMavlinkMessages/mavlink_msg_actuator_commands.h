// MESSAGE ACTUATOR_COMMANDS PACKING

#define MAVLINK_MSG_ID_ACTUATOR_COMMANDS 200

typedef struct __mavlink_actuator_commands_t
{
 float servoCommands[10]; ///< 10 Servo commands [-1000,1000] 
} mavlink_actuator_commands_t;

#define MAVLINK_MSG_ID_ACTUATOR_COMMANDS_LEN 40
#define MAVLINK_MSG_ID_200_LEN 40

#define MAVLINK_MSG_ACTUATOR_COMMANDS_FIELD_SERVOCOMMANDS_LEN 10

#define MAVLINK_MESSAGE_INFO_ACTUATOR_COMMANDS { \
	"ACTUATOR_COMMANDS", \
	1, \
	{  { "servoCommands", NULL, MAVLINK_TYPE_FLOAT, 10, 0, offsetof(mavlink_actuator_commands_t, servoCommands) }, \
         } \
}


/**
 * @brief Pack a actuator_commands message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param servoCommands 10 Servo commands [-1000,1000] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_commands_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const float *servoCommands)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];

	_mav_put_float_array(buf, 0, servoCommands, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 40);
#else
	mavlink_actuator_commands_t packet;

	mav_array_memcpy(packet.servoCommands, servoCommands, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 40);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACTUATOR_COMMANDS;
	return mavlink_finalize_message(msg, system_id, component_id, 40, 95);
}

/**
 * @brief Pack a actuator_commands message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param servoCommands 10 Servo commands [-1000,1000] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_commands_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const float *servoCommands)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];

	_mav_put_float_array(buf, 0, servoCommands, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 40);
#else
	mavlink_actuator_commands_t packet;

	mav_array_memcpy(packet.servoCommands, servoCommands, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 40);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACTUATOR_COMMANDS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 40, 95);
}

/**
 * @brief Encode a actuator_commands struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param actuator_commands C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_commands_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_actuator_commands_t* actuator_commands)
{
	return mavlink_msg_actuator_commands_pack(system_id, component_id, msg, actuator_commands->servoCommands);
}

/**
 * @brief Send a actuator_commands message
 * @param chan MAVLink channel to send the message
 *
 * @param servoCommands 10 Servo commands [-1000,1000] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_actuator_commands_send(mavlink_channel_t chan, const float *servoCommands)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[40];

	_mav_put_float_array(buf, 0, servoCommands, 10);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_COMMANDS, buf, 40, 95);
#else
	mavlink_actuator_commands_t packet;

	mav_array_memcpy(packet.servoCommands, servoCommands, sizeof(float)*10);
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_COMMANDS, (const char *)&packet, 40, 95);
#endif
}

#endif

// MESSAGE ACTUATOR_COMMANDS UNPACKING


/**
 * @brief Get field servoCommands from actuator_commands message
 *
 * @return 10 Servo commands [-1000,1000] 
 */
static inline uint16_t mavlink_msg_actuator_commands_get_servoCommands(const mavlink_message_t* msg, float *servoCommands)
{
	return _MAV_RETURN_float_array(msg, servoCommands, 10,  0);
}

/**
 * @brief Decode a actuator_commands message into a struct
 *
 * @param msg The message to decode
 * @param actuator_commands C-struct to decode the message contents into
 */
static inline void mavlink_msg_actuator_commands_decode(const mavlink_message_t* msg, mavlink_actuator_commands_t* actuator_commands)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_actuator_commands_get_servoCommands(msg, actuator_commands->servoCommands);
#else
	memcpy(actuator_commands, _MAV_PAYLOAD(msg), 40);
#endif
}
