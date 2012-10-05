/** @file
 *	@brief MAVLink comm protocol testsuite generated from jtecMavlinkMessages.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef JTECMAVLINKMESSAGES_TESTSUITE_H
#define JTECMAVLINKMESSAGES_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_jtecMavlinkMessages(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_jtecMavlinkMessages(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_actuator_commands(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_actuator_commands_t packet_in = {
		{ 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0 },
	};
	mavlink_actuator_commands_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        
        	mav_array_memcpy(packet1.servoCommands, packet_in.servoCommands, sizeof(float)*10);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_actuator_commands_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_actuator_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_actuator_commands_pack(system_id, component_id, &msg , packet1.servoCommands );
	mavlink_msg_actuator_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_actuator_commands_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.servoCommands );
	mavlink_msg_actuator_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_actuator_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_actuator_commands_send(MAVLINK_COMM_1 , packet1.servoCommands );
	mavlink_msg_actuator_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_jtecMavlinkMessages(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_actuator_commands(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // JTECMAVLINKMESSAGES_TESTSUITE_H
