
#ifndef ALBUS_H_
#define ALBUS_H_

#include "alMAVLink.h"

enum nodeIDs {mainNode, RFNode, servoNode, periodicPingNode};


class alBus{
public:
        alBus(alMAVLink* mavlinkToUse, nodeIDs ID);
		~alBus();
		void requestServoCommands();
		void sendServoCommands(float* commands, nodeIDs targetID);
		void sendHeartbeat();
		void tick();
		bool isTransmitting();
		bool newRCCommandsAvailable();
		bool newRCCommandRequestAvailable();
		void getRCCommands(float* cmds);
		void RCCommandRequestHandled();
		alMAVLink* mavlink;
private:
		static const uint8_t numchannels = 10;
		float lastCommands[numchannels];
		static void mavlinkMessageHandler(mavlink_message_t* msg);
		static alBus* instance;
		nodeIDs nodeID;
		bool hasNewRCCommandRequest;
		bool hasNewRCCommands;
};

#endif /* ALBUS_H_ */
