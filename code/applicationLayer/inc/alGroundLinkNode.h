/*
 * \file alGroundLinkNode.h
 * \author jan
 */

#ifndef ALGROUNDLINKNODE_H_
#define ALGROUNDLINKNODE_H_

#include "almainNode.h"
#include "alBus.h"
#include "alMAVLink.h"
#include "alStopWatch.h"
#include "../../convenienceLayer/inc/clFactory.h"
#include "../../convenienceLayer/inc/clTimebase.h"

/**
 * Structure to store data received from the ground control station.
 */
typedef struct{
	float rcCommands[10];
}GroundData;

class alGroundLinkNode{
public:
	alGroundLinkNode();
	~alGroundLinkNode();
	void runLoop();
private:
	clFactory* factory;
	alBus* bus;
	alMAVLink* mavlink;
	clSpektrumSatellite* rcReceiver;
	clHVD78* RS485Transceiver;
	GroundData lastGroundData;
	clLED* LED;
};

#endif /* ALGROUNDLINKNODE_H_ */
