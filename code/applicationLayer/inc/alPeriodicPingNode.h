/*
 * \file alPeriodicPingNode.h
 * \author jan
 */

#ifndef ALPERIODICPINGNODE_H_
#define ALPERIODICPINGNODE_H_


#include "alStopwatch.h"
#include "alMainNode.h"
#include "alBus.h"
#include "alMAVLink.h"
#include "../../convenienceLayer/inc/clFactory.h"
#include "../../convenienceLayer/inc/clTimebase.h"

class alPeriodicPingNode{
public:
	alPeriodicPingNode();
	~alPeriodicPingNode();
	void runLoop();
private:
	alBus* bus;
	clSerialPort* sPort;
	clLED* LED;
	clHVD78* RS485Transceiver;
};

#endif /* ALPERIODICPINGNODE_H_ */
