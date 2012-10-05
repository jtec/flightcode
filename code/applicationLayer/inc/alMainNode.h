/*
 * almainNode.h
 *
 *  Created on: Jul 14, 2012
 *      Author: Stealer
 */

#ifndef MAINNODE_H_
#define MAINNODE_H_


#include "alStopwatch.h"
#include "alBus.h"
#include "alMAVLink.h"
#include "../../convenienceLayer/inc/clFactory.h"
#include "../../convenienceLayer/inc/clTimebase.h"

class alMainNode{
public :
	alMainNode();
	~alMainNode();
	void runloop();
private:
	//Die Klasse hat einen Zeiger auf einen Bus, eine Factory und einen Spektrumsatelliten
	alBus* bus;
	alMAVLink* mavlinkBus;
	float receivedServoCommands[10];
	bool RCRequestHasBeenServed;
	clHVD78* RS485Transceiver;
	clLED* LED;

};

#endif /* MAINNODE_H_ */
