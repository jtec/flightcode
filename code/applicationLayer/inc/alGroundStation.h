/*
 * alGroundStation.h
 *
 *  Created on: Jul 14, 2012
 *      Author: Stealer
 */

#ifndef ALGROUNDSTATION_H_
#define ALGROUNDSTATION_H_

#include "alGroundStation.h"
#include "alMAVLink.h"
#include "alStopwatch.h"
#include "../../convenienceLayer/inc/clFactory.h"
#include "../../convenienceLayer/inc/clTimebase.h"
#include "../../convenienceLayer/inc/clAssistant.h"

class alGroundStation{
public :
	alGroundStation();
	~alGroundStation();
	void runLoop();
private:
	void mavlinkMessageHandler(mavlink_message_t* msg);
	static alGroundStation instance;
	clSpektrumSatellite* receiver;
	clFactory* factory;
	alMAVLink* mavlink;
};


#endif /* ALGROUNDSTATION_H_ */
