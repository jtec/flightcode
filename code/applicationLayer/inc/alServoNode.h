/*
 * \file alServoNode.h
 * \brief Please give a short description of the file's purpose and content.
 * \author jan
 */

#ifndef ALSERVONODE_H_
#define ALSERVONODE_H_

#include "almainNode.h"
#include "alBus.h"
#include "alMAVLink.h"
#include "../../convenienceLayer/inc/clFactory.h"
#include "../../convenienceLayer/inc/clTimebase.h"

class alServoNode{
public:
	alServoNode();
	~alServoNode();
	void runLoop();
private:
	clFactory* factory;
	alBus* bus;
	clPWMServoBank* pwmOutput;
	clHVD78* RS485Transceiver;
	clLED* LED;
};

#endif /* ALSERVONODE_H_ */
