/*
 * alBusCaptain.h
 *
 *  Created on: 01.09.2012
 *      Author: Stealer
 */

#ifndef ALBUSCAPTAIN_H_
#define ALBUSCAPTAIN_H_

class alBusCaptain {
public:
	alBusCaptain();
	virtual ~alBusCaptain();
	void gotServoMessageRequest();
	//void gotRCCommands(float* values);
};

#endif /* ALBUSCAPTAIN_H_ */
