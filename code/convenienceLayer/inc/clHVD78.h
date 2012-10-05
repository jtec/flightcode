/*
 * clHVD78.h
 *
 *  Created on: 01.09.2012
 *      Author: Stealer
 */

#ifndef CLHVD78_H_
#define CLHVD78_H_

#include "clGPIOPin.h"

class clHVD78 {
public:
	clHVD78();
	~clHVD78();
	void enableDriver();
	void disableDriver();
	void enableReceiver();
	void disableReceiver();
	clGPIOPin* driverEnablePin;
	clGPIOPin* receiverEnablePin;
};

#endif /* CLHVD78_H_ */
