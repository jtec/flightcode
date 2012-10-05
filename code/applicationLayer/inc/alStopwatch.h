/*
 * \file alStopwatch.h
 */

#ifndef ALSTOPWATCH_H_
#define ALSTOPWATCH_H_

#include "../../convenienceLayer/inc/clTimebase.h"

class alStopwatch{
public:
	alStopwatch();
	~alStopwatch();
	void stop();
	void restart();
	uint32_t getTime();
private:
	uint32_t startTime_ms;
	uint32_t stopTime;
	bool isRunning;
};



#endif /* ALSTOPWATCH_H_ */
