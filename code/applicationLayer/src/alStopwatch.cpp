/**
* \file alStopwatch.cpp
 * \brief An alStopwatch object is a model of a real stopwatch. It can be used to measure spans of time.
 * Its resolution is 1 ms.
 * TODO Make resolution 1µs
 * \author jan
*/

#include "../inc/alStopwatch.h"

/**
* \brief Constructor. The returned alStopWatch is set to 0 and stopped. Call the restart() method
* to start the clock.
*/
alStopwatch::alStopwatch()
{
	this->startTime_ms = 0;
	this->isRunning = false;
}

/**
* \brief Destructor.
*/
alStopwatch::~alStopwatch()
{
}

/**
 * \brief Resets the clock to 0 and restarts it.
 */
void alStopwatch::restart(){
	this->startTime_ms = TimeBase::getSystemTimeMs();
	this->isRunning = true;
}

/**
 * \brief Stops the clock, does not reset it to 0.
*/
void alStopwatch::stop(){
	this->stopTime = this->getTime();
	this->isRunning = false;
}

/**
 * \brief Returns the time, just imagine as the time you would read from the display of
 * a real stopwatch.
 */
uint32_t alStopwatch::getTime(){
	if(this->isRunning){
		return TimeBase::getSystemTimeMs() - this->startTime_ms;
	}else{
		return this->stopTime;
	}

}
