/*
 * \file PID.cpp
 * \brief This is a simple Single Input Single Output discrete PID filter.
 * \author jan
 *
 */

#include "../inc/PID.h"
#include "math.h"

/**
 * FIXME Documentation.
 */
PID::PID(float pGain, float iGain, float dGain, float initialOutput, float iLimit) {
	this->pGain = pGain;
	this->iGain = iGain;
	this->dGain = dGain;
	this->iLimit = fabs(iLimit);
	this->integral = 0;
	this->lastInput = 0;
	this->initialOutput = initialOutput;
	this->p = 0;
	this->i = 0;
	this->d = 0;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

/**
 * Returns the filter's current output/state, Imagine as y_k = pid(x_0, x_1...x_k), y_k being the
 * output and x_k being the last input.
 *
 * Example: Imagine applying the filter to a constant value of 2.0 with a sampling frequency of 100 Hz:
 *
 * PID filter(10.0, 0.25, 1.0, 0.0);
 * while(true){
 * 		float filterOutput = filter->getOutput(2.0, 0.01);
 * 		wait_ms(10);
 * }
 * The filter response would start from 0 due to the zero initial value, then jump to 20 (proportional response)
 * after the first time step and then rise with 0.5/s due to the filters integral term.
 *
 * \param[in] input - The latest filter input value.
 * \param[in] dt - The time increment between this input and the one before [s]; needed for computation
 * of the integral and derivative terms.
 *
 */
float PID::getOutput(float input, float dt){
	this->integral = this->integral + input*dt;

	this->p = this->pGain * input;
	this->i = this->iGain*this->integral;
	// Limit I term:
	if(this->i > this->iLimit){
		this->i = this->iLimit;
	}
	if(this->i < -this->iLimit){
			this->i = -this->iLimit;
		}
	// TODO Add smoothing to derivative computing?
	this->d = this->dGain*(input - this->lastInput)/dt;

	this->lastInput = input;

	return this->initialOutput + this->p + this->i + this->d;
}

/**
 * \ brief Sets the P gain to the specified value.
 * \param p - The new P gain.
 */
void PID::setP(float pg){
	this->pGain = pg;
}

/**
 * \ brief Sets the I gain to the specified value.
 * \param i - The new I gain.
 */
void PID::setI(float ig){
	this->iGain = ig;
}

/**
 * \ brief Sets the D gain to the specified value.
 * \param d - The new D gain.
 */
void PID::setD(float dg){
	this->dGain = dg;
}

float PID::getP(){
	return this->p;
}

float PID::getI(){
	return this->i;
}

float PID::getD(){
	return this->d;
}
