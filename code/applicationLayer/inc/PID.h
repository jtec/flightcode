/*
 * \file PID.h
 * \author jan
 */

#ifndef PID_H_
#define PID_H_

class PID {
public:
	PID(float pGain, float iGain, float dGain, float initialOutput);
	~PID();
	float getOutput(float input, float dt);
	void setP(float p);
	void setI(float i);
	void setD(float d);
private:
	float pGain;
	float iGain;
	float dGain;
	float integral;
	float lastInput;
	float initialOutput;
};

#endif /* PID_H_ */
