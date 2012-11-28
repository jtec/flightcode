/*
 * \file PID.h
 * \author jan
 */

#ifndef PID_H_
#define PID_H_

class PID {
public:
	PID(float pGain, float iGain, float dGain, float initialOutput, float iLimit);
	~PID();
	float getOutput(float inputP, float inputI, float inputD, float dt);
	void setP(float p);
	void setI(float i);
	void setD(float d);
	float getP();
	float getI();
	float getD();
private:
	float pGain;
	float iGain;
	float dGain;
	float iLimit;
	float initialOutput;
	float p;
	float i;
	float d;
};

#endif /* PID_H_ */
