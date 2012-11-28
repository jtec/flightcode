/**
 * \file alStandaloneNode.h
 */

#ifndef ALSTANDALONENODE_H_
#define ALSTANDALONENODE_H_


#include "alStopwatch.h"
#include "alBus.h"
#include "alMAVLink.h"
#include "GroundLink.h"
#include "ParameterManager.h"
#include "../../convenienceLayer/inc/clFactory.h"
#include "../../convenienceLayer/inc/clTimebase.h"

struct controlOutputQuadrocopter{
	float throttle;
	float nick;
	float roll;
	float yaw;
};

struct controlGains{
	float roll_P;
	float roll_I;
	float roll_D;
	float pitch_P;
	float pitch_I;
	float pitch_D;
	float yaw_P;
	float yaw_I;
	float yaw_D;

};

class alStandaloneNode{
public :
	alStandaloneNode();
	~alStandaloneNode();
	void runloop();
private:
	void estimateAttitude(mpu6050Output* measurements, float timestep, attitudeEuler* euler, attitudeEuler* eulerLowPAss, float* bias);
	void calculateControlOutput(attitudeEuler* attitude, attitudeEuler* attitudeLowPass, attitudeEuler* desiredAttitude, controlOutputQuadrocopter* controlOutputs, float dt);
	void output2MotorCommands(controlOutputQuadrocopter* controlOutputs, float* motorOutputs);
	float rad2deg(float rad);
	alBus* bus;
	alMAVLink* mavlinkBus;
	alMAVLink* mavlinkXBee;
	GroundLink* groundLink;
	ParameterManager* parameterGuy;
	float receivedServoCommands[10];
	bool RCRequestHasBeenServed;
	clHVD78* RS485Transceiver;
	clLED* LED;
	clMPU6050* imu;
	clSpektrumSatellite* rcReceiver;
	clPWMServoBank* servos;
	controlGains gains;
};

#endif /* ALSTANDALONENODE_H_ */
