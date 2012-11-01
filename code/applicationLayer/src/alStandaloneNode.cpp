/*
 * \file alStandaloneNode.cpp
 * \date Created on: Sept 21, 2012
 * \author: Jan
 *
 * \brief  An alStandaloneNode works as a single-board autopilot, it is used for testing
 * with a minimum of hardware and without the additional risk of a bus failure.
 */

#include "../inc/alStandaloneNode.h"
#include "../inc/alBus.h"
#include "../inc/PID.h"
#include "math.h"

/**
 * \brief Constructor.
 */
alStandaloneNode::alStandaloneNode(){
	clFactory* factory = clFactory::buildForMainNode();

	// Create the two communication links (Bus, GCS)
	clSerialPort* port = factory->getSerialPortBus();
	this->mavlinkBus = new alMAVLink(port);
	port = factory->getSerialPortXBee();
	this->mavlinkXBee = new alMAVLink(port);
	this->groundLink = new GroundLink(port);
	this->bus = new alBus(this->mavlinkBus, mainNode);

	this->LED = factory->getStatusLed();
	this->imu = factory->getMPU6050();
	this->rcReceiver = factory->getSpektrumSatellite();
	this->servos = factory->getPWMServoBank();

	this->RS485Transceiver = factory->getRS485Transceiver();
	// Switch receiver on - not to miss incoming messages - and disable
	// the driver not to block the bus.
	factory->getRS485Transceiver()->disableDriver();
	factory->getRS485Transceiver()->enableReceiver();

	this->parameterGuy = new ParameterManager();
	this->groundLink->introduceComponent(this->parameterGuy);

	// Initialize control gains:
	this->gains.roll_P = 0;
	this->gains.roll_I = 0;
	this->gains.roll_D = 0;
	this->gains.pitch_P = 0;
	this->gains.pitch_I = 0;
	this->gains.pitch_D = 0;
	this->gains.yaw_P = 0;
	this->gains.yaw_I = 0;
	this->gains.yaw_D = 0;
}

alStandaloneNode::~alStandaloneNode(){
}

/**
 * \brief This method goes into an infinite loop and periodically
 * 1) Requests information from the sensors
 * 2) Calculates the current attitude
 * 3) Calculates the desired attitude from RC inputs
 * 4) Calculates the control answer
 * 5) Sends the control commands to the servos
 * 6) Send all data of interest to the ground control station.
 */
void alStandaloneNode::runloop(){
	// Create and initialize all the variables and objects we need here:
	alStopwatch*  controlTimer = new alStopwatch();
	controlTimer->restart();
	alStopwatch*  LEDTimer = new alStopwatch();
	LEDTimer->restart();
	alStopwatch*  RFTimer = new alStopwatch();
	RFTimer->restart();

	bool test = false;
	struct mpu6050Output imuData;
	struct attitudeEuler euler;
	struct attitudeEuler eulerDesired;
	clMPU6050::initStructure(&imuData);
	alMAVLink::initStructure(&euler);
	alMAVLink::initStructure(&eulerDesired);
	float rcInputs[8] = {0};
	float motorOutputs[8] = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0,};
	struct controlOutputQuadrocopter controlOutput;
	float bias = 0;
	bool isLocked = true;

	this->parameterGuy->checkInFloat(&this->gains.pitch_P, "ctrl_pitch_p", 12);
	this->parameterGuy->checkInFloat(&this->gains.pitch_P, "ctrl_pitch_i", 12);
	this->parameterGuy->checkInFloat(&this->gains.pitch_P, "ctrl_pitch_d", 12);
	this->parameterGuy->checkInFloat(&this->gains.pitch_P, "ctrl_roll_p", 11);
	this->parameterGuy->checkInFloat(&this->gains.pitch_P, "ctrl_roll_i", 11);
	this->parameterGuy->checkInFloat(&this->gains.pitch_P, "ctrl_roll_d", 11);
	this->parameterGuy->checkInFloat(&this->gains.pitch_P, "ctrl_yaw_p", 10);
	this->parameterGuy->checkInFloat(&this->gains.pitch_P, "ctrl_yaw_i", 10);
	this->parameterGuy->checkInFloat(&this->gains.pitch_P, "ctrl_yaw_d", 10);


	while(true){
		// Keep ground link running:
		this->groundLink->tick();
		// Do all the control stuff:
		const float timestep = 100;	// [ms]
		if(controlTimer->getTime() >= timestep){
			// Don't forget to restart timer:
			controlTimer->restart();
			// Get RC inputs:
			this->rcReceiver->getRcChannels(rcInputs);
			// Get IMU data:
			this->imu->getRawMeasurements(&imuData);
			// Compute attitude:
			this->estimateAttitude(&imuData, timestep, &euler, &bias);
			// Get desired body rotation rates from RC inputs:
			// Roll rate from aileron command:
			eulerDesired.bodyRotationRate[0] = 60 * rcInputs[1];
			// Pitch rate from elevator command:
			eulerDesired.bodyRotationRate[1] = 60 * rcInputs[2];
			// Yaw rate from rudder command:
			eulerDesired.bodyRotationRate[2] = 60 * rcInputs[3];
			// Calculate control reaction:
			this->calculateControlOutput(&euler, &eulerDesired, &controlOutput, timestep);
			// Get desired throttle directly fromRC input:
			controlOutput.throttle = rcInputs[0];
			// Apply control output to the 4 motors:
			this->output2MotorCommands(&controlOutput, motorOutputs);
			// Check if emergency switch is in on or off position:
			isLocked = rcInputs[4] > 0.0;
			// If switch indicates "off", shut down your engines:
			if(isLocked){
				for(int i=0; i<8; i++){
					motorOutputs[i] = - 1.0;
				}
			}
			// Update PWM
			this->servos->setPositions(motorOutputs);
		}

		//Send data to the GCS.
		if(RFTimer->getTime() > 50){
			RFTimer->restart();
				this->mavlinkXBee->sendAttitude(&euler);
				this->mavlinkXBee->sendFloatCommands(rcInputs, 0);
				// Send control output.
				this->mavlinkXBee->sendFloat(controlOutput.nick, "nickC");
				this->mavlinkXBee->sendFloat(controlOutput.roll, "rollC");
				this->mavlinkXBee->sendFloat(controlOutput.yaw, "yawC");
				this->mavlinkXBee->sendFloat(controlOutput.throttle, "throttleC");
			}

		// Make LED blink:
		if(LEDTimer->getTime() >= 100){
			this->LED->toggle();
			LEDTimer->restart();
		}
	}
}

/**
 * This method takes inertial measurements and estimates the current attitude.
 * Estimation algorithm:
 * For the moment the body rotation rates are simply integrated to see what happens.
 *
 * \param[in] measurements - Pointer to a structure holding the newset measurements.
 * \param[in] timestep - time [ms] between this call and the last call of this method, e.g.
 * needed to do integrations.
 * param[out] euler - Pointer to a structure the method writes the estimated attitude to.
 */
void alStandaloneNode::estimateAttitude(mpu6050Output* measurements, float timestep, attitudeEuler* euler, float* bias){

		// Copy raw rotation rates.
		euler->bodyRotationRate[0] = measurements->rawGyro[0];
		euler->bodyRotationRate[1] = measurements->rawGyro[1];
		euler->bodyRotationRate[2] = measurements->rawGyro[2];

		// Integrate body rotation rates.
		euler->euler[0] += euler->bodyRotationRate[0] * (timestep/1000);
		euler->euler[1] -= euler->bodyRotationRate[1] * (timestep/1000);
		euler->euler[2] += euler->bodyRotationRate[2] * (timestep/1000);

	// Calculate euler angles from accelerometer data:
	static float pitchAcc = 0;
	static float rollAcc = 0;
	pitchAcc = rad2deg(asin(measurements->rawAcc[0]));
	rollAcc = rad2deg(asin(measurements->rawAcc[1]));
	// Since the arcsin function returns NaN for input values greater or smaller than [-1,1]:
	static float NaNValuePitch = 0;
	static float NaNValueRoll = 0;
	if(isnan(pitchAcc)){
		pitchAcc = NaNValuePitch;
	}else{
		NaNValuePitch = pitchAcc;
	}

	if(isnan(rollAcc)){
		rollAcc = NaNValueRoll;
	}else{
		NaNValueRoll = rollAcc;
	}

	// Combine angles derived from rotation rates and those calculated from accelerations in a complementary manner:
	euler->euler[0] += 0.01 * (rollAcc - euler->euler[0]);
	euler->euler[1] += 0.01 * (pitchAcc - euler->euler[1]);
}



/**
 * \brief This method calculates the control output based on the current actual attitude and the desired attitude.
 */
void alStandaloneNode::calculateControlOutput(attitudeEuler* attitude, attitudeEuler* desiredAttitude, controlOutputQuadrocopter* controlOutputs, float dt){
	static PID rollRatePID(0.001, 0.0, 0.0001, 0.0);
	static PID pitchRatePID(0.001, 0.0, 0.0001, 0.0);
	static PID yawRatePID(0.001, 0., 0.0001, 0.0);

	// Calculate rotation rate errors:
	float errorPitchRate = attitude->bodyRotationRate[1] - desiredAttitude->bodyRotationRate[1];
	float errorRollRate = attitude->bodyRotationRate[0] - desiredAttitude->bodyRotationRate[0];
	float errorYawRate = attitude->bodyRotationRate[2] - desiredAttitude->bodyRotationRate[2];

	controlOutputs->nick = pitchRatePID.getOutput(errorPitchRate, dt);
	controlOutputs->roll= rollRatePID.getOutput(errorRollRate, dt);
	controlOutputs->yaw = yawRatePID.getOutput(errorYawRate, dt);
}

/**
 * This method takes in the control output [-1.0, 1.0] and calculates the actual motor commands [- 1.0, 1.0]
 * Indices:
 * 0= front , turning clockwise
 * 1= right , turning counter-clockwise
 * 2= rear , turning clockwise
 * 3= left , turning counter-clockwise
 */
void alStandaloneNode::output2MotorCommands(controlOutputQuadrocopter* controlOutputs, float* motorOutputs){
	// apply throttle to all motors:
	for(int i=0; i<4; i++){
		motorOutputs[i] = controlOutputs->throttle;
	}
	// Turn roll and nick commands into differential throttle commands:
	// Nick:
	motorOutputs[0] += controlOutputs->nick;
	motorOutputs[2] -= controlOutputs->nick;
	// Roll:
	motorOutputs[1] += controlOutputs->roll;
	motorOutputs[3] -= controlOutputs->roll;
	// Turn yaw command into differential throttle commands:
	// Front and rear:
	motorOutputs[0] += controlOutputs->yaw;
	motorOutputs[2] += controlOutputs->yaw;
	// Left and right:
	motorOutputs[1] -= controlOutputs->yaw;
	motorOutputs[3] -= controlOutputs->yaw;
}

/**
 * Converts an angle in radians to an angle in degrees.
 * \param rad - The angle in radians.
 * \return The angle in degrees.
 */
float alStandaloneNode::rad2deg(float rad){
	float pi = 3.1415;
	float retVal = rad;
	retVal = retVal * 180;
	retVal = retVal/pi;
	return retVal;
}
