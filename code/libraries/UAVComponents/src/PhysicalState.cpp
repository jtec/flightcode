/**
 * \file PhysicalState.cpp
 * \brief
 *
 */

#include "loisCommandes.h"
#include "navigation.h"
#include "PhysicalState.h"
#include "Pilot.h"
#include "TimeBase.h"
#include "outils.h"
#include "math.h"

/**
 * \brief Constructor, returns an instance of PhysicalState.
 */
PhysicalState::PhysicalState()
{
	this->hasSensors = false;
	this->hasNavigator= false;
	this->initializeState();
	this->isStillInitialising = true;
	this->fakeGPS = false;
}

/**
 * \brief Destructor.
 */
PhysicalState::~PhysicalState()
{

}

/**
 * \brief Tells the component where to get sensor data.
 */
void PhysicalState::introduceComponent(Sensors* s){
	this->sensors = s;
	this->hasSensors = true;

}

/**
 * \brief Reads Sensors and updates the UAVs physical state (attitude, position etc.).
 */
void PhysicalState::updateState(){
	static UINT16 sampleCounter = 0;
	static float initSamples = 100.0;
	static float sumAccz[3] = {0};

	if(this->hasSensors){

		RestituerSBG(this->sensors->SBGData, &this->state);
		RestituerGPS(this->sensors->GPSData, &this->state.GPS, this->navigator->navigationData);
		this->state.voltage = this->sensors->battery.getVoltage();
		quaternion_to_Euler(this->state.quat,this->state.euler);

		// TODO We need a filter here for altitude and climb rate.
		this->state.zd = 0;
		this->state.z = 0;

		// Integrate accelerations to get velocity estimate.
		if(!this->isStillInitialising){
			float dt = 0.01;

			this->state.relativeVelocityEstimate[0] += this->state.accInert[0]*dt;
			this->state.relativeVelocityEstimate[1] += this->state.accInert[1]*dt;
			// Subtract gravitation for z axis:
			float accZ = this->state.accInert[2] + this->state.gravHome;
			this->state.relativeVelocityEstimate[2] += accZ*dt;
			// Integrate velocities to get position estimate:
			for (int i=0; i<3; i++){
				this->state.relativePositionEstimate[i] += this->state.relativeVelocityEstimate[i]*dt;
			}
		}

	if(sampleCounter < initSamples){
		// Sample local gravity
		float sample = sqrt(this->state.accInert[0]*this->state.accInert[0] + this->state.accInert[1]*this->state.accInert[1] + this->state.accInert[2]*this->state.accInert[2]) / initSamples;
		this->state.gravHome += sample;
		sampleCounter += 1;
	}else{
		// Switch to normal mode.
		this->isStillInitialising = false;
	}

	// If we are supposed to do so, replace the GPS sensor output by a fake position
	// derived from RC inputs. Quite useful for debugging vehicle navigation indoors.
	if(this->fakeGPS){
		this->dofakeGPS();
	}
}
}

/**
 * \brief Integrates RC inputs to a GPS position to enable the pilot to set the UAVs position.
 * May be used to test navigiation code without GPS.
 */
void PhysicalState::dofakeGPS(){
	static UINT32 timeOfLastCall = 0; // System time at which this function has been called last.
	static float initialLat = 43.5650760;
	static float initialLOn = 1.4770657;
	static float fakeLat = initialLat;
	static float fakeLon = initialLOn;
	static float scale = 0.001;

	float latInput =  Pilot::publicInstance->controlData.x_command;
	float lonInput = - Pilot::publicInstance->controlData.heading_rate;
	// Eliminate drift caused by RC input jitter.
	if(fabs(latInput + lonInput) > 0.01){
		float dT = (TimeBase::getSystemTime_ms() - timeOfLastCall)/1000.0;
		float latIncrement = (latInput * dT) * scale;
		float lonIncrement = (lonInput * dT) * scale;
		// Integrate input to new position
		fakeLat = fakeLat + latIncrement;
		fakeLon = fakeLon + lonIncrement;
	}
	// Write fake position into GPS data structure.
	this->state.GPS.longitude = fakeLon;
	this->state.GPS.latitude = fakeLat;
	this->state.GPS.NED[0] = (fakeLat - this->navigator->navigationData.latHome)*LATTOMETERS;
	this->state.GPS.vitesseNS = 0.0;
	this->state.GPS.vitesseEW = 0.0;
	this->state.GPS.vitesseSol = 0.0;
	this->state.GPS.altitudeMSL = 150.0;
	this->state.GPS.nbSat = 6;  // Fake a normal number of satellites in view.
	this->state.GPS.NED[0] = (fakeLat - this->navigator->navigationData.latHome)*LATTOMETERS;
	this->state.GPS.NED[1] = (fakeLon - this->navigator->navigationData.lonHome)*LONTOMETERS;

	timeOfLastCall = TimeBase::getSystemTime_ms();
	return;
}

/**
 * \brief Does whatever has to be done in a more frequent manner.
 */
void PhysicalState::tick(){
	// Right now, there is nothing to be done here.
}

/**
 * \brief Return value indicates wether this PhysicalState is still initializing (e.g. sampling sensor biases)
 *        or if it is ready to be used.
 * \return true or false
 */
bool PhysicalState::isStillInitializing(){
	return this->isStillInitialising;
}

/**
 * \ Sets the state structure to their start values
 */
void PhysicalState::initializeState(){
	// initialize state:
	//this->state.inclination = (59.0+7.0/60.0)*DEGTORAD;
	//this->state.declination = (-18.0/60.0)*DEGTORAD;
	// From the gains file used for IMAV 2011 in Delft:
	this->state.inclination = 1.03;
	this->state.declination = -0.005236;
	this->state.z = 0.0;
	this->state.zd = 0.0;
	this->state.zdd = 0.0;
	this->state.gravHome = 0.;

	for(int i=0;i<3;i++){
		this->state.accBody[i] = 0;
		this->state.accInert[i] = 0;
		this->state.relativePositionEstimate[i] = 0;
		this->state.relativeVelocityEstimate[i] = 0;
		this->state.omega[i] = 0;
		this->state.omegaf[i] = this->state.omega[i];
		this->state.correctionI[i] = 0.0;
		this->state.mag0[i] = 0.0;
		this->state.vec100_iner[i] = 0.0;
		this->state.euler[i] = 0;
		this->state.quat[i] = 0;	// Zero rotation
	}
	this->state.quat[3] = 1.0;	// Zero rotation

	for(int i=0;i<4;i++){
		this->state.quatf[i]=this->state.quat[i];
	}

	// Init GPS position: Toulouse, Campus ISAE.
	this->state.GPS.longitude = 1.477;
	this->state.GPS.latitude = 43.565;
	this->state.GPS.altitudeMSL = 150.0;
	this->state.mode = 0;
}

/**
 * \brief Just a workaround to preserve old data distribution, PhysicalState should
 * not have to know a Navigator, since it just provides data, so there is no need for collaboration.
 */
void PhysicalState::introduceComponent(Navigator* n){
	this->navigator = n;
	this->hasNavigator = true;
}

/**
 * \brief Makes this PhysicalState stop listening to the GPS Sensor and derive the GPS
 * position from two RC input channels. THis way the pilot can set the UAVs position for
 * testing the navigation algorithms on the desk rather than outside.
 */
void PhysicalState::startFakingGPS(){
	this->fakeGPS = true;
}

/**
 * \brief Makes this PhysicalState stop deriving the GPS
 * position from RC input channels but instead take the output from the GPS receiver.
 */
void PhysicalState::stopFakingGPS(){
	this->fakeGPS = false;
}
