/**
* \file Sensors.cpp
* \brief Controls and reads all the sensors on board a UAV, as ultrasound, IMU etc.
*
*/

#include "Sensors.h"
#include "loisCommandes.h"


/**
* \brief Constructor, returns an instance of Sensors.
* \param[in] serialPortIMU - Pointer to a LiaisonSerie used for communications with the SBG IMU
* \param[in] serialPortGPS - Pointer to a LiaisonSerie used for communications with a ublox LEA-5x/-6x GPS
*/
Sensors::Sensors(LiaisonSerie* serialPortIMU, LiaisonSerie* serialPortGPS)
:battery(ADC1_7), pressureSensor(0x78)
{
  // Initialize IMU related stuff:
  this->imu = new VN100(serialPortIMU);
  this->gps.initPS(serialPortGPS);
  this->SBGType =  SBG_OUTPUT_QUATERNION | SBG_OUTPUT_ACCELEROMETERS | SBG_OUTPUT_GYROSCOPES|SBG_OUTPUT_TEMPERATURES|SBG_OUTPUT_BARO_PRESSURE;   
  
  this->SBGData.gyroscopes[0] = 0;
  this->SBGData.gyroscopes[1] = 0;
  this->SBGData.gyroscopes[2] = 0;
  
  this->barometricPressure = 0;
  
  // TODO Initialize GPSData  and SBGData structures
  
}

/**
* \brief Destructor.
*/
Sensors::~Sensors()
{
}

/**
* \brief Makes this component check for new data from the sensors it reads and update its state.
*/
void Sensors::tick(){
	static struct VN100Output VN100Data;
  // Check fo new data received from the IMU:
  this->imu->tick();
  if(this->imu->areNewDataAvailable()){
	  this->imu->read(&VN100Data);
	  // Copy to local structure:
	  // TODO Use generic structure instead of that of the SBG device driver.
	  for(int i=0; i<3; i++){
		  this->SBGData.accelerometers[i] = VN100Data.accelerations[i];
		  this->SBGData.gyroscopes[i] = VN100Data.angularRates[i];
	  }
	  this->SBGData.baroPressure = VN100Data.pressure;
	  this->SBGData.stateQuat[0] = VN100Data.attitude.w;
	  this->SBGData.stateQuat[1] = VN100Data.attitude.x;
	  this->SBGData.stateQuat[2] = VN100Data.attitude.y;
	  this->SBGData.stateQuat[3] = VN100Data.attitude.z;
  }
  // Check fo new data received from the GPS receiver:
  this->gps.TraiterTrame();
  // FIXME Has Gps::TraiterGPS to be called?
  // If new data GPS are available: copy them to local structure.
  if (gps.infosPOSLLHDispo())
  {
    gps.getInfosPOSLLH_faster(&this->GPSData.GPSInfosLLH);
  } 
  if (gps.infosVELNEDDispo())
  {
    gps.getInfosVELNED_faster(&this->GPSData.GPSInfosVelned);
  }
  if (gps.infosSOLDispo())
  {
    
    gps.getInfosSOL_faster(&this->GPSData.GPSInfosSol);
  }
  
  // Make the battery check for new completed ADC conversions.
  this->battery.tick();
}

/*
* \brief Read the sensor values that take almost no time for communication (e.g. read ADC output register).
* TODO Define "almost no time".
*/
void Sensors::read(){
}
