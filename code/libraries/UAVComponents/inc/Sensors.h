/**
* \file Sensors.h
* \brief 
*
*/

#ifndef _SENSORS_H_
#define _SENSORS_H_

#include "../../../rollFlyLibrary/VN100.h"
#include "SBG.h"
#include "GPS.h"
#include "CapteurPS.h"
#include "systemConfig.h"
#include "LiaisonSerie.h"  
#include "Battery.h"
#include "declarationsGlobales.h"
#include "Navigator.h"

class Sensors{
public :
  Sensors(LiaisonSerie* serialPortIMU, LiaisonSerie* serialPortGPS);
  ~Sensors();
  void tick();
  void read();

  SbgOutput SBGData;                // TODO Should not be public, check if performance allows copying to access data.
  TypeInfosGps GPSData;             // TODO Should not be public, check if performance allows copying to access data.
  Battery battery;                  // Onboard main battery. 
  float barometricPressure;                   // current barometric pressure.
private:
  VN100* imu;                   // The IMU, connected to a serial port.
  Gps gps;                          // The ublox GPS receiver, connected to a serial port.
  CapteurPS pressureSensor;         // HCA0811 pressure sensor.
  //LiaisonSerie* serialPortIMU;
  SbgErrorCode lastErrorCodeSBG;    // When read, the IMU driver returns an error code, this structure is used to store it.
  UINT32 SBGType;                   // USed to define the IMU'S output (quaternions, euler angles, raw data...) 

};


#endif  // _SENSORS_H_
