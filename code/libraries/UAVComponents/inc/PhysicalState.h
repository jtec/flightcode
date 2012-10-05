/**
* \file PhysicalState.h
* \brief 
*
*/

#ifndef _PHYSICALSTATE_H_
#define _PHYSICALSTATE_H_

#include "Sensors.h"
#include "Navigator.h"
#include "declarationsGlobales.h"


class PhysicalState{
public :
  PhysicalState();
  ~PhysicalState();
  void introduceComponent(Sensors* s);
  void introduceComponent(Navigator* n);
 
  void tick();
  void updateState();
  TypeEtat state;             // This structure holds the uav's physical state (attitude, position etc.)
  bool isStillInitializing(); 
  void startFakingGPS();  
  void stopFakingGPS();  
private:
  void dofakeGPS();
  Sensors* sensors;           // Object to access sensors like imu, gps etc. HAs to be provided to PhysicalState using its introduceComponent(Sensors* s) method.
  bool hasSensors;            // Indicates wether this Physical state knows a Sensors object it can get sensor data from. Used to avoid accessing
                              // 'sensors' if the pointer does not point to a valid Sensors object (provided by the 'introduceComponent(Sensors* s)' method)
  void initializeState();     // Sets the state structure to their start values.
  bool isStillInitialising;   // Indicates wether a PhysicalState object is still initializing or if it is ready to be used.
  void integrateAccelerations();
  Navigator* navigator;       // TODO Just here because we need to access some data in the navigators data structure to update our own state.
  bool hasNavigator;          // Indicates wether this Physical state has got a reference to a Navigator object.         
  bool fakeGPS;               // Indicates wether the position should be taken from the GPS receiver
                              // or integrated RC inputs.
};

#endif  // _PHYSICALSTATE_H_
