/**
* \file Pilot.h
* \brief 
*
*/

#ifndef _PILOT_H_
#define _PILOT_H_

#include "declarationsGlobales.h"
#include "Actuators.h"
#include "GroundLink.h"
#include "Navigator.h"
#include "ParameterManager.h"

// Holds the RC inputs [-1, 1].
struct RcInputs{

  float rollInput;
  float pitchInput;
  float yawInput;
  float operatingModeInput;
  float altitudeModeInput;
};

class Pilot{
public :
  Pilot(mavion_version mavionVersion);
  ~Pilot();
  void control();
  mavion_operatingMode getOperatingMode();
  
  void introduceComponent(Actuators* a);
  void introduceComponent(GroundLink* a);
  void introduceComponent(PhysicalState* a);
  void introduceComponent(Navigator* a);
  void introduceComponent(ParameterManager* p);
 
  TypeControl controlData;        // Holds data used for vehicle control, e.g. desired orientation. FIXME Should not be public, architectural flaw.
    TypeCommandes commandData;      // Holds the calculated outputs. FIXME Just public for debugging.
  static Pilot* publicInstance;   // FIXME just here to enable GroundLink ot access the desired orientation - should be removed as soon as possible by finding a better way to do this.
private:
  void calculateOutput_KillAllLaw();
  void calculateOutput_AutoTakeoff();
  void calculateOutput_GyroLaw();
  void calculateOutput_OrientationGuidedLaw();
  void calculateOutput_AutomaticLaw();
  
  void navigate();
  
  void readRCInputs();
  void figureOutOperatingMode();
  void controlActuators();
  void initializeControlParameters();
  void publishMode();
  
  Actuators* actuators;   // The Pilot uses this object as an interface to control servos, motors etc. 
  bool hasActuators;
  GroundLink* groundlink; // The Pilot gets ground control station commands and the pilot's inputs from a GroundLink.
  bool hasGroundLink;
  PhysicalState* uavState;  // To control the UAV, the Pilot has to know its current state.
  bool hasUAVState;
  Navigator* navigator;     // A navigator objects manages the mission (waypoints, safety zones etc.)
  bool hasNavigator;
  ParameterManager* parameterManager;   //A Pilot uses a ParameterManager to make its control parameters (PID gains etc.) accessible to QGroundControl.
  bool hasParameterManager;
  
  static float pwm2float(UINT16 pwm);
  static UINT16 float2pwm(float flt);
  mavion_operatingMode currentOperatingMode;    // The operating mode of the UAV (manual, guided, automatic etc.)
  mavion_operatingMode lastOperatingMode;      // The operating mode of the timestep before the current one. Used to allow laws to detect mode switches.
  
  static const UINT16 hoverTrimspeed = 140;            //Estimated motor rotation speed that is required to maintain stationary hover flight  [rev/s].
  mavion_version mavionVersion;                 // Holds the version of the MAVion UAV this Pilot is supposed to control. The Pilot uses
                                                // different default control gains depending on the version.
                                                // FIXME Just a workaround since sending parameter files with QGroundControl does not work.
};

#endif  // _PILOT_H_
