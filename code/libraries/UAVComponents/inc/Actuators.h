/**
* \file Actuators.h
* \brief  An Actuators object provides a simple interface to control actuators like servos and motors.
*
*/

#ifndef _ACTUATORS_H_
#define _ACTUATORS_H_

#include "AT91_PWM.h"
#include "declarationsGlobales.h"
#include "type.h"
#include "Variateur.h"
#include "Sensors.h"
#include "Mixer.h"

struct actuatorControlParameters{
  float rev2Pwm; // Maps a desired rotational motor speed to a PWM command. 
};

class Actuators{
public :
  Actuators(mavion_version config);
  ~Actuators();
  void setPWM(UINT8 channel, UINT16 value);
  void setMotorSpeed(UINT8 motorID, float speed);
  float getMotorSpeed(UINT8 motorID);
  
  void control();
  void introduceComponent(Sensors* s);
  static const UINT8 numberOfPWMChannels = 5;
  float desiredRotationalSpeed[2];            // FIXME Just public for debugging.
  float actualRotationalSpeed[2];            // FIXME Just public for debugging.
  INT32 motorCommands[2];                   // FIXME Just public for debugging.
  INT32 pwm[numberOfPWMChannels];                   // FIXME Just public for debugging.
  void introduceComponent(ParameterManager* p);
private:
    typedef enum {
    MOTOR_SPINUP_PHASE,
    MOTOR_IS_RUNNING
  }motorControlStates;
  Mixer mixer;                                  // Allows to manage different PWM channel mix configurations (e.g. for some UAVs the servos might be connected to channels 2 and 3 instead of 0 and 1).
  PWM* pwmOutputs[numberOfPWMChannels];         // PWM drivers, each one controls one channel.
  Variateur* speedControllers[2];               // I2C speed controlers (mikrokopter BLCtrl, V. 2)
  bool motorIsRunning[2];                       // Indicates wether the motor has reached a certain rotational speed at least once.
  Sensors* sensors;                             // Actuators uses this object to get the battery voltage for motor control.
  bool hasSensors;                              // Indicates wether we have a pointer to a valid Sensors object yet.
  ParameterManager* parameterManager;            // An Actuators object uses a ParameterManager to make its control parameters (e.g. motor closed loop control gains etc.) accessible to the GCS.
  bool hasParameterManager;
  struct actuatorControlParameters gains;
};

#endif  // _ACTUATORS_H_
