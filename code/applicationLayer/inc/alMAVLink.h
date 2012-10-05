/**
* \file alMAVLink.h
* \brief This class may be used to exchange alMAVLink messages via a serial port.
*        It can be configured to
*        a) forward every message to another alMAVLink
*        b) call a handler function each time e new message is received and pass
*           it a pointer to this message.
*/

#ifndef _MAVLINK_H_
#define _MAVLINK_H_

#include "../../convenienceLayer/inc/clSerialPort.h"
#include "../../libraries/mavlink/include/mavlink/v1.0/common/common.h"
#include "../../libraries/mavlink/include/mavlink/v1.0/jtec/jtecMavlinkMessages/jtecMavlinkMessages.h"

/**
* Attitude angles (euler angles )and rotation rates in the body frame.
*/
struct attitudeEuler{
  float euler[3];               // Roll, Pitch, Yaw [radians
  float bodyRotationRate[3];    // Roll, Pitch, Yaw rates [radians/s]
};

/**
* Attitude as quaternion and rotation rates in the body frame.
*/
struct attitudeQuaternion{
  float w;  // Real component.
  float x;  // First imaginary component / x.
  float y;  // Second imaginary component / y.
  float z;  // Third imaginary component / z.
  float bodyRotationRate[3];    // Roll, Pitch, Yaw rates [radians/s]
};

/**
* Inertial and magnetic field measurements (in the body frame).
*/
struct measurementsInertial{
	float acc[3];     		// Accelerations [m/s²]
	float gyro[3];    		// Rotation rates [radians/s]
	float magField[3];    	// Magnetic field [mTesla]
};

class alMAVLink{
public :
  alMAVLink(clSerialPort* serialPort );
  ~alMAVLink();
  void tick();	  // Call this function as least as fast as bytes can arrive on the serial connection..
  void setPort(clSerialPort * port);
  void startForwardingMessages(alMAVLink* target);
  void stopForwardingMessages();
  void startCallingMessageHandler(void (*handler)(mavlink_message_t*));
  void stopCallingMessageHandler();
  void sendRawData(uint16_t length, uint8_t* buf);
  void sendAttitude(attitudeEuler* a);
  void sendAttitude(attitudeQuaternion* q);
  void sendAttitude(measurementsInertial* a);
  void sendHeartbeat(uint8_t systemState);
  void sendPWMCommands(uint16_t* pulseWidths, uint8_t targetID);
  void sendFloatCommands(float* commands, uint8_t targetID);
  void sendActuatorCommands(float* commands);
  void sendInt32(int32_t value, char* name);
  void sendFloat(float value, char* name);
  void sendRequestServoCommands(uint8_t targetID);
  void sendMessage(mavlink_message_t* msg);
  void sendFloatParameter(float value, uint16_t totalNumberOfParameters, uint16_t indexOfParameter, char* paramID, uint8_t parameterType);
  bool isTransmitting();
  static void initStructure(attitudeEuler* s);
  static void initStructure(attitudeQuaternion* s);
  static void initStructure(measurementsInertial* s);
  static void pwm2float(int32_t n, uint16_t* pwm, float* flt);
  static void float2pwm(uint16_t n, float* flt, uint16_t* pwm);
  uint32_t getTimeSinceLastHeartBeat();
  uint32_t numberOfReceivedHeartbeats;
  mavlink_system_t mavlink_system;
  bool isMySystemID(uint8_t idToCheck);

private:
  void checkForNewBytes();
  alMAVLink* forwardTo;
  clSerialPort* mySerialPort;
  uint8_t systemID;
  bool forwardEveryMessage;
  bool callMessageHandler;
  void (*messageHandler)(mavlink_message_t*);
  uint32_t timeOfLastHeartBeat;
};

#endif  // _MAVLINK_H_
