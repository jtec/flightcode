/**
* \file MAVLinkMAVion.h
* \brief This class may be used to exchange MAVLink messages via a serial port.
*        It can be configured to
*        a) forward every message to another MAVLink
*        b) call a handler function each time e new message is received and pass
*           it a pointer to this message.
*
* TODO Decide for one type system (UINT8 or uint8_t) and be consistent about it.
*/

#ifndef _MAVLINK_MAVION_H_
#define _MAVLINK_MAVION_H_

#include "type.h" 
#include "liaisonSerie.h"
#include "common.h"
#include "MavlinkMessageBuffer.h"
#include "Periode.h"



/**
* Attitude angles (euler angles )and rotation rates in the body frame.
*/
struct mavlink_attitudeEuler{
  float euler[3];               // Roll, Pitch, Yaw [radians
  float bodyRotationRate[3];    // Roll, Pitch, Yaw rates [radians/s]
};

/**
* Attitude quaternion and rotation rates in the body frame.
*/
struct mavlink_attitudeQuaternion{
  float w;  // Real component.
  float x;  // First imaginary component / x.
  float y;  // Second imaginary component / y.
  float z;  // Third imaginary component / z.
  float bodyRotationRate[3];    // Roll, Pitch, Yaw rates [radians/s]
};

/**
* Position and groundspeed. 
* TODO Would be more convenient if the user could simply provide the data as floats,
* check if float resolution fits our requirements (~10cm, better for DGPS).
*/
struct mavlink_positionAndSpeed{
  INT32 lat;                    // lat[10^7 degrees], WGS84
  INT32 lon;                    // lon[10^7 degrees], WGS84
  INT32 alt;                    // altitude above Mean Sea Level[10^-3 m], WGS84
  INT32 altAboveGround;         // altitude above Ground [10^-3 m]
  INT16 groundspeedLat;         // ground speed in latitude direction[10^2 m/s]
  INT16 groundspeedLon;         // ground speed in longitude direction [10^2 m/s]
  INT16 groundspeedVertical;    // vertical ground speed [10^2 m/s]
};

/**
* Position and groundspeed. 
* TODO Would be more convenient if the user could simply provide the data as floats,
* check if float resolution fits our requirements (~10cm, better for DGPS).
*/
struct mavlink_GPSOutput{
  float lat;                    // lat[degrees], WGS84
  float lon;                    // lon[degrees], WGS84
  float alt;                    // altitude above Mean Sea Level[m], WGS84
  float groundspeed;            // GPS ground speed [m/s]
  float hdop;                   // GPS HDOP [m]
  float vdop;                   // GPS VDOP [m]
  float course;                 // trajectory azimuth, direction of movent [degrees].
  UINT16 svInView;              // number of satellites in view [1].
  UINT8 fix;                    // 0: no fix; 2: 2D-fix; 3: 3D-fix  
};

class MAVLinkMAVion{
public :
  // Constructor. Note: a MAVLink instance does nothing until it is configured to do so 
  // by one of the methods below.
  MAVLinkMAVion(LiaisonSerie* serialPort );
  // Destructor.
  ~MAVLinkMAVion();
  // Call this function as least as fast as bytes can arrive at the serial connection..
  void checkForNewBytes();
  // Use this function to provide a MAVLink object with a serial port to use for communication.
  void setPort(LiaisonSerie * port);
  // Defines where the MAVLink forwards messages to. Before this method is called, 
  // no message is forwarded.
  void startForwardingMessages(MAVLinkMAVion* target);
  void stopForwardingMessages();
  // Defines where the MAVLink decodes messages. Before this method is called, 
  // no message is decoded.
  void startCallingMessageHandler(void (*handler)(mavlink_message_t*));
  void stopCallingMessageHandler();
  bool isMySystemID(UINT8 idToCheck);
  
  void sendRawData(UINT16 length, UINT8* buf);
  void sendAttitude(mavlink_attitudeEuler* a);
  void sendAttitude(mavlink_attitudeQuaternion* q);
  void sendGlobalPositionAndSpeed(mavlink_positionAndSpeed* s, float heading);
  void sendGPSPositionAndSpeed(mavlink_GPSOutput* s);
  void sendHeartbeat(uint8_t systemState, uint8_t systemMode);
  void sendPWMCommands(UINT16* pulseWidths, UINT8 targetSystemID);
  void sendSystemStatus(float batteryVoltage);
  void sendInt32(INT32 value, char* name);
  void sendFloat(float value, char* name);
  void sendStatusText(UINT8 severity, char* text);
  void sendMissionCount(UINT16 count);
  void sendWaypoint(float lat, float lon, float alt, float hoverTime, UINT16 command, UINT16 numberOfWaypoint);
  void sendWhatIsCurrentWaypoint(UINT16 numberOfCurrentWaypoint);
  void sendWaypointRequest(UINT16 numberOfWaypoint);
  void sendWaypointAck(bool transactionOK);
  void sendFloatParameter(float value, UINT16 totalNumberOfParameters, UINT16 indexOfParameter, char* paramID, UINT8 parameterType);
  void openTxWindow(UINT16 time);

  static void initStructure(mavlink_attitudeEuler* s);
  static void initStructure(mavlink_attitudeQuaternion* s);

  UINT32 getTimeSinceLastHeartBeat();
  UINT32 numberOfReceivedHeartbeats;
  LiaisonSerie* getSerialPort();
  bool isBusyTransmitting();  // Return value indicates wether the MAVLink's serial port is done transmitting.
  mavlink_system_t mavlink_system;  // Identifies the UAV according to the myvlink protocol. TODO Should be private.
  void sendMessage(mavlink_message_t* msg);
private:
  // Used by the sendXXX([...]) methods to turn mavlink message strcutures in a stream of bytes.
  mavlink_message_t bufferMessage;
  // The MAVLink forwards all messages to this MAVLink.
  MAVLinkMAVion* forwardTo;
  // The LiaisonSerie (serial port) this MAVLink uses for data transfer.
  LiaisonSerie * mySerialPort;
  //Indicates wether each incoming message is supposed to be forwarded to another MAVLink.
  bool forwardEveryMessage;
  // Indicates wether this MAVLink is supposed to call a handler function each time it receives
  // a valid mavlink message.
  bool callMessageHandler;
  // Pointer to the function that is called each time a valid message is received if the MAVLink is configured to do so.
  void (*messageHandler)(mavlink_message_t*);
  // Indicates how much time has passed since the last heartbeat message has been received by this MAVLink.
  UINT32 timeOfLastHeartBeat;

  mavlink_system_t gcsTarget_mavlink_system;
  Periode messageTransmissionTimer;
};

#endif  // _MAVLINK_MAVION_H_
