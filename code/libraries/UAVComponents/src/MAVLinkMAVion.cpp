/**
* \file MAVLinkMAVion.cpp
* \brief This class may be used to exchange mavlink (protocol version 1.0) messages via a serial port.
*        It can be configured to
*        a) forward every message to another MAVLink
*        b) call a handler function each time a new message has been received and pass
*           it a pointer to this message.
*
*
* TODO Decide for one type system (UINT8 or uint8_t) and be consistent about it.
*/

#include "common.h"
#include "MAVLinkMAVion.h"
#include "type.h"
#include "liaisonSerie.h"
#include "TimeBase.h"



/**
* \brief Constructor, returns an instance of MAVLinkMAVion.
* \param
*/
MAVLinkMAVion::MAVLinkMAVion(LiaisonSerie* serialPort)
{
  this->mySerialPort = serialPort;
  this->numberOfReceivedHeartbeats = 0;
  this->forwardEveryMessage = false;
  this->callMessageHandler = false;
  this->timeOfLastHeartBeat = 0;
  //this->messageTransmissionTimer = Periode(50); //Messages are sent every 50 ms.
  
  // TODO make system configurable
  this->mavlink_system.sysid = 11;
  this->mavlink_system.compid = MAV_COMP_ID_MISSIONPLANNER;
  this->mavlink_system.type = MAV_TYPE_FIXED_WING;
  
  this->gcsTarget_mavlink_system.sysid = 100;  
  this->gcsTarget_mavlink_system.compid = MAV_COMP_ID_ALL;
  this->gcsTarget_mavlink_system.type = MAV_TYPE_GCS;
}

/**
* \brief Destructor.
*/
MAVLinkMAVion::~MAVLinkMAVion()
{
}

/*
* \brief Sends the heartbeat message
* \param[in] system state  - the system's system status as defined by the mavlink common message set.
* \param[in] system state  - the system's base mode as defined by the mavlink common message set.
*/
void MAVLinkMAVion::sendHeartbeat(uint8_t systemState, uint8_t systemMode)
{
  // Define the system type, in this case an airplane
  uint8_t system_type = MAV_TYPE_FIXED_WING;
  uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
  
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  // Pack the message
  mavlink_msg_heartbeat_pack(this->mavlink_system.sysid, 
                             this->mavlink_system.compid, 
                             &this->bufferMessage, 
                             system_type, 
                             autopilot_type, 
                             systemMode, 
                             custom_mode, 
                             systemState);
  
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
  return;
}

/*
* \brief Sends the euler angles and angular rates in the body frame using the MAVLink protocol.
* \param[in] state attitudeEuler structure pointer
*/
void MAVLinkMAVion::sendAttitude(mavlink_attitudeEuler* a)
{ 
  // Pack the message
  UINT32 time = TimeBase::getSystemTime_ms();
  uint16_t length = mavlink_msg_attitude_pack(this->mavlink_system.sysid, 
                                              this->mavlink_system.compid, 
                                              &this->bufferMessage,
                                              time,
                                              a->euler[0],
                                              a->euler[1],
                                              a->euler[2],
                                              a->bodyRotationRate[0],
                                              a->bodyRotationRate[1],
                                              a->bodyRotationRate[2]);
  
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
  return;
}

/*
* \brief Sends the attitude (quaternion) using the MAVLink protocol.
* \param[in] state attitudeEuler structure pointer
*/
void MAVLinkMAVion::sendAttitude(mavlink_attitudeQuaternion* q){
  // Pack the message.
  UINT32 time = TimeBase::getSystemTime_ms();
  uint16_t len = mavlink_msg_attitude_quaternion_pack(this->mavlink_system.sysid, 
                                                      this->mavlink_system.compid, 
                                                      &this->bufferMessage,
                                                      time,
                                                      q->w,
                                                      q->x,
                                                      q->y,
                                                      q->z,
                                                      q->bodyRotationRate[0],
                                                      q->bodyRotationRate[1],
                                                      q->bodyRotationRate[2]);
  
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
  return;
}

/**
* \brief Sends the UAV's position and ground speed estimate in the WGS84 system.
* \param[in] s - Pointer to a structure (declared in MAVLinkMAVion.h) holding the data to be sent.
*/
void MAVLinkMAVion::sendGlobalPositionAndSpeed(mavlink_positionAndSpeed* s, float heading){
  // Pack the message.
  UINT32 time = TimeBase::getSystemTime_ms();
  UINT16 len =  mavlink_msg_global_position_int_pack(this->mavlink_system.sysid, 
                                                    this->mavlink_system.compid,
                                                    &this->bufferMessage,
                                                    time,
                                                    s->lat,
                                                    s->lon,
                                                    s->alt,
                                                    s->altAboveGround,
                                                    s->groundspeedLat, 
                                                    s->groundspeedLon,
                                                    s->groundspeedVertical,
                                                    (UINT16)(heading*100));
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
}

/**
* \brief Sends position and ground speed as indicated by the onboard GPS receiver (WGS84).
* \param[in] s - Pointer to a structure (declared in MAVLinkMAVion.h) holding the data to be sent.
* Uses time since system start for message timestamp.
*/
void MAVLinkMAVion::sendGPSPositionAndSpeed(mavlink_GPSOutput* s){
  // Pack the message.
  UINT32 time = TimeBase::getSystemTime_ms();
  UINT16 len =  mavlink_msg_gps_raw_int_pack(this->mavlink_system.sysid, 
                                                    this->mavlink_system.compid,
                                                    &this->bufferMessage,
                                                    time*1000,
                                                    s->fix,
                                                    (INT32)(s->lat*10000000),
                                                    (INT32)(s->lon*10000000),
                                                    (INT32)(s->alt*1000),
                                                    (UINT16)(s->hdop*100),
                                                    (UINT16)(s->vdop*100), 
                                                    (UINT16)(s->groundspeed*100),
                                                    (UINT16)(s->course*100),
                                                    s->svInView);
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
}

/**
* \brief Sends 8 PWM values.
* \param[in] pulseWidths - array with values to be sent.
* \param[in] targetSystemID - system ID of the system that is supposed to receive this message
                             (in most cases that will be a UAV, unless the PWM values are sent to a GCS for debugging) 
*/
void MAVLinkMAVion::sendPWMCommands(UINT16* pulseWidths, UINT8 targetSystemID){
 // Pack the message.
  UINT32 time = TimeBase::getSystemTime_ms();
  uint16_t len = mavlink_msg_rc_channels_override_pack(this->mavlink_system.sysid, 
                                                       this->mavlink_system.compid, 
                                                       &this->bufferMessage,
                                                       targetSystemID,
                                                       MAV_COMP_ID_ALL,
                                                       pulseWidths[0],
                                                       pulseWidths[1],
                                                       pulseWidths[2],
                                                       pulseWidths[3],
                                                       pulseWidths[4],
                                                       pulseWidths[5],
                                                       pulseWidths[6],
                                                       pulseWidths[7]);
  
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
  return;
}

/**
* \brief Sends the general system status.  
* // TODO Message content not fully implemented yet, just sends battery voltage.
*/
void MAVLinkMAVion::sendSystemStatus(float batteryVoltage){
  // Pack the message
  // TODO Implement all message fields.
  static UINT32 sensorsPresent    = 0xFFFF; // FIXME Just mark those sensors as existing/enabled/healthy that relly
  static UINT32 sensorsEnabled    = 0xFFFF;
  static UINT32 sensorsHealth     = 0xFFFF;
  
  static UINT16 processingLoad    = 1000;
  
  static UINT16 voltageBattery    = 0;
  static UINT16 currentBattery    = 0;  
  static INT8 batteryRemaining    = -1; // -1 means the autopilot does not estimate the battery energy level.
  
  static UINT16 dropRateCom       = 0;
  static UINT16 errorRateCom      = 0;
  static UINT16 errorsCount_1to4  = 0;
  
  voltageBattery = (UINT16)(batteryVoltage*1000.0);
  
  
  uint16_t length = mavlink_msg_sys_status_pack(this->mavlink_system.sysid,
                                                     this->mavlink_system.compid, 
                                                     &this->bufferMessage,
                                                     sensorsPresent,
                                                     sensorsEnabled,
                                                     sensorsHealth,
                                                     processingLoad,
                                                     voltageBattery,
                                                     currentBattery,
                                                     batteryRemaining,
                                                     dropRateCom,
                                                     errorRateCom,
                                                     errorsCount_1to4,
                                                     errorsCount_1to4,
                                                     errorsCount_1to4,
                                                     errorsCount_1to4);
  
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
}

/*
* \brief Sends a named float value. Maximum length of name: 9 characters
*/
void MAVLinkMAVion::sendFloat(float value, char* name){  
  // Pack the message
  UINT32 time = TimeBase::getSystemTime_ms();
  uint16_t length = mavlink_msg_named_value_float_pack(this->mavlink_system.sysid,
                                                     this->mavlink_system.compid, 
                                                     &this->bufferMessage,
                                                     time,
                                                     name,
                                                     value);
  
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
}

/**
* \brief Sends a named signed 32 integer value. Maximum length of name: 10 characters
*/
void MAVLinkMAVion::sendInt32(INT32 value, char* name){
  // Pack the message
  UINT32 time = TimeBase::getSystemTime_ms();
  uint16_t length = mavlink_msg_named_value_int_pack(this->mavlink_system.sysid,
                                                     this->mavlink_system.compid, 
                                                     &this->bufferMessage,
                                                     time,
                                                     name,
                                                     value
                                                       );
  
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
}

/**
* \brief Sends a status text. Maximum length of text: 50 characters.
* \param[in] severity - indicates the type of this message, for avaliable types please have a look at 
*                       the MAV_SEVERITY enumeration of the mavlink common message set.
* \param text - the text to be sent, max. length 50 chars.
*/
void MAVLinkMAVion::sendStatusText(UINT8 severity, char* text){
  // Pack the message
  UINT32 time = TimeBase::getSystemTime_ms();
  uint16_t length = mavlink_msg_statustext_pack(this->mavlink_system.sysid,
                                                     this->mavlink_system.compid, 
                                                     &this->bufferMessage,
                                                     severity,
                                                     text);
  
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
}

/**
* \brief May be used to call e.g. the number of waypoints on the onboard waypoint list.
* \param[in] count - the number of mission items.
*/
void MAVLinkMAVion::sendMissionCount(UINT16 count){
  UINT16 length = mavlink_msg_mission_count_pack(this->mavlink_system.sysid,
                                                     this->mavlink_system.compid, 
                                                     &this->bufferMessage, 
                                                     this->gcsTarget_mavlink_system.sysid,
                                                     this->gcsTarget_mavlink_system.compid,
                                                     count);
  MAVLinkMAVion::sendMessage(&this->bufferMessage);        
}

/*
* \brief Sends a waypoint using a mission_item message.
*        Note: methods does not use all fields of the message_item message, therefore
*        please have a look at what the method does before using it.
* \param[in] lat - waypoint latitude in the WGS84 ECEF coordinate system [degree]
* \param[in] lon - waypoint longitude in the WGS84 ECEF coordinate system [degree]
* \param[in] alt - waypoint altitude in the WGS84 ECEF coordinate system [m]
* \param[in] alt - waypoint altitude in the WGS84 ECEF coordinate system [m]
* \param[in] hoverTime - the time [ms] the UAV is suppsoed to hover at the given waypoint before continuing its mission.
* \param[in] command - indicates the type of waypint (hover, drop payload, pass in cruise flight etc.), one of the mavlink common message set'S MAV_CMD enum.
* \param[in] numberOfWaypoint - the number of the waypoint in the UAV's waypoint list (starting at 0).

*/
void MAVLinkMAVion::sendWaypoint(float lat, float lon, float alt, float hoverTime, UINT16 command, UINT16 numberOfWaypoint){
  // TODO Not entirely sure if the 'seq' field of the message is really the number of da waypoint.
  static UINT8 autoContinue = 1;
  static UINT8 current = 0;       // Indicates wether this waypoint is the current waypoint (e.g. the waypoint the UAV is currently flying to).
                                  // FIXME Implement correct usage of message field.
  float unusedParameter = 0;          // paramters not used yet.
  UINT16 length = mavlink_msg_mission_item_pack(this->mavlink_system.sysid,
                                                this->mavlink_system.compid, 
                                                &this->bufferMessage, 
                                                this->gcsTarget_mavlink_system.sysid,
                                                this->gcsTarget_mavlink_system.compid,
                                                numberOfWaypoint,
                                                MAV_FRAME_GLOBAL, // TODO Make frame function parameter.
                                                command,
                                                current,                    
                                                autoContinue,  
                                                hoverTime,
                                                unusedParameter,
                                                unusedParameter,
                                                unusedParameter,
                                                lat,
                                                lon,
                                                alt);
                                                
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
}

/**
* \brief Notifies the receiver of which waypoint the UAV is currently heading to.
* \param[in] numberOfCurrentWaypoint - the index of this waypoint in the waypoint list.
*/
void MAVLinkMAVion::sendWhatIsCurrentWaypoint(UINT16 numberOfCurrentWaypoint){
  UINT16 length = mavlink_msg_mission_current_pack(this->mavlink_system.sysid,
                                                this->mavlink_system.compid, 
                                                &this->bufferMessage, 
                                                numberOfCurrentWaypoint);                                               
  MAVLinkMAVion::sendMessage(&this->bufferMessage);  
}

/*
* \brief Requests a waypoint from the GCS.
* \param[in] numberOfWaypoint - the number of the waypoint in the UAV's waypoint list (starting at 1).
*/
void MAVLinkMAVion::sendWaypointRequest(UINT16 numberOfWaypoint){
  // TODO Not entirely sure if the 'seq' field of the message is really the number of da waypoint.
  UINT16 length = mavlink_msg_mission_request_pack(this->mavlink_system.sysid,
                                                this->mavlink_system.compid, 
                                                &this->bufferMessage, 
                                                this->gcsTarget_mavlink_system.sysid,
                                                this->gcsTarget_mavlink_system.compid,
                                                numberOfWaypoint);
                                                
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
}


/*
* \brief Sends waypoint acknowledge message e.g. at the end of a waypoint list transaction.
* \param transactionOK - Indicates wether the transaction acknowldged by this message has been a valid transaction.
*                        Use false e.g. to let the GCS know that there has been invalid data, e.g. waypoint
*                        coordinates in space and not on earth.                        
*/
void MAVLinkMAVion::sendWaypointAck(bool transactionOK){
  UINT8 transactionResult = MAV_MISSION_ERROR;
  if(transactionOK){
    transactionResult = MAV_MISSION_ACCEPTED;
  }
  UINT16 length = mavlink_msg_mission_ack_pack(this->mavlink_system.sysid,
                                                this->mavlink_system.compid, 
                                                &this->bufferMessage, 
                                                this->gcsTarget_mavlink_system.sysid,
                                                this->gcsTarget_mavlink_system.compid,
                                                transactionResult);
                                                
  MAVLinkMAVion::sendMessage(&this->bufferMessage);
}

/**
* \brief Sends a parameter value.
*
* \param[in] value - the value of the parameter, e.g. 1.23.
* \param[in] totalNumberOfParameters - how many onboard parameters exist.
* \param[in] indexOfParameter - the index of this parameter in the list of parameters.
* \param[in] paramID - the String ID of this parameter, e.g. "Pgain_engineLaw", max. 16 characters.
* \param[in] parameterType - one of the types defined in the mavlink_message_type_t typedef in mavlink's mavlink_types.h
*/
void MAVLinkMAVion::sendFloatParameter(float value, UINT16 totalNumberOfParameters, UINT16 indexOfParameter, char* paramID, UINT8 parameterType){
  // Pack message.
  UINT16 len = mavlink_msg_param_value_pack(this->mavlink_system.sysid,
                                            this->mavlink_system.compid, 
                                            &this->bufferMessage,
                                            paramID,
                                            value,
                                            parameterType,
                                            totalNumberOfParameters,
                                            indexOfParameter);
  // Send message:
  MAVLinkMAVion::sendMessage(&this->bufferMessage);                                            
}

/**
* \brief Allows the receiver of this message to send for a certain time. Part of the mechanism
*        to deal with the issues the XBee modems have with simultaneous sending and receiving.
         due to their half duplex RF design with serial buffering.
* \param[in] time time in ms the receiver is allowed to send.
*
*        FIXME Uses the message_rate field of the MAVLINK_MSG_ID_DATA_STREAM 
*        message to transmit the time - better create a proper TX_WINDOW_OPEN message.
*/
void MAVLinkMAVion::openTxWindow(UINT16 time){
  UINT8 streamID = 0;
  UINT8 OnOff = 1;
  mavlink_msg_data_stream_pack(this->mavlink_system.sysid,
                                      this->mavlink_system.compid, 
                                      &this->bufferMessage,
                                      streamID,
                                      time,
                                      OnOff);
  MAVLinkMAVion::sendMessage(&this->bufferMessage);

}

/**
* \brief Writes a message to the serial port.
*/
void MAVLinkMAVion::sendMessage(mavlink_message_t* msg){
  // Copy the message to the message buffer:
  static UINT8 buffer[MAVLINK_MAX_PACKET_LEN];
  UINT16 length = mavlink_msg_to_send_buffer(buffer, msg);
  this->mySerialPort->EnvoyerTrame(length, buffer);
}

/*
* \brief Sets the serial link this MAVLink is supposed to use.
* \param[in] port The LiaisonSerie this MAVLink is supposed to use for data transfer.
*/
void MAVLinkMAVion::setPort(LiaisonSerie* port)
{
  this->mySerialPort = port;
  return;
}

/**
* \brief Defines where the MAVLink forwards messages to. 
*        Before this method has been called, no message is forwarded.
*/
void MAVLinkMAVion::startForwardingMessages(MAVLinkMAVion* target){
  this->forwardTo = target;
  this->forwardEveryMessage = true;
}

/**
* \brief Makes the MAVLink stop forwarding messages.
*/
void MAVLinkMAVion::stopForwardingMessages(){
  this->forwardEveryMessage = false;
}

/**
* \brief Makes the MAVlink call the given message handler every time a mavlink message
*        is received.
* \param[in] messageHandler The function that is called. MAVlink passes a pointer to the received message to this function.
* TODO Check if message handling takes too much time and may hence cause the serial port's input buffer to overflow.
*/
void MAVLinkMAVion::startCallingMessageHandler(void (*handler)(mavlink_message_t*)){
  this->callMessageHandler = true;
  this->messageHandler = handler;
}

/**
* \brief Makes the MAVlink stop calling the message handler function.
*/
void MAVLinkMAVion::stopCallingMessageHandler(){
  this->callMessageHandler = false;
}

/**
* \brief Return value indicates wether the MAVLink'S serial port is currently transmitting.
*/
bool MAVLinkMAVion::isBusyTransmitting(){
  return this->mySerialPort->isBusyTransmitting();
}

/*
* \brief Call this function as least as fast as bytes can arrive at the serial connection.
*        It checks for new received bytes and decodes MAVLink messages.
*        Note: May not work for XBee modems, since they are transmitting half-duplex, 
*        hence flow control has to be used. Therefore, only use this forwarding functionality
         only for true full-duplex serial connections.
*/
void MAVLinkMAVion::checkForNewBytes(){
  // TODO make function use a timeout in case there is an endless stream of bytes coming in.
  // TODO Maybe add message filter allowing to forward only certain message types.
  
  // Buffer messages used for decoding.
  static mavlink_message_t msg;
  static mavlink_status_t status;
  
  while(this->mySerialPort->donneeDisponible()){
    UINT8 c = this->mySerialPort->lire();
    if(this->forwardEveryMessage){
      this->forwardTo->getSerialPort()->ecrire(c);
    }
    
    // Try to get a new message
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if(this->callMessageHandler){
        this->messageHandler(&msg);
      }
    }
  }
  
}

/* 
* \brief Returns how much time has passed since the last heartbeat message has been received by this MAVLink.
* TODO Test.
*/
UINT32 MAVLinkMAVion::getTimeSinceLastHeartBeat(){
  return TimeBase::getSystemTime_ms() - this->timeOfLastHeartBeat;
}

/*
* \brief Sends an array of bytes, the user has to take care of the data structure, 
*        so it is not guaranteed to send a valid mavlink message.
*/
void MAVLinkMAVion::sendRawData(uint16_t length, uint8_t* buf){
  this->mySerialPort->EnvoyerTrame(length, buf);
}

/**
* \brief Sets all fields of a given Structure to their default values.
* \param[in] s The attitudeEuler structure to be initialized.
*/
void MAVLinkMAVion::initStructure(mavlink_attitudeEuler *s)
{
  for(UINT8 i=0; i<3; i++){
    s->euler[i] = 0;
    s->bodyRotationRate[i] = 0;
  }
}

/**
* \brief Sets all fields of a given Structure to their default values.
* \param[in] s The attitudeEuler structure to be initialized.
*/
void MAVLinkMAVion::initStructure(mavlink_attitudeQuaternion *q)
{
  q->w = 0;
  q->x = 0;
  q->y = 0;
  q->z = 0;
  for(UINT8 i=0; i<3; i++){
    q->bodyRotationRate[i] = 0;
  }
}

LiaisonSerie* MAVLinkMAVion::getSerialPort(){
  return this->mySerialPort;
}

/**
* \brief Checks wether a given ID is the ID of this MAVLinkMAVion.
* \param[in] idToCheck - The ID to be checked for identity with this MAVLinkMAVion'S ID.
* \return boolean, true if the given ID is the ID of this MAVLinkMAVion, false if not.
*/
bool MAVLinkMAVion::isMySystemID(UINT8 idToCheck){
  return idToCheck == this->mavlink_system.sysid;
}
