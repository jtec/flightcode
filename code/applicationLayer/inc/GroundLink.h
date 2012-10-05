/**
* \file GroundLink.h
* \brief Manages communications with an adapter merging the RC inputs and data from QGroundControl. Decodes 
*        mavlinlk messages and forwards the information to the other UAVComponents
*
*/

#ifndef _GROUNDLINK_H_
#define _GROUNDLINK_H_

#include "alMAVLink.h"
#include "clFactory.h"
#include "ParameterManager.h"

// States for the state machine managing receiving/sending parameters from/to the GCS.
typedef enum {
  // States for sending parameters to the GCS:
  PAR_IDLE,                        // The state machine is in idle state and waits for its state to be changed to take action.
  PAR_LIST_REQUEST_RECEIVED,        // The GroundLink has received a request for a list of onboard parameters from the GCS.
  PAR_GOT_PARAMETER                 // The GroundLink has received a parameter and has not yet acknowledged it by sending the same parameter back to the GCS.
} parameterTransactionStates;


struct ParameterTransactionManager{
  parameterTransactionStates state;
  uint16_t txCounter;          // Used to keep track of the number of parameters already sent.
  uint16_t indexOfLastReceivedParameter;  // Index of the parameter that has been received from the GCS and that has
                                        // to be acknowledged by the state machine.
};

// These flags are used to store the state of the data link to the ground control station 
struct GroundLinkState{
  bool txWindowOpen;  // Indicates wether the grounnd control station (i.e. the adapter betweeen QGroundControl and the MAVion) has allowed the MAVion to send.
  uint16_t txTimeLeft;   // Time [ms] that is left to transmit messages in the current communication window.
  uint16_t prescaler;     // Some messages are only sent every x='prescaler' time that a tx window opens up.
  uint16_t prescalerCounter;     // helper variable for the prescaler.
  struct ParameterTransactionManager parameterTransactionManager;         // Holds the state of the parameter transaction state machine. TODO Should be moved to its own class.
};

class GroundLink{
public :
  GroundLink(clSerialPort* serialPortXBee);
  ~GroundLink();
  void tick();       
  void introduceComponent(ParameterManager* s);
  void sendInfoMessage(char* m);
  int16_t howManyConnectionTimeouts();
  struct GroundLinkState linkState; // Public only for debugging.
private:
  static void mavlinkMessageHandler(mavlink_message_t* msg);
  static GroundLink* instance; // It is expected that only one instance of GroundLink is needed. FIXME Turn it into a singleton or make it work for more than one instance, if necessary.  
  void evaluateParameterTransactionStateMachine();
  alMAVLink mavLink;
  ParameterManager* parameterManager; // A GroundLink accesses the onboard parameters using this object.
  bool hasParameterManager;         // Indicates wether we have got a reference to a ParameterManager object yet.
  
  int32_t timeOfLastMessage;       // Indicates when the last message has been received [ms], time since startup.
  int16_t numberOfConnectionTimeouts;
  float connectionTimeoutTime;    // Indicates how much time has to pass without a message from the GCS to consider the connection as lost [ms]. 
                               // Is of type float because the ParameterManager only supports float parameters.
};



#endif  // _GROUNDLINK_H_
