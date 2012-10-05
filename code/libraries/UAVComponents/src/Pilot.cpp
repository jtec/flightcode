/**
* \file Pilot.cpp
* \brief A Pilot object calculates the control output necessary to maintain the desired state
* of the UAV and applies this output to the actuators. A Pilot has various operating modes, i.e. flight laws:
* - gyro mode - In this mode the Pilot interprets the RC PWM inputs (it gets from a GroundLink object)
*                as inputs for a feed-forward controller of the body rotation rates. Please have a look
*                at the appropriate calculateOutputXYZ() member function for the implementation of this controller.
* - guided orientation mode - In this mode the RC inputs are translated into a desired body orientation.
* - automatic mode - In this mode, the UAV is supposed to follow a series of waypoints automatically.
* TODO Remove laws that are not used anymore.
*
*/

#include "Pilot.h"
#include "TimeBase.h"
#include "loisCommandes.h"
#include "navigation.h"
#include "outils.h"

Pilot* Pilot::publicInstance = 0;

/**
* \brief Constructor, returns an instance of Pilot.
*/
Pilot::Pilot(mavion_version mavionVersion)
{
  this->mavionVersion = mavionVersion;
  // At startup, disable all moving surfaces:
  this->currentOperatingMode = MAVION_OPMODE_INIT;
  this->lastOperatingMode = MAVION_OPMODE_INIT;
  // TODO Inizialize data structures for command and control.
  
  this->hasGroundLink = false;
  this->hasActuators = false;
  this->hasUAVState = false;
  this->hasNavigator = false;
  this->hasParameterManager = false;
  
  this->initializeControlParameters();
  Pilot::publicInstance = this;
}

/**
* \brief Destructor.
*/
Pilot::~Pilot()
{
}

/**
* \brief Sets the control parameters (mostly controller gains) to their default values
*        to ensure that the controllers work properly right from the beginning.
*/
void Pilot::initializeControlParameters(){

  if(this->mavionVersion == OLD_MAVION){
    // Default gains for the MAVion version pre-2012 (black EPP, pulso motors).
    // Updated june 26. 2012 after test flight with the pre-2012 MAVion.
    // roll
    this->controlData.kp[0] = 2.0;
    this->controlData.kd[0] = 0.1;
    this->controlData.ki[0] = 0.0;
    // pitch
    this->controlData.kp[1] = 6.4;
    this->controlData.kd[1] = 0.34;
    this->controlData.ki[1] = 0.0;
    // yaw
    this->controlData.kp[2] = -2.0;
    this->controlData.kd[2] = -0.3;
    this->controlData.ki[2] = 0.0;
    // altitude
    this->controlData.kp[3] = 40.0;
    this->controlData.kd[3] = 15.0;
    this->controlData.ki[3] = 0.0;  
    // Gain scheduling.
    // The old MAVion worked well with a gain scheduling between 0.25 and 1.0 for pitch
    // and no gain scheduling at all for roll and yaw.
    this->controlData.gainSchedulingFactorPitch_PD = 1.0;  
    this->controlData.gainSchedulingFactorRoll_PD = 1.0;
    this->controlData.gainSchedulingFactorYaw_PD = 1.0;  
    this->controlData.minimalGainSchedulingFactorPitch_PD = 0.25;  
    this->controlData.minimalGainSchedulingFactorRoll_PD = 1.0;  
    this->controlData.minimalGainSchedulingFactorYaw_PD = 1.0;  
  }else if(this->mavionVersion == MAVION_2012){
    // Default gains and PID I term limits for the MAVion version 2012 (slightly larger, axi gold motors, battery in the front).
    // roll
    this->controlData.kp[0] = 2.0;
    this->controlData.kd[0] = 0.1;
    this->controlData.ki[0] = 0.0;
    this->controlData.PID_iTerm_max[0] = 0.0;
    
    // pitch
    this->controlData.kp[1] = 1.5;
    this->controlData.kd[1] = 0.15;
    this->controlData.ki[1] = 0.025;  // In testflight on june 18 the MAVion#2012 had problems recovering from cruise flight, adding an integrator should help.
    this->controlData.PID_iTerm_max[1] = 0.1;
    // yaw
    this->controlData.kp[2] = -0.7;
    this->controlData.kd[2] = -0.13;
    this->controlData.ki[2] = 0.0;
    this->controlData.PID_iTerm_max[2] = 0.0;
    
    // altitude
    this->controlData.kp[3] = 20.0;
    this->controlData.kd[3] = 15.0;
    this->controlData.ki[3] = 0.0;  
    this->controlData.PID_iTerm_max[3] = 0.0;
    
    // Gain scheduling.
    // Default factors updated after test flight july 18. 2012.
    this->controlData.gainSchedulingFactorPitch_PD = 1.0;  
    this->controlData.gainSchedulingFactorRoll_PD = 1.0;
    this->controlData.gainSchedulingFactorYaw_PD = 1.0;  
    this->controlData.minimalGainSchedulingFactorPitch_PD = 0.3;  
    this->controlData.minimalGainSchedulingFactorRoll_PD = 0.5;  
    this->controlData.minimalGainSchedulingFactorYaw_PD = 0.5;        
  }else{ 
    // If the mavion version is undefined, set all gains to zero to make the error obvious to the user.
    for(int i=0; i<4; i++){
      this->controlData.kp[i] = 0.0;
      this->controlData.kd[i] = 0.0;
      this->controlData.ki[i] = 0.0;
    }
  }
  //  The following parameters are independent of the MAVion version.
  this->controlData.kheading = 1.5;
  this->controlData.kth = 65.0;
  // Altitude kalman filter gains
  this->controlData.kf[0] = 0.001;
  this->controlData.kf[1] = 0.002;
  // Initialize desired attitude quaternion, error quaternion, integral error quaternion.
  for(int i=0;i<3;i++){
    this->controlData.quatc[i] = 0;
    this->controlData.quate[i] = 0;  
    this->controlData.quate_i[i]=0.0;
  }
  this->controlData.quatc[3] = 1.0;
  this->controlData.quate[3] = 1.0;
  
  this->controlData.altMode = 0;  // 0 = altitude control disabled.
  this->controlData.altModeFloat = 0.0;  // Wrapper variable used make the 8-Bit altMode variable accessible
                                         // to QGroundcontrol, since currently only float variables can be displayed and set in the GCS.
                                         // TODO Make the 'ParameterManager' class support booleans/bytes/integers.
  this->controlData.error_pitch = 0.0;
  this->controlData.error_roll = 0.0;
  this->controlData.error_yaw = 0.0; 
  this->controlData.maxMotorSpeedManualFlight = 100.0;     
}

/**
* \brief Call this method to make the Pilot calculate the control outputs and
*        apply them to the actuators
*/
void Pilot::control(){
  // To make altMode settable by the GCS (ParameterManager supports only floats), a float variable is used as a wrapper.
  // Convert wrapper variable into real altMode variable:
  if(this->controlData.altModeFloat > 0.5){
    this->controlData.altMode = 1;
  }else if(this->controlData.altModeFloat < 0.5){
    this->controlData.altMode = 0;
  }
  // Read the last RC inputs, updating the control inputs and the operating mode.
  this->readRCInputs();
  // Before the new operating mode is determined, save it as the last operating mode.
  this->lastOperatingMode = this->currentOperatingMode;
  this->figureOutOperatingMode();
  // Tell the GroundLink about it:
  if(this->hasGroundLink){
    this->groundlink->updateOperatingMode(this->currentOperatingMode);
    // If the mode has changed, send an info message.
    if(this->currentOperatingMode != this->lastOperatingMode){
      this->publishMode();
    }
  }
  // If the pilot has just switched from guided orientation mode to
  // automatic mode, reanalyze the flight path.
  if((this->currentOperatingMode == MAVION_OPMODE_AUTOMATIC) && (this->lastOperatingMode == MAVION_OPMODE_ORIENTATION_GUIDED)){
    this->navigator->doTrajectoryAnalysis(); 
  }
  // Calculate control output based on operating mode using a little state machine:
  switch(this->currentOperatingMode){
  case MAVION_OPMODE_INIT:
    this->calculateOutput_KillAllLaw();
    break;  
  case MAVION_OPMODE_KILLALL:
    this->calculateOutput_KillAllLaw();
    break;
  case MAVION_OPMODE_KILLALL_FOREVER:
    this->calculateOutput_KillAllLaw();
    break;
  case MAVION_OPMODE_AUTOTAKEOFF:
    this->calculateOutput_AutoTakeoff();
    break;
  case MAVION_OPMODE_GYRO_STABILIZED:
    this->calculateOutput_GyroLaw();
    break;    
  case MAVION_OPMODE_ORIENTATION_GUIDED:
    this->calculateOutput_OrientationGuidedLaw();
    break;  
  case MAVION_OPMODE_AUTOMATIC:
    this->calculateOutput_AutomaticLaw();
    break;    
  default:
    // If no valid mode is provided, stop everything since apparently something is going wrong.
    this->calculateOutput_KillAllLaw();
    break;
  }
  // Send output to actuators. 
  this->controlActuators();
  return;
}

/**
* \brief Applies the control output that has been calculated before to the actuators.
* Converts control outputs [-1, 1] to PWM [1000, 2000].
*/
void Pilot::controlActuators(){
  static bool disableEngines = true;
  // If we are in initialisation or gyro mode, cut the engines if the throttle stick is in it's lower position.
  // This is to keep the motors from spinning up unexpectedly due to rotations around z in gyro mode.
  disableEngines = ((this->currentOperatingMode == MAVION_OPMODE_INIT) || (this->currentOperatingMode == MAVION_OPMODE_GYRO_STABILIZED)) && (this->commandData.entreesRC[0] < 1100);                   
  // Don't access actuators if we don't know any i.e. have no reference to an Actuators object.
  if(this->hasActuators){
    // Apply output to servos:
    // Don't allow servos to saturate due to aileron, this causes problems with elevator.
    this->commandData.aileron_p = this->commandData.aileron*(1-this->commandData.elevator*this->commandData.elevator);
    float servo1 = 1500 + 500* this->commandData.elevator + 500* this->commandData.aileron_p;
    float servo2 = 1500 - 500* this->commandData.elevator + 500* this->commandData.aileron_p;
    this->actuators->setPWM(0, servo1);
    this->actuators->setPWM(1, servo2);
    
    if(disableEngines){
      this->controlData.consigne_vitesse_motor1 = 0;
      this->controlData.consigne_vitesse_motor2 = 0;
    }
    // Apply output to motors:
    this->actuators->setMotorSpeed(0, this->controlData.consigne_vitesse_motor1 );
    this->actuators->setMotorSpeed(1, this->controlData.consigne_vitesse_motor2 );
  }
  return;
}

/**
* \brief Reads the RC inputs received by the GroundLink component and copies them to local class variables.
*/
void Pilot::readRCInputs(){
  
  if(this->hasGroundLink){
    // Copy RC inputs from GroundLink to command structure (because it is used by some functions of the pre-2012 MAVion code)
    for(int i=0;i<8;i++) {
      this->commandData.entreesRC[i] = this->groundlink->pwmInputs[i];
    }
    // Convert RC inputs [1000,2000] to control inputs [-1.0,1.0].
    this->controlData.heading_rate = Pilot::pwm2float( this->commandData.entreesRC[3]);
    this->controlData.x_command =  Pilot::pwm2float( this->commandData.entreesRC[2]);
    this->controlData.y_command =  Pilot::pwm2float( this->commandData.entreesRC[1]);
    // The conversion between RC input channel 1 and the throttle control input is not linear
    // but linear in two sections to enhance the pilots control for low throttle values.
    /*
    if (((float)this->commandData.entreesRC[0])<1100.0) {
    // PWM [1000, 1100] -> rotation speed [0.0, 100.0]
    this->controlData.consigne_vitesse = 1.0*(((float)this->commandData.entreesRC[0])-1000.0);
  }else {
    // PWM [1100, 2000] -> rotation speed [100.0, 220.0]
    this->controlData.consigne_vitesse = 100.0 + 120.0*(((float)this->commandData.entreesRC[0])-1100.0)/900.0;
  } 
    */
    // Simpler linear function, works better with PWM speed controllers:
    this->controlData.consigne_vitesse = this->controlData.maxMotorSpeedManualFlight*(this->commandData.entreesRC[0]-1000.0)/1000.0;
  }
}
/**
* \brief Interprets RC inputs to determine the operating mode. The pilot uses a switch on channel 5
to decide between 
1) stabilized mode / gyro mode -  the desired rotation rates are kept to zero  + a feed forward component derived from RC input.
2)  GPS/Auto mode - the UAV follows its mission automatically.
*   or guided orientation mode - the stick positions are mapped to the UAV'S attitude. 
*   This depends on the position of a second switch on RC channel 6.
*/
void Pilot::figureOutOperatingMode(){
  static UINT16 initialAutotakeoffChannelValue = 0; // The PWM pulse width of the RC channel used for the automatic takeoff command when the RC Transmitter gets switched on.
  
  // RC channels are translated into a set of booleans that is easier to evaluate for the new operating mode;
  static bool isGyroModeOn = false;       // Indicates wether the auto/gyro mode switch is in 'gyro mode' position;
  static bool isAutomaticModeOn = false;  // Indicates wether the guided/auto mode switch is in 'automatic mode' position;
  static bool isGuidedModeOn = false;  // Indicates wether the guided/auto mode switch is in 'automatic mode' position;
  
  static bool isAutoTakeoffOn = false;   // Indicates wether the automatic takeoff switch is in 'auto takeoff' position;
  static bool isEmergencyStopRequested = false; 
  
  isGyroModeOn = (this->commandData.entreesRC[4] < 1500);
  isAutomaticModeOn = (this->commandData.entreesRC[5] > 1500);
  isGuidedModeOn = (this->commandData.entreesRC[5] < 1500);
  //    isAutoTakeoffOn = (fabs(initialAutotakeoffChannelValue - this->commandData.entreesRC[5]) > 500);
  isEmergencyStopRequested = (this->commandData.entreesRC[6] < 1500);
  
  // Decide between gyro mode / guided orientation mode / automatic mode, controlled by two RC channels.
  if(isGyroModeOn){
    this->currentOperatingMode = MAVION_OPMODE_GYRO_STABILIZED;
  }else if(isAutomaticModeOn){
    this->currentOperatingMode = MAVION_OPMODE_AUTOMATIC;
  }
  else if(isGuidedModeOn){
    this->currentOperatingMode = MAVION_OPMODE_ORIENTATION_GUIDED;
  }else{  // If mode is undefined, kill all, since something seems to be wrong.
    this->currentOperatingMode = MAVION_OPMODE_KILLALL;
  }  
  
  
  if(isAutoTakeoffOn && (this->currentOperatingMode == MAVION_OPMODE_GYRO_STABILIZED)){ // Auto takeoff mode can only be entered from gyro mode.
    //this->currentOperatingMode = MAVION_OPMODE_AUTOTAKEOFF;   // automatic takeoff is still flawed, rethink; maybe just take 
    // off automatically if the pilot switches to auto mode while the MAVion is still on the ground, making auto takeoff a part aof the automatic mode.
  }
  
  // If the state filter is still initializing, stay in init mode.
  if(this->uavState->isStillInitializing()){
    this->currentOperatingMode = MAVION_OPMODE_INIT;
  }

  // If the emergency stop channel indicates it (either because the pilot has pressed
  // the emergency stop button or because the RC receiver went into fail safe mode, e.g. due to the
  // RC transmitter having been switched off), stop all moving parts, since obviously a severe malfunction
  // has occured and the safety pilot can no longer control the UAV.
  // If we are in init mode, we can stay there, since all moving parts are stopped anyway.
  if(isEmergencyStopRequested && (this->currentOperatingMode != MAVION_OPMODE_INIT)){
    this->currentOperatingMode = MAVION_OPMODE_KILLALL;
  }
  // Check if the connection to the GCS has been lost; if so, shut down the UAV forever.
  // To get out of this mode the system has to be restarted.
  if(this->groundlink->howManyConnectionTimeouts() > 0){
    this->currentOperatingMode = MAVION_OPMODE_KILLALL_FOREVER;
  }
  
}

/**
* \brief Sends an info message about the current operating mode to the GCS.
*/
void Pilot::publishMode(){
  if(this->hasGroundLink){
    switch(this->currentOperatingMode){
    case MAVION_OPMODE_INIT:
      this->groundlink->sendInfoMessage("Pilot.cpp: operating mode: init");
      break;  
    case MAVION_OPMODE_KILLALL:
      this->groundlink->sendInfoMessage("Pilot.cpp: operating mode: kill all");
      break;
    case MAVION_OPMODE_KILLALL_FOREVER:
      this->groundlink->sendInfoMessage("Pilot.cpp: system shut down due to com. loss");  // TODO Not very likely to be displayed in the GCS since apparently the commnuncation link has stopped working.
      break;
    case MAVION_OPMODE_AUTOTAKEOFF:
      this->groundlink->sendInfoMessage( "Pilot.cpp: operating mode: auto takeoff");
      break;
    case MAVION_OPMODE_GYRO_STABILIZED:
      this->groundlink->sendInfoMessage("Pilot.cpp: operating mode: gyro stabilized");
      break;    
    case MAVION_OPMODE_ORIENTATION_GUIDED:
      this->groundlink->sendInfoMessage("Pilot.cpp: operating mode: guided orientation");
      break;  
    case MAVION_OPMODE_AUTOMATIC:
      this->groundlink->sendInfoMessage("Pilot.cpp: operating mode: automatic");
      break;    
    default:
      this->groundlink->sendInfoMessage("Pilot.cpp: operating mode: undefined mode");
      break;
    }
  }
}

/**
* \brief Returns the Pilot's current operating mode.
* \return the operating mode, a vaule of the enumeration type mavion_operatingMode.
*/
mavion_operatingMode Pilot::getOperatingMode(){
  return this->currentOperatingMode;
}

/**
* \brief Method calculates the control outputs in KILLALL mode.
*        It stops all motors and sets the servos to their default positions.
*/
void Pilot::calculateOutput_KillAllLaw(){
  
  this->controlData.consigne_vitesse = 0;  
  this->controlData.consigne_vitesse_motor1 = 0;
  this->controlData.consigne_vitesse_motor2 = 0;
  
  this->commandData.elevator = 0;
  this->commandData.aileron = 0;
  this->commandData.rudder = 0;
  this->commandData.motor1 = 0;
  this->commandData.motor2 = 0;
}

/**
* \brief Method calculates the control outputs during automatic takeoff
*        In this mode, the UAV gently spins up its engines up to 90% of hover trim speed
*        (If the pilot has not excceeded this level while taking off in gyro mode), inserts a
*        waypoint to hover at for several seconds 10 m above its current position at the 
*        very beginning of the waypoint list and switches to automatic mode.
*/
void Pilot::calculateOutput_AutoTakeoff(){
  static UINT32 spinupStartTime = 0;    // Used to spin up the motors in a gentle way.
  static float deltaSpeed[2] = {0};      // The difference between the motor speed when the auto takeoff law is entered and the motor speed
  // when the auto takeoff law switches over to the automatic law.
  static float initialMotorSpeed[2] = {0};  // The motor rotation speeds when the pilot switches to auto takeoff mode
  // Check wether we have just switched to this law.
  if(this->lastOperatingMode == MAVION_OPMODE_GYRO_STABILIZED){
    for(UINT8 i=0; i<2; i++){
      initialMotorSpeed[i] = this->actuators->getMotorSpeed(i);
      deltaSpeed[i] = Pilot::hoverTrimspeed - initialMotorSpeed[i];
      spinupStartTime = TimeBase::getSystemTime_ms();
    }
  }
  // If the motor speed is still less than 90% of hover trim speed, 
  // accelerate with 20 rev/s/s, i.e. 0.002 rev/s/ms
  if(this->controlData.consigne_vitesse < 0.9*(float)Pilot::hoverTrimspeed){
    float timeSinceAutoTakeoffCommand = TimeBase::getSystemTime_ms() - spinupStartTime;
    this->controlData.consigne_vitesse += timeSinceAutoTakeoffCommand * 0.002;
  }else{
    // FIXME set waypoint 10m above the current position to make the UAV take off when in auto mode.
    //this->autoTakeoffLawHasAutoModeRequested = true;;
  }
  
}

/**
* \brief Method calculates the control outputs in guided orientation mode.
*        In this mode, the pilot controls the UAVs body orientation.
*/
void Pilot::calculateOutput_OrientationGuidedLaw(){  
  // Only use UAV state object refernce if it has been set.
  if(this->hasUAVState && this->hasNavigator){
    // Use pilot inputs to calculate new desired orientation.
    preparerConsignesManuel(&(this->uavState->state), &this->controlData);     
    // Calculates the orientation quaternion error.
    reference_model(this->uavState->state, &this->controlData);                 
    // PID control laws, use desired orientation stored in control structure and actual orientation stored in etat structure
    // to output aileron, elevator, rudder, and motor commands (commandes structure)
    loisCENTRALE(this->uavState->state, &this->controlData,  &this->commandData);
    // set desired altitude to current altitude.
    // TODO explain conditional.
    if(this->controlData.altMode==0){
      this->controlData.z = this->uavState->state.z;
    }
    // Make sure all GPS related variables are properly initialized before entering GPS mode / automatic law.
    // As a consequence, the UAV restarts its mission at the at the first waypoint whenever the
    // pilot switches from guided orientation law to automatic law.
    initGPSMode(&this->navigator->navigationData,this->controlData,this->uavState->state);                    
  }
  
}

/**
* \brief Method calculates the control outputs in automatic mode.
In this mode, the UAV follows its mission automatically using GPS.
*/
void Pilot::calculateOutput_AutomaticLaw(){
  if(this->hasUAVState && this->hasNavigator){
    // Call navigation algorithm (computes the new desired attiude)
    navigationMain(&this->navigator->navigationData,&this->uavState->state,&this->controlData);
    // Calculates the orientation attitude quaternion.
    reference_model(this->uavState->state,&this->controlData); 
    // PID control laws, try to maintain the desired attitude.
    loisCENTRALE(this->uavState->state,&this->controlData, &this->commandData);
    if(this->uavState->state.GPS.nbSat < 4){
      this->navigator->navigationData.outsideYellow = true; 
    }
  }
}

/**
* \brief Method calculates the control outputs in stabilized mode. In this mode, 
the desired body rotation rates are kept to zero and a feed forward component derived from the
RC inputs is added to the control outputs.
*/
void Pilot::calculateOutput_GyroLaw(){
  // Set desired orientation = current orientation.
  //set desired heading based on current heading
  this->controlData.heading = this->uavState->state.headingVTOL; 
  for(int i=0;i<4;i++){
    this->controlData.quatc[i] = this->uavState->state.quat[i];
  }
  loisGYRO(this->uavState->state, &this->controlData, &this->commandData);  
  
  // Set integral quaternion error to zero, preparing transition to guided or automatic laws.
  for(int i=0; i<3; i++) {
    this->controlData.quate_i[i] = 0.0; 
  }
}   

/**
* \brief Gives the Pilot a reference to an Actuators object it can use to control
*        servos, motors etc.
*/
void Pilot::introduceComponent(Actuators* a){
  this->actuators = a;
  this->hasActuators = true;
}

/**
* \brief Gives the Pilot a reference to a GroundLink object it can get the RC inputs
*        and commands sent by the ground control station from.
*/
void Pilot::introduceComponent(GroundLink* a){
  this->groundlink = a;
  this->hasGroundLink = true;
  // Send a mode update message to the GCS (just for convenience, mode is sent to the GCS by GroundLink as part of the 1 Hz status message all the time)
  this->publishMode();
}

/**
* \brief Gives the Pilot a reference to a PhysicalState object it may use to get 
information about the UAVs current orientation, position, altitude etc..
*/
void Pilot::introduceComponent(PhysicalState* p){
  this->uavState = p;
  this->hasUAVState = true;
}

/**
* \brief Provides the Pilot with a reference to a Navigator object it may use to get 
information about the UAVs mission.
*/
void Pilot::introduceComponent(Navigator* n){
  this->navigator = n;
  this->hasNavigator = true;
}

/**
* \brief Gives the Pilot a reference to a ParameterManager object it may use
*        to make its control parameters accessible to QGroundControl.
*/
void Pilot::introduceComponent(ParameterManager* p){
  this->parameterManager = p;
  this->hasParameterManager = true;
  // Make the control parameters accessible to the ParameterManager
  this->parameterManager->checkInFloat(&(this->controlData.kp[0]), "ctrl_roll_kp", 16);
  this->parameterManager->checkInFloat(&(this->controlData.kp[1]), "ctrl_pitch_kp", 16);
  this->parameterManager->checkInFloat(&(this->controlData.kp[2]), "ctrl_yaw_kp", 16);
  this->parameterManager->checkInFloat(&(this->controlData.kp[3]), "ctrl_alt_kp", 16);
  
  this->parameterManager->checkInFloat(&(this->controlData.kd[0]), "ctrl_roll_kd", 16);
  this->parameterManager->checkInFloat(&(this->controlData.kd[1]), "ctrl_pitch_kd", 16);
  this->parameterManager->checkInFloat(&(this->controlData.kd[2]), "ctrl_yaw_kd", 16);
  this->parameterManager->checkInFloat(&(this->controlData.kd[3]), "ctrl_alt_kd", 16);
  
  this->parameterManager->checkInFloat(&(this->controlData.ki[0]), "ctrl_roll_ki", 16);
  this->parameterManager->checkInFloat(&(this->controlData.ki[1]), "ctrl_pitch_ki", 16);
  this->parameterManager->checkInFloat(&(this->controlData.ki[2]), "ctrl_yaw_ki", 16);
  this->parameterManager->checkInFloat(&(this->controlData.ki[3]), "ctrl_alt_ki", 16);
  
  this->parameterManager->checkInFloat(&(this->controlData.PID_iTerm_max[0]), "ctrl_roll_I_max", 16);
  this->parameterManager->checkInFloat(&(this->controlData.PID_iTerm_max[1]), "ctrl_ptch_I_max", 16);
  this->parameterManager->checkInFloat(&(this->controlData.PID_iTerm_max[2]), "ctrl_yaw_I_max", 16);
  this->parameterManager->checkInFloat(&(this->controlData.PID_iTerm_max[3]), "ctrl_alt_I_max", 16);
  
  this->parameterManager->checkInFloat(&(this->controlData.kheading), "ctrl_kheading", 16);
  this->parameterManager->checkInFloat(&(this->controlData.kth), "ctrl_kth_pilot", 16);
  
  this->parameterManager->checkInFloat(&(this->controlData.kf[0]), "ctrl_kalmanAlt0", 16);
  this->parameterManager->checkInFloat(&(this->controlData.kf[1]), "ctrl_kalmanAlt1", 16);
  
  this->parameterManager->checkInFloat(&(this->controlData.altModeFloat), "ctrl_altMode", 16);
  this->parameterManager->checkInFloat(&(this->controlData.maxMotorSpeedManualFlight), "ctrl_maxV", 16);

  this->parameterManager->checkInFloat(&(this->controlData.minimalGainSchedulingFactorPitch_PD), "ctrl_minGS_Ptch", 16);
  this->parameterManager->checkInFloat(&(this->controlData.minimalGainSchedulingFactorRoll_PD), "ctrl_minGS_Roll", 16);
  this->parameterManager->checkInFloat(&(this->controlData.minimalGainSchedulingFactorYaw_PD), "ctrl_minGS_Yaw", 16);

}

/**
* \brief Converts a PWM value [1000, 2000] into a float value [-1, 1].
*/
float Pilot::pwm2float(UINT16 pwm){
  return (((float)pwm)-1500)/500;
}

/**
* \brief Converts a float value [-1, 1] into a PWM value [1000, 2000].
*/
UINT16 Pilot::float2pwm(float flt){
  return (float)(flt*500+1500);
}
