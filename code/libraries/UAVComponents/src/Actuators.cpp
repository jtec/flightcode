 /**
* \file Actuators.cpp
* \brief An Actuators object provides a simple interface to control actuators like servos and motors
*
*/

#include "Actuators.h"
#include "loisCommandes.h"
#include "outils.h"

/**
* \brief Constructor, returns an instance of Actuators.
* \param isOldMAVion - indicates if teh PWM output channels should be configured for
* ths old MAVion (true) or the MAVion 2012 with PWM speed controllers (false)
*/
Actuators::Actuators(mavion_version config)
{
  // Define PWM outputs. Available PWM Channels: 0-5. TODO Don't use makros from declarationsGlobales.h, makes the class less reusable.
  this->pwmOutputs[0] = new PWM(VOIE_SORTIE_PWM_SERVO1);
  this->pwmOutputs[1] = new PWM(VOIE_SORTIE_PWM_SERVO2);
  this->pwmOutputs[2] = new PWM(VOIE_SORTIE_PWM_SERVO3);
  this->pwmOutputs[3] = new PWM(VOIE_SORTIE_PWM_SERVO4);
  this->pwmOutputs[4] = new PWM(VOIE_SORTIE_PWM_SERVO5);
  
  // Define speed controllers. TODO Don't use makros for controller addresses, maybe pass addresses to constructor?
  this->speedControllers[0] = new Variateur(ADRESSE_MOTEUR_1);
  this->speedControllers[1] = new Variateur(ADRESSE_MOTEUR_2);
  
  // Set default PWM outputs (1500µs = 50% deflection):
  for(int i=0; i<Actuators::numberOfPWMChannels; i++){
    this->pwmOutputs[i]->MiseAJourPWM(1500);
    this->pwmOutputs[i]->StartPWM();
  }
  
  for(int i=0; i<2; i++){
    // Set initial motor speeds to zero:
    this->desiredRotationalSpeed[i] = 0;
    // Initialize mesured rotational speed to zero.
    this->actualRotationalSpeed[i] = 0;    
    this->motorIsRunning[i] = false;
    this->motorCommands[i] = 0;
  }
  // Initialize control gains.
  this->gains.rev2Pwm = 10.0;    
  
  //We have no Sensors object yet, so we should't try to access it.
  this->hasSensors = false;
  
  // Set up PWM channel mixing:
  if(config == OLD_MAVION){
    this->mixer.map(0)->to(0);
    this->mixer.map(1)->to(1);
    this->mixer.map(2)->to(2);
    this->mixer.map(3)->to(3);
    this->mixer.map(4)->to(4);
  }else if(config == MAVION_2012){
    this->mixer.map(0)->to(0);
    this->mixer.map(1)->to(1);
    this->mixer.map(2)->to(3);
    this->mixer.map(3)->to(4);
    this->mixer.map(4)->to(2);    
  }// Else: keep Mixer standard configuration.
}

/**
* \brief Destructor.
*/
Actuators::~Actuators()
{
}

/**
* \brief Sets the pulse width [µs] of a PWM channel.
* \param[in] channel - channel number (0 to Actuators::numberOfPWMChannels-1)
* \param[in] value - pulse width in µs
*/
void Actuators::setPWM(UINT8 channel, UINT16 value){
  // Check if channel exists:
  if(channel < Actuators::numberOfPWMChannels){
	   // Limit value to Servo PWM minimum/maximum [1000,2000]:
	    if(value>2000){
	      value = 2000;
	    }
	    else if(value<1000){
	      value = 1000;
	    }

    this->pwm[this->mixer.get(channel)] = value;
    // Write value to PWM output
    this->pwmOutputs[this->mixer.get(channel)]->MiseAJourPWM(value);
  }
}

/**
* \brief Sets the rotational speed of a motor.
* \param[in] motorID - 0 or 1
* \param[in] speed - desired rotational speed [rev/s]
*                    For the MAVion: hover trim speed ~= 140 rev/s
*/
void Actuators::setMotorSpeed(UINT8 motorID, float speed){
  // Check if number exists:
  if(motorID == 0 || motorID == 1){
    this->desiredRotationalSpeed[motorID] = speed;
    // Saturate desired rotational speeds.    
     if (this->desiredRotationalSpeed[motorID]<0){
      this->desiredRotationalSpeed[motorID] = 0;
    }
    if (this->desiredRotationalSpeed[motorID]>1000){
      this->desiredRotationalSpeed[motorID] = 1000;
    }
  }
  
}

/**
* \brief Returns the rotational speed of a motor.
* \param[in] motorID - 0 or 1
* \return[in] speed - current rotational speed [rev/s]
*/
float Actuators::getMotorSpeed(UINT8 motorID){
  // Check if number exists:
  if(motorID == 0 || motorID == 1){
    return this->desiredRotationalSpeed[motorID];   // TODO Assumes that (desired speed) == (actual speed) because for certain speeds the mikrokopter I2C controller's measurements are not reliable.
  }else{
    return 0;
  }
}


/**
*
\brief Manages the closed loop control of the motors. Call with an appropriate frequency, e.g. 100 Hz.
*/
void Actuators::control(){     
  static bool closedLoopMotorControl = false;   // Indicates wether closed loop control will be used to control motor speed. Reason: closed loop does not work for speeds inferior to 30 rev/s. TODO Why not?
  // Parameters of the motor control law.
  static float tabBoucleVit_motor[5];
  static float gainsBoucleVit_motor[2] = {0.93, 2.0};
  static float batteryVoltage = 0;
  static float lastSpeed[2] = {0};
  
  if(this->hasSensors){
    batteryVoltage = this->sensors->battery.getVoltage();
  }else{  // If we have no sensors, just use an estimated battery voltage (3s LithiumPolymer) to keep motor control working.
    batteryVoltage = 11.0;
  }
  
  // Control of motor speeds:
  for(int i=0; i<2; i++){
    lastSpeed[i] = this->actualRotationalSpeed[i];
    // Measure motor speeds:
    //this->actualRotationalSpeed[i] = this->speedControllers[i]->recup_vitesse();
    
    // Measure rotational speed:
    // TODO Why do we have an distinction here between rotational speeds < 20 and above?
    if (this->desiredRotationalSpeed[i]>20){
      // get current rotational speed from speed controller:
      float inv_vit = this->speedControllers[i]->recup_vitesse();
      // TODO Explain the condition below. Seems to improve spinning up.
      if (inv_vit>2.0) {
        this->actualRotationalSpeed[i] = 420000.0/inv_vit;
      }
    }
    else {
      this->actualRotationalSpeed[i] = 20.0;
    }
    // Filter for absurdly high values sometimes received from the speed controllers:
    if(this->actualRotationalSpeed[i] > 500){
      this->actualRotationalSpeed[i] = lastSpeed[i];
    }
    
    // Switch to closed loop if speed > x tr/s since for speeds inferior to that, the speed measurement does not work.
    closedLoopMotorControl = (this->actualRotationalSpeed[i] > 80.0); 

    boucleFermeeVitesse(this->desiredRotationalSpeed[i], this->actualRotationalSpeed[i], &gainsBoucleVit_motor[0], closedLoopMotorControl, &tabBoucleVit_motor[0], batteryVoltage);
    //this->motorCommands[i] = (float)conversion_vitesse_throttle(tabBoucleVit_motor[0], batteryVoltage);
    // PWM motor command
    static float cmd = 0;
    cmd = 1000.0 + this->desiredRotationalSpeed[i] * this->gains.rev2Pwm;
    this->motorCommands[i] = (UINT32) cmd;
    }

     // Send commands
  //this->speedControllers[0]->envoi_vitesse_BO(this->motorCommands[0]);
  //this->speedControllers[1]->envoi_vitesse_BO(this->motorCommands[1]);
  // For IMAV 2012: using PWM controllers due to problems with closed-loop control using the mikrocopter I2C controllers.
  this->setPWM(2, this->motorCommands[0]);
  this->setPWM(3, this->motorCommands[1]);  
}

/**
* \brief Provides Actuators a Sensors object it can get the battery voltage from needed for controlling the motors.
*/
void Actuators::introduceComponent(Sensors* s){
  this->sensors = s;
  this->hasSensors = true;
}

/**
* \brief Gives the Actuators object a reference to a ParameterManager object it may use
*        to make its control parameters accessible to QGroundControl.
*/
void Actuators::introduceComponent(ParameterManager* p){
  this->parameterManager = p;
  this->hasParameterManager = true;
  // Make the control parameters accessible to the ParameterManager
  this->parameterManager->checkInFloat(&(this->gains.rev2Pwm), "actuat_rev2Pwm", 16);
 
}
