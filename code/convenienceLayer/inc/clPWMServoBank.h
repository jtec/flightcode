/*
 * \file clPWMServoBank.h
 * \brief Please give a short description of the file's purpose and content.
 * \author Jan
 */

#ifndef DEVICEDRIVER_SERVOBANK_H_
#define DEVICEDRIVER_SERVOBANK_H_

//Inclusions
#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"
#include "clAssistant.h"


//Class containing all initial information the constructor needs to configure the peripherals.
class ServoBankBuildConfiguration{
public: TIM_TypeDef* timers[4];
		GPIO_TypeDef* ports [16];
		uint16_t pins[16];
		uint32_t frequency;
		uint32_t initialPulseWidths[16];
		uint8_t numberOfChannels;
};




//class declaration
// A ServoBank represents a set of servo connectors.
class clPWMServoBank{
	public:
		//methods
	clPWMServoBank(ServoBankBuildConfiguration*);
		~clPWMServoBank();
		void setFrequency(uint32_t frequency);
		void setPositions(float* ChannelWidths);
		void setPosition(uint8_t channel, float value);
		//uint8_t numberOfChannels;
	private:
		//methods
		uint32_t initTimer(TIM_TypeDef* timer);
		void configureGPIO(GPIO_TypeDef* gpio, uint16_t pins);
		void enableGPIOClockForPWM(GPIO_TypeDef* gpio);
		void enableTIMERClockForPWM(TIM_TypeDef* timer);
		uint32_t getTimerClock(TIM_TypeDef* timer);
		void updatePWM();
		//fields
		static const uint8_t maxNumberOfChannels = 16;
		uint32_t pwm[maxNumberOfChannels];
		uint8_t numberOfTimers;
		uint8_t numberOfGpioPins;
		uint8_t numberOfGpioPorts;
		TIM_TypeDef* timerPeripherals[4];
		uint32_t timer_nsPerTick;

};

#endif /* DEVICEDRIVER_SERVOBANK_H_ */
