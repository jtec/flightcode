/**
 *\file clPWMServoBank.cpp
 *\brief FIXME Do documentation.
 *\author Jan
 */

#include "../inc/clPWMServoBank.h"

/**
 * \brief Constructor;
 * \param[in] ServoBankBuildConfigurationToUse - pointer to a configurationstructure holding all the necessary
 * information about which timer peripherals to use, the desired number of channels etc.
 * TODO Alternative: the initial value scould by set by using a setAllChannelsTozero() method and then a startGeneratingPWM() method.
 */
clPWMServoBank::clPWMServoBank(ServoBankBuildConfiguration* ServoBankBuildConfigurationToUse)
{
	// TODO Check if TIMERs and GPIOs are actual peripherals.
	if(ServoBankBuildConfigurationToUse->numberOfChannels != 0){
		//Calculating the number of timers:
		//Since the number of timers directly depends on the number of channels (i.e. one timer serves 4 channels), the
		//number of timers has to be computed based on the number of channels.
		if(ServoBankBuildConfigurationToUse->numberOfChannels % 4 == 0){
			this->numberOfTimers = ServoBankBuildConfigurationToUse->numberOfChannels/4;
		}
		else{
			numberOfTimers = ServoBankBuildConfigurationToUse->numberOfChannels/4 +1;
		}

		//Setting Number of GPIO Pins:
		//Each Pin Serves one Chanel -> we can thus say that the number of pins is equal to the number of channels.
		numberOfGpioPins = ServoBankBuildConfigurationToUse->numberOfChannels;

		//Calculating the Number of used GPIO Ports:
		//Since we want to configure our Pins to PWM and the "configureGPIO" function needs a port for
		//each of the pins, the number of ports corresponds to the number of pins:
		numberOfGpioPorts = numberOfGpioPins;

		//Configure GPIO pins for PWM output and set the initial Pulse widths values:
		for(uint8_t i = 0; i<numberOfGpioPins; i++){
			configureGPIO(ServoBankBuildConfigurationToUse->ports[i], ServoBankBuildConfigurationToUse->pins[i]);
			this->pwm[i] = ServoBankBuildConfigurationToUse->initialPulseWidths[i];
		}

		// Configure PWM Timers and related peripherals
		// configure timers for PWM:
		for(uint8_t i = 0; i<numberOfTimers; i++){
			//copy pointers to Timer structures
			this->timerPeripherals[i] = ServoBankBuildConfigurationToUse->timers[i];
			this->timer_nsPerTick = initTimer(ServoBankBuildConfigurationToUse->timers[i]);
		}

		this->setFrequency(ServoBankBuildConfigurationToUse->frequency);
		this->updatePWM();
	}
	return;

}//eof

/**
 * \brief Destructor.
 */
clPWMServoBank::~clPWMServoBank(){
	// TODO Free memory?
}//eof

/**
 * @brief Sets the frequency of the PWM pulses (e.g. 50 Hz for standard PWM servos)
 * @param uint32_t - the desired frequency [Hz]. If the parameter is 0, a default frequency
 * of 1 Hz is chosen.
 */
void clPWMServoBank::setFrequency(uint32_t frequency)
{
	// Protection against division by zero:
	if(frequency == 0){
		frequency = 1;
	}
	float interval = 1000000000/(float)frequency;
	for(uint8_t i = 0; i<this->numberOfTimers; i++){
		// Change the the Timer's ARR register:
		uint32_t intervalInTimerTicks = (uint32_t)interval/this->timer_nsPerTick;
		TIM_SetAutoreload(this->timerPeripherals[i], intervalInTimerTicks);
	}

	return;
}//eof

/**
 * @brief Sets all channels to the given positions.
 * @param positions - pointer to an array (12 elements) containing the new positions [-1.0, 1.0].
 */
void clPWMServoBank::setPositions(float* positions)
{
	//We use the variable "numberOfGpioPins" because the variable "numberOfChannels" is used in the class
	//"ServoBankBuildConfiguration". As discribed above, the number of GPIO Pins corresponds to the number
	//of channels
	for(uint8_t i = 0; i<numberOfGpioPins; i++){
		this->pwm[i] = 1500000 + positions[i]*500000;
	}
	this->updatePWM();
	return;
}//eof

/**
 * \brief Sets one channel to the given value.
 *
 * \param channel - the number of the channel [0, maxNumberOfChannels-1].
 * \param value - the new output [-1.0, 1.0].
 */
void clPWMServoBank::setPosition(uint8_t channel, float value){
	// Check if a timer for this channel exists.
	if(channel < this->numberOfGpioPins){
		this->pwm[channel] = 1500000 + value*500000;
	}
	this->updatePWM();
	return;
}

/**
 * @brief Sets the PWM pulsewidths to the values in the private pwm array [ns].
 */
void clPWMServoBank::updatePWM()
{
	for(uint8_t i = 0; i<this->numberOfTimers; i++){
		this->timerPeripherals[i]->CCR1 = (uint16_t)(this->pwm[i*4 + 0]/this->timer_nsPerTick);
		this->timerPeripherals[i]->CCR2 = (uint16_t)(this->pwm[i*4 + 1]/this->timer_nsPerTick);
		this->timerPeripherals[i]->CCR3 = (uint16_t)(this->pwm[i*4 + 2]/this->timer_nsPerTick);
		this->timerPeripherals[i]->CCR4 = (uint16_t)(this->pwm[i*4 + 3]/this->timer_nsPerTick);
	}
	return;
}//eof

/**
 *
 * @brief Configures and starts a General Purpose Timer as standard PWM Timer
 * (initial setting: generate a pulse every 20ms, initial pulswidth 1,5ms).
 * @return uint32_t - the time interval of one timer tick [ns/tick]
 */
uint32_t clPWMServoBank::initTimer(TIM_TypeDef* timer){
	// FIXME a certain option has to be set for advanced timers as TIM1, TIM8 to make them work; something like 'output activation'.
#define CCDEFAULTVALUE 1500
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	//enable timer clock and get Timer clock frequency
	clAssistant::enableTIMERClock(timer);
	uint32_t timer_clockFrequency = getTimerClock(timer);
	uint32_t prescaler = (timer_clockFrequency/2000000);				//counter frequency = 2 MHz
	uint32_t counterFrequency = timer_clockFrequency/prescaler;
	uint32_t nsPerTick = 1000000000/counterFrequency;				//10^6 ns divided by the counter frequency

	//set registers to their default values
	TIM_DeInit(timer);

	/* Time Base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;							//indicates the division ratio between the timer clock (CK_INT) frequency and
	//sampling clock used by the digital filters (ETR, TIx),
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 20000000/nsPerTick;							//sets the ARR register: timer is reloaded once it has reached x
	TIM_TimeBaseStructure.TIM_Prescaler = 2*prescaler;								//FIXME check why it has to be "2*"  to set counter frequency to 1MHz
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;								//only applies to TIM1 and TIM8
	TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
	//done

	TIM_OCStructInit(&TIM_OCInitStructure);

	/* configure Channel 1, 2, 3 and 4 in PWM 1 mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;							//only applies to TIM1 and TIM8
	TIM_OCInitStructure.TIM_Pulse = CCDEFAULTVALUE;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;								//only applies to TIM1 and TIM8
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;							//only applies to TIM1 and TIM8
	TIM_OC1Init(timer, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = CCDEFAULTVALUE;
	TIM_OC2Init(timer, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = CCDEFAULTVALUE;
	TIM_OC3Init(timer, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = CCDEFAULTVALUE;
	TIM_OC4Init(timer, &TIM_OCInitStructure);

	//create timer update event to start transfer from preload registers to shadow registers
	TIM_GenerateEvent(timer, TIM_EventSource_Update);

	// Enable timer.
	TIM_Cmd(timer, ENABLE);

	return nsPerTick;
}//eof

/**
 * \brief Configures 1-16 GPIO pins to be used for PWM output.
 * \param gpio - the identifier of the port, e.g. GPIOB.
 * \param pins - a 16Bit (since there are 16 pins per GPIO port) binary number indicating which pins are supposed to be configured;
 * e.g. (0b0001 | 0b0100 ) means pin 0 and pin 2
 */
void clPWMServoBank::configureGPIO(GPIO_TypeDef* gpio, uint16_t pins){
	GPIO_InitTypeDef GPIO_InitStructure;

	clAssistant::enableGPIOClock(gpio);
	GPIO_InitStructure.GPIO_Pin = pins;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(gpio, &GPIO_InitStructure);
}//eof

/**
 * @brief returns the clock frequency of the given TIMER peripheral.
 * @param TIM_Typedef* timer - the identifier of the TIMEr peripheral, e.g. TIM2.
 */
uint32_t clPWMServoBank::getTimerClock(TIM_TypeDef* timer){
	RCC_ClocksTypeDef rccClock;
	RCC_GetClocksFreq(&rccClock);
	uint32_t timer_clockFrequency = 0;
	//timers connected to APB1
	if(timer==TIM2 || timer==TIM3 || timer==TIM4 || timer==TIM5 || timer==TIM6 || timer==TIM7 || timer==TIM12 || timer==TIM13 || timer==TIM14)
	{
		timer_clockFrequency = rccClock.PCLK1_Frequency;
	}
	else if(timer==TIM1 || timer==TIM8 || timer==TIM9 || timer==TIM10 || timer==TIM11 )
	{
		timer_clockFrequency = rccClock.PCLK2_Frequency;
	}
	return timer_clockFrequency;
}//eof
