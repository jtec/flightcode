/****************************************************************************************************
 * \file clFactory.cpp
 * \brief This class creates all drivers to access all components the used hardware consists of.
 *
 * 	      e.g. USART,LED,...
 * *
 * \author jan,martin
 * ****************************************************************************************************
 */

#include "../inc/clFactory.h"

/**
 * \brief Private constructor.
 */
clFactory::clFactory(){
}

/**
 * \brief Returns an instance of type clFactory configured for a servo node. Creates and configures
 * all the necessary hardware drivers. In case of a different hardware setup
 * (e.g. different chip pinout, different chip type), just modify this function or write a custom one.
 * This way the high level layers using this factory don't have to deal with any hardware details, e.g. what baudrate
 * the USART used for the bus has or which GPIO pin USART_Tx is connected to.
 */
clFactory* clFactory::buildForServoNode(){
	clFactory* fact = new clFactory();
	fact->RS485Transceiver = new clHVD78();

	// Configuration for the PWM driver constructor:
	ServoBankBuildConfiguration servoBankConfig;
	servoBankConfig.numberOfChannels = 5;
	// Standard RC servos expect PWM with 50 Hz update rate.
	servoBankConfig.frequency = 50;
	for(uint8_t i=0; i<	servoBankConfig.numberOfChannels; i++){
		// Set every channel to the neutral position.
		servoBankConfig.initialPulseWidths[i] = 1500000;
	}
	// Set channel 1 (throttle) to zero.
	servoBankConfig.initialPulseWidths[0] = 1000000;
	// Define ports and pins to use.
	servoBankConfig.pins[0] = GPIO_Pin_6;
	servoBankConfig.ports[0] = GPIOC;

	servoBankConfig.pins[1] = GPIO_Pin_7;
	servoBankConfig.ports[1] = GPIOC;

	servoBankConfig.pins[2] = GPIO_Pin_8;
	servoBankConfig.ports[2] = GPIOC;

	servoBankConfig.pins[3] = GPIO_Pin_9;
	servoBankConfig.ports[3] = GPIOC;

	servoBankConfig.pins[4] = GPIO_Pin_0;
	servoBankConfig.ports[4] = GPIOA;
	// Define timers: for 5 channels we need 2 timers because they have 4 channels each.
	servoBankConfig.timers[0] = TIM3;
	servoBankConfig.timers[1] = TIM5;
	// Build PWM driver:
	fact->ServoBank = new clPWMServoBank(&servoBankConfig);
	//FIXME Integrate into PWM driver.
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);

	fact->statusLED = new clLED(GPIOD, GPIO_Pin_12, clGPIOPin::HIGH);

	fact->SerialPortBus = new clSerialPort(USART2, 2500000, GPIOA, GPIOA, GPIO_Pin_2, GPIO_Pin_3);

	//FIXME Integrate into hardware driver.

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	fact->SerialPortXBee = new clSerialPort(USART3, 111111, GPIOB, GPIOB, GPIO_Pin_10, GPIO_Pin_11);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	fact->SpektrumSatellite = new clSpektrumSatellite(USART1, GPIOA, GPIO_Pin_10);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	// Create MPU6050 IMU:
	fact->imuMPU6050 = new clMPU6050(I2C1, GPIOB, GPIO_Pin_6, GPIOB, GPIO_Pin_7);

	return fact;
}

/**
 * \brief Makes the clFactory configure its members for a mainNode application.
 */
clFactory* clFactory::buildForMainNode(){
	// In project phase I all nodes are the same hardware.
	return clFactory::buildForServoNode();
}

/**
 * \brief Makes the clFactory configure its members for a stand alone autopilot.
 */
clFactory* clFactory::buildForStandaloneNode(){
	// In project phase I all nodes are the same hardware.
	return clFactory::buildForServoNode();
}

/**
 * \brief Makes the clFactory configure its members for a groundLinkNode application.
 */
clFactory* clFactory::buildForGroundLinkNode(){
	// In project phase I all nodes are the same hardware.
	return clFactory::buildForServoNode();
}

/**
 * \brief Makes the clFactory configure its members for the ground control station adapter.
 */
clFactory* clFactory::buildForGroundStation(){
	// The ground control station adapter is based on a STM32F4Discovery board, but we
	//can use the same setup.
	return clFactory::buildForServoNode();

}

/**
 * \brief
 */
clFactory::~clFactory(void){}

/**
 * \brief Returns a pointer to the boards's PWM output driver
 * \return  a pointer to a clPWMServoBank object.
 */
clPWMServoBank* clFactory::getPWMServoBank(){
	return this->ServoBank;
}

/**
 * \brief Returns a pointer to the boards's Status LED.
 * \return  a pointer to a clLED object.
 */
clLED* clFactory::getStatusLed(){
	return this->statusLED;
}

/**
 * \brief Returns a pointer to a Serial port that may be used for an XBee modem.
 * \return  a pointer to a SerialPort object.
 */
clSerialPort* clFactory::getSerialPortXBee(){
	return this->SerialPortXBee;
}

/**
 * \brief Returns a pointer to a Serial port that may be used to communicate with the RS485 bus.
 * \return  a pointer to a SerialPort object.
 */
clSerialPort* clFactory::getSerialPortBus(){
	return this->SerialPortBus;
}

/**
 * \brief Returns a pointer to a Satellite receiver driver.
 * \return  a pointer to a clSpektrumSatellite object.
 */
clSpektrumSatellite* clFactory::getSpektrumSatellite(){
	return this->SpektrumSatellite;
}

/**
 * \brief Returns a pointer to a RS485 Transceiver driver.
 * \return  a pointer to a clHVD78 object.
 */
clHVD78* clFactory::getRS485Transceiver(){
	return this->RS485Transceiver;
}

/**
 * \brief Returns a pointer to a MPU6050 iu chip driver.
 * \return  a pointer to a clMPU6050 object.
 */
clMPU6050* clFactory::getMPU6050(){
	return this->imuMPU6050;
}

