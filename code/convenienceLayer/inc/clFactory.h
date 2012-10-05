/****************************************************************************************************
 * \file clFactory.h
 * \brief This class creates all drivers to access all components the used hardware consists of.
 *
 * 	      e.g. USART,LED,...
 *
 * 	      example: To use the status LED in a higher layer just define
 * 	      clFactory* fakultaet = new clFactory();
 * 		  clLED* led = fakultaet->getStatusLed();
 *
 * *
 * \author jan,martin
 * ****************************************************************************************************
 */

#ifndef CLFACTORY_H_
#define CLFACTORY_H_


#include "clHVD78.h"
#include "clPWMServoBank.h"
#include "clLED.h"
#include "clSerialPort.h"
#include "clSpektrumSatellite.h"
#include "clMPU6050.h"
#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"

class clFactory
{
	public:
		clPWMServoBank* getPWMServoBank();
		clLED* getStatusLed();
		clSerialPort* getSerialPortXBee();
		clSerialPort* getSerialPortBus();
		clSpektrumSatellite* getSpektrumSatellite();
		clHVD78* getRS485Transceiver();
		clMPU6050* getMPU6050();

		static clFactory* buildForMainNode();
		static clFactory* buildForStandaloneNode();
		static clFactory* buildForServoNode();
		static clFactory* buildForGroundLinkNode();
		static clFactory* buildForGroundStation();

	private:
		clFactory();							//constructor
		~clFactory(void);						//destructor
		clLED* statusLED;
		clSerialPort* SerialPortXBee;
		clSerialPort* SerialPortBus;
		clPWMServoBank* ServoBank;
		clSpektrumSatellite* SpektrumSatellite;
		clHVD78* RS485Transceiver;
		clMPU6050* imuMPU6050;
};



#endif /* CLFACTORY_H_ */
