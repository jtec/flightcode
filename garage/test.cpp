 /*************************************
 * stm32_test.cpp
 * author: jan
 * last modifications: 
 * 
 * description: 
 **************************************/

 /*Includes*********************************/
#include "CANTestcase.h"
#include "../inc/test.h"
#include "../inc/main.h"
#include "../HardwareAbstractionLayer//deviceDrivers/inc/devices.h"
#include "../HardwareAbstractionLayer//deviceDrivers/inc/deviceDriver_IMUchr6d.h"
#include "../HardwareAbstractionLayer//deviceDrivers/inc/deviceDriver_MPU6000.h"
#include "../HardwareAbstractionLayer/inc/modules.h"
#include "../HardwareAbstractionLayer/inc/NodeLink.h"
#include "../HardwareAbstractionLayer/inc/CANTest.h"
#include "../HardwareAbstractionLayer/inc/GroundLink.h"
#include "../HardwareAbstractionLayer/inc/MAVLink.h"
#include "../HardwareAbstractionLayer/inc/UAVTalk.h"
#include "../HardwareAbstractionLayer/examples/inc/SerialPort_example.h"


 /*Definitions******************************/

 /*Functions********************************/
/*
 * @brief Constructor.
 */
Test::Test(){

}
/*
 * @brief Destructor.
 */
Test::~Test(){

}

/**
 * @brief Implementation of 'test()' method of 'Test' class.
 */
void Test::testM(void){
	return;
}//eof

void testloop()
{
//select which loop routine you want to execute
//#define TEST_GROUNDLINK
//#define TEST_UAVTALK
//#define TEST_MAVLINK
//#define TEST_ARDUIMU_V3
//#define TEST_NODELINK
//#define TEST_BLINK_APBOARD
//#define TEST_SERVOS
//#define TEST_QUADCOPTER
//#define TEST_SPEKTRUMSATELLITE
//#define TEST_IMU_CHR6D

#ifdef TEST_QUADCOPTER
	/* Basic quadrocopter nick and roll damping. Sequence of events:
	* 	1.) Reads gyros to get d(nick)/dt and d(roll)/dt in °/s
	* 	2.) Gets RC receiver input ([-1000, +1000]) to determine
	* 		a) the desired vertical acceleration [m/s²]
	* 		b) the desired pitch and roll rate [°/s]
	*	3.) calculates
	*		a) the necessary level of thrust for all 4 engines to achieve the desired vertical acceleration (open loop, for now)
	*		b) the necessary level of thrust offsets for the left/right and leading/rear engine. E.g. for a desired nick
	*			rate of 10°/s that may be +120/-120
	*	4.) Sets the calculated gas positions.
	*/
	int32_t v = 0;
	while(true){
		TimeBase::waitMicrosec(100000);
		//led->toggle();
	//	gLink->sendValue(v);
		v--;
	}
#endif

#ifdef TEST_ARDUIMU_V3
	ArduIMU_V3* imu = new ArduIMU_V3();
	Devices::backPointer->statusLED.set(LED::state_OFF);
	while(true){
		imu->test();
		TimeBase::waitMicrosec(500000);
		Devices::backPointer->statusLED.toggle();
	}
#endif

#ifdef TEST_MAVLINK
	LED* lede = new LED(GPIOC, GPIO_Pin_12, LED::state_ON);
	MAVLink* mavlink = new MAVLink(USART2);
	ArduIMU_V3* imu = new ArduIMU_V3();
	lede->set(LED::state_OFF);
	uint32_t heartBeatTime = TimeBase::getSystemTimeMs();
	uint32_t airdataTime = TimeBase::getSystemTimeMs();
	uint32_t newTime = 0;
	while(true){
		newTime = TimeBase::getSystemTimeMs();
		// Do every second:
		if((newTime - heartBeatTime)>1000)
		{
			mavlink->sendHeartbeat();
		//	lede->toggle();
			heartBeatTime = newTime;
		}
		//Do every 10 milliseconds
		if((newTime-airdataTime)>100)
		{
			mavlink->sendAirData((uint8_t)(imu->values.temp));
			airdataTime = newTime;
		}
	}
#endif

#ifdef TEST_UAVTALK
	LED* lede = new LED(GPIOC, GPIO_Pin_12, LED::state_ON);
	UAVTalk* talk = new UAVTalk(USART2);
	int32_t v = 0;
	lede->set(LED::state_OFF);
	while(true){
		TimeBase::waitMicrosec(500000);

		lede->set(LED::state_ON);
		TimeBase::waitMicrosec(1000);
		lede->set(LED::state_OFF);

		talk->sendTestMessage();
		v--;
	}
#endif

#ifdef TEST_GROUNDLINK
	LED* led = new LED(GPIOC, GPIO_Pin_12, LED::state_ON);
	GroundLink* gLink = new GroundLink(USART2);
	int32_t v = 0;
	led->set_GPIOC_10(0);
	while(true){
		TimeBase::waitMicrosec(500000);
		led->toggle();
		TimeBase::waitMicrosec(500000)
		gLink->sendRawInertial();
		v--;
	}
#endif

#ifdef TEST_NODELINK
	init_can();
	while(true){
		TimeBase::waitMicrosec(100000);
		can_send_test();
		Devices::backPointer->statusLED.toggle();
	}
#endif

#ifdef TEST_IMU_CHR6D

while(true)
{
	IMU_CHR6D* imu = new IMU_CHR6D(USART3);
	imu->testLoop();
}

#endif

#ifdef TEST_SERVOS

bool setPulsewidth = false;
bool setPosition = true;

uint32_t servoBankGpioPins[2] = { (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7),
								   (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_6| GPIO_Pin_7| GPIO_Pin_8| GPIO_Pin_9)
								};
GPIO_TypeDef* servoBankGPIOs[2] = {GPIOA, GPIOB};
TIM_TypeDef* servoBankTimers[3] = {TIM2, TIM3, TIM4};
uint32_t defaultChannelWidths[12] = {1000000, 1000000, 1000000, 1000000,
										1000000, 1000000, 1000000, 1000000,
										1000000, 1000000, 1000000, 1000000, };
ServoBank* servoBank = new ServoBank (12,
									servoBankTimers, 3,
									servoBankGPIOs,
									&servoBankGpioPins[0],
									2,
									20000000,
									defaultChannelWidths);

uint32_t pwidth[12] = {0};
int16_t position[12] = {0};
//vary servo position from -100% to +100%
while(1)
{
	if(setPosition){
	for(int16_t p=-1000; p<1000; p+=5){
		for(uint8_t i=0; i<12; i++){
				position[i]= p;
			}
		servoBank->setPositions(position);
		TimeBase::waitMicrosec(5000);
	}
	}
	else if(setPulsewidth){
		for(uint32_t pwi= 1000000; pwi<2000000; pwi+=300){
			for(uint8_t i=0; i<12; i++){
					pwidth[i]= pwi;
				}
			servoBank->setPWMPulseWidth(pwidth);
			TimeBase::waitMicrosec(5000);
		}
		}

}

#endif

#ifdef TEST_BLINK_APBOARD

SerialPort* s = new SerialPort(USART1, 115200);
LED* led = new LED(GPIOC, GPIO_Pin_12, LED::state_ON);

while(1)
{
	led->toggle();
	s->sendTestMessage();
	TimeBase::waitMicrosec(600000);
}

#endif

#ifdef TEST_BLINK

while(1)
{
	TimeBase::waitMicrosec(500000);
	Devices::backPointer->statusLED.toggle();
	Modules::backPointer->messagePort.sendTestMessage();
}

#endif


#ifdef TEST_SPEKTRUMSATELLITE
//GroundLink* gLink = new GroundLink(USART1);
uint16_t counter = 0;
int16_t rcChannels[12] = {0};

while(1){
	counter++;
	Devices::backPointer->spektrumSatellite.getRcChannels(rcChannels);
	Devices::backPointer->servoBank.setPositions(rcChannels);
//	gLink->sendValue((int32_t)rcChannels[2]*1000);
	TimeBase::waitMicrosec(10000);
	if(counter>=10)
	{
		Devices::backPointer->statusLED.toggle();
		counter = 0;
	}
}
#endif

return;
}//eof

uint32_t runtimetest(){
	uint32_t count = 0;
	uint32_t a = 0;
	float b = 2.78;
	float c = 3.14;
	uint32_t  *DWT_CYCCNT  = (uint32_t *)0xE0001004; //address of the register
	uint32_t  *DWT_CONTROL = (uint32_t *)0xE0001000; //address of the register
	uint32_t  *SCB_DEMCR   = (uint32_t *)0xE000EDFC; //address of the register

	*SCB_DEMCR = *SCB_DEMCR | 0x01000000;
	*DWT_CYCCNT = 0; // reset the counters
	*DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
	*DWT_CYCCNT = 0; // reset the counter
	//cycles are counted beginning from here
	// 0 cycles
		a += 1;
	//8 cycles
		a += 1;
	//13 cycles
		a *= 3;
	//20 Cycles
		b *= c;
	//78 cycles
		b /= c;
	//302 cycles
		b /= c;
	//527 cycles

	count = *DWT_CYCCNT;

	return count;
}
