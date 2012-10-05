 /*************************************
 * stm32_test.h
 * author: jan
 * last modifications: 
 * 
 * description: 
 **************************************/
#ifndef STM32_TEST_H_
#define STM32_TEST_H_

 /*Includes*********************************/
#include "../../libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/stm32f10x.h"
#include "../HardwareAbstractionLayer/deviceDrivers/inc/deviceDriver_LED_CPP.h"
#include "../HardwareAbstractionLayer/inc/serviceFunctions.h"

//test object
class Test{
public:
	Test();
	~Test();
	uint8_t testV;
	uint8_t testV_2;
	uint8_t testV_3;
	void testM(void);
};
 /*Definitions******************************/

 /*function prototypes********************************/
void testloop();
uint32_t runtimetest();


#endif /* STM32_TEST_H_ */
