 /*************************************
 * init.h
 * author: jan
 *
 * description:
 **************************************/

#ifndef STM32_INIT_PERIPHERALS_H_
#define STM32_INIT_PERIPHERALS_H_

/*included header files***************/
#include "../../hardwareAccessLayer/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx.h"
#include "../../convenienceLayer/inc/clAssistant.h"

/*function prototypes*****************/

void Init(void);
void NVIC_Configuration(void);


#endif /* STM32_INIT_PERIPHERALS_H_ */
