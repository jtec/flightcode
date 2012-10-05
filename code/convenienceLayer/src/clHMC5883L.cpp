/*
 * \file clHMC5883L.cpp
 * \brief Device driver for the magnetometer IC HMC5883L.
 * \author jan
 * TODO Finish and test driver.
 */

#include "../inc/clHMC5883L.h"
#include "../inc/clAssistant.h"
#include "../inc/clTimebase.h"

/**
 * \brief Constructor, configures the sensor and the necessary peripherals.
 * TODO Pass pins and peripheral (SPI1, SPI2, ...) to the constructor.
 */
clHMC5883L::clHMC5883L(){
	GPIO_InitTypeDef GPIO_InitStructure;
	  I2C_InitTypeDef I2C_InitStructure;
/*
	  // Enable Clk
	  RCC_APB1PeriphClockCmd(IMU_I2C_CLK, ENABLE);
	  RCC_AHB1PeriphClockCmd(IMU_I2C_SCL_GPIO_CLK | IMU_I2C_SDA_GPIO_CLK, ENABLE);

	  // Performing a Reset
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	  RCC_APB1PeriphResetCmd(IMU_I2C_CLK, ENABLE);
	  RCC_APB1PeriphResetCmd(IMU_I2C_CLK, DISABLE);

	  // Remap Pins
	  GPIO_PinAFConfig(IMU_I2C_SCL_GPIO_PORT, IMU_I2C_SCL_SOURCE, IMU_I2C_SCL_AF);
	  GPIO_PinAFConfig(IMU_I2C_SDA_GPIO_PORT, IMU_I2C_SDA_SOURCE, IMU_I2C_SDA_AF);

	  // Initialize SCL
	  GPIO_InitStructure.GPIO_Pin = IMU_I2C_SCL_PIN;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	  GPIO_Init(IMU_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

	  // Initialize SDA
	  GPIO_InitStructure.GPIO_Pin = IMU_I2C_SDA_PIN;
	  GPIO_Init(IMU_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

	  // I2C configuration
	  I2C_InitStructure.I2C_Mode        = I2C_Mode_I2C;
	  I2C_InitStructure.I2C_DutyCycle   = I2C_DutyCycle_2;
	  I2C_InitStructure.I2C_OwnAddress1 = 0;
	  I2C_InitStructure.I2C_Ack         = I2C_Ack_Enable;
	  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	  I2C_InitStructure.I2C_ClockSpeed = 100000;

	  I2C_Cmd(IMU_I2C, ENABLE);
	  I2C_Init(IMU_I2C, &I2C_InitStructure);
	  */
}

/**
 * \brief Destructor.
 */
clHMC5883L::~clHMC5883L(){


}
