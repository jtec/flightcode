/**
 *\file clMPU6050.cpp
 *\brief This Device driver uses an I2C interface to get rotation rate and
 *		 acceleration measurements data from an IMU chip MPU6050 (by Invensense).
 *\author Jan
 */

#include "../inc/clMPU6050.h"
#include "../inc/clAssistant.h"
#include "../inc/clTimebase.h"
#include "../inc/stm32f4xx_i2c.h"

/**
 * \brief Constructor, configures the IMU and the necessary peripherals.
 * TODO Pass pins and peripheral as arguments to the constructor.
 */
clMPU6050::clMPU6050(I2C_TypeDef* I2CToUse, GPIO_TypeDef* gpio_scl, uint16_t pin_scl, GPIO_TypeDef* gpio_sda, uint16_t pin_sda){
	this->i2c = I2CToUse;
	this->sck_port = gpio_scl;
	this->sck_pin = pin_scl;
	this->sda_port = gpio_sda;
	this->sda_pin = pin_sda;


	// Enable I2C and GPIO clocks.
	clAssistant::enableI2CClock(this->i2c);

	clAssistant::enableGPIOClock(this->sck_port);
	clAssistant::enableGPIOClock(this->sda_port);
	// TODO Make pin multiplexing work for every pin combination.
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	/* Configure I2C pins: SCL and SDA */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = this->sck_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(this->sck_port, &GPIO_InitStructure);

	/*!< Configure pins SDA pin*/
	GPIO_InitStructure.GPIO_Pin = this->sda_pin;
	GPIO_Init(this->sda_port, &GPIO_InitStructure);

	/* I2C configuration */
	this->i2cConfiguration.I2C_Mode = I2C_Mode_I2C;
	this->i2cConfiguration.I2C_DutyCycle = I2C_DutyCycle_2;
	this->i2cConfiguration.I2C_OwnAddress1 = MPU6050_DEFAULT_ADDRESS; // MPU6050 7-bit adress = 0x68, 8-bit adress = 0xD0;
	this->i2cConfiguration.I2C_Ack = I2C_Ack_Disable;
	this->i2cConfiguration.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	this->i2cConfiguration.I2C_ClockSpeed = I2CSpeed;

	this->timeForOneByte = (10*1000000)/I2CSpeed;	//  Compute time of one byte [µs], useful for timeouts.
	// FIXME I2C Slaves can stretch the clock, so time for one bit cannot be predicted by the master.
	this->timeForOneBit = (timeForOneByte/10)  + 1;	// +1 to account for rounding error.
	this->timeoutTimerStartTime[0] = 0;
	this->timeoutTimerStartTime[1] = 0;

	I2C_Init(this->i2c, &(this->i2cConfiguration));
	/* I2C Peripheral Enable */
	I2C_Cmd(this->i2c, ENABLE);
	// Initialize scale factors and biases:
	for(int i=0; i<3; i++){
		this->sensors.accelerometerBias[i] = 0;
		this->sensors.gyroBias[i] = 0;
	}
	this->sensors.accelerometerScaleFactor = 0;
	this->sensors.gyroScaleFactor = 0;
	//Calculated from the MPU6000 Product Specification:
	this->sensors.temperatureScaleFactor = 340;
	this->sensors.temperatureBias = 36.5323529;

	this->restartI2CBus();
	// Configure measurement ranges etc.
	this->configMPU();
}//eof

/**
 *\brief Destructor.
 */
clMPU6050::~clMPU6050()
{
	// TODO	Free memory?
}//eof

/**
 *	\brief Puts the current raw (= no temperature/bias/nonlinearity compensation) measurements (accelerations, angular velocities,
 *	see definition of mpu6050Output structure for units) into a structure you                              provide as a pointer.
 *	\param[in] output - Pointer to a structure to copy the data to.
 */
void clMPU6050::getRawMeasurements(mpu6050Output* output){
	static int16_t unscaledMeasurements[6] = {0};
	static int16_t unscaledTemperature = 0;
	// Get unscaled data, i.e. in LSBs.
	//this->GetRawAccelGyro(unscaledMeasurements, &unscaledTemperature);
	//this->configMPU();

	// Multiply with scale factors:
	for(int32_t i=0; i<3; i++){
		output->rawAcc[i] = this->sensors.accelerometerBias[i] + ((float)unscaledMeasurements[i])/this->sensors.accelerometerScaleFactor ;
		output->rawGyro[i] = this->sensors.gyroBias[i] + ((float)unscaledMeasurements[3 + i])/this->sensors.gyroScaleFactor;
	}
	output->temp = this->sensors.temperatureBias + ((float)unscaledTemperature) / this->sensors.temperatureScaleFactor;
	this->restartI2CBus();
	TimeBase::waitMicrosec(1000);
	output->rawGyro[0] = this->GetDeviceID();
	output->rawGyro[2] = 1;
}

/**
 * \brief Configures the MPU6050 sensor.
 */
void clMPU6050::configMPU(){
	SetSleepModeStatus(DISABLE);
	SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
	SetFullScaleGyroRange(MPU6050_GYRO_FS_500);
	SetFullScaleAccelRange(MPU6050_ACCEL_FS_4);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param AccelGyro 16-bit signed integer array of length 6
 * @see MPU6050_RA_ACCEL_XOUT_H
 */
void clMPU6050::GetRawAccelGyro(s16* AccelGyro, s16* temperature){
	static u8 tmpBuffer[14];
	if(!I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, tmpBuffer, MPU6050_RA_ACCEL_XOUT_H, 14)){
		// If the bus gets blocked by a slave, restart it.
		restartI2CBus();
		// Exit method, since we could not receive any valid data.
		return;
	}
	/* Get acceleration */
	for(int i=0; i<3; i++){
		AccelGyro[i]=((s16)((u16)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);
	}
	/* Get Angular rate */
	for(int i=4; i<7; i++){
		AccelGyro[i-1]=((s16)((u16)tmpBuffer[2*i] << 8) + tmpBuffer[2*i+1]);
	}
	// Get temperature:
	*temperature = ((s16)((u16)tmpBuffer[6] << 8) + tmpBuffer[7]);;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, FALSE otherwise
 */
bool clMPU6050::TestConnection(){
	if(GetDeviceID() == 0x34) //0b110100; 8-bit representation in hex = 0x34
		return true;
	else
		return false;
}
// WHO_AM_I register

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 * @see MPU6050_RA_WHO_AM_I
 * @see MPU6050_WHO_AM_I_BIT
 * @see MPU6050_WHO_AM_I_LENGTH
 */
uint8_t clMPU6050::GetDeviceID()
{
	uint8_t tmp;
	ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
	return tmp;
}
/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see MPU6050_GetClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void clMPU6050::SetClockSource(uint8_t source)
{
	WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see MPU6050_GetFullScaleGyroRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void clMPU6050::SetFullScaleGyroRange(uint8_t range)
{
	WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
	// Memorize range:
	float fullRangeLSB = 32768;
	switch(range){
	case MPU6050_GYRO_FS_250:
		this->sensors.gyroScaleFactor = fullRangeLSB/250;
		break;
	case MPU6050_GYRO_FS_500:
		this->sensors.gyroScaleFactor = fullRangeLSB/500;
		break;
	case MPU6050_GYRO_FS_1000:
		this->sensors.gyroScaleFactor = fullRangeLSB/1000;
		break;
	case MPU6050_GYRO_FS_2000:
		this->sensors.gyroScaleFactor = fullRangeLSB/2000;
		break;
	default:
		// To avoid erroneous measurements, set to zero to make the error more obvious.
		this->sensors.gyroScaleFactor = 0;
	}
}

// GYRO_CONFIG register

/** Get full-scale gyroscope range.
 * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 0 = +/- 250 degrees/sec
 * 1 = +/- 500 degrees/sec
 * 2 = +/- 1000 degrees/sec
 * 3 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
uint8_t clMPU6050::GetFullScaleGyroRange()
{
	uint8_t tmp;
	ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
	return tmp;
}
/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 * 0 = +/- 2g
 * 1 = +/- 4g
 * 2 = +/- 8g
 * 3 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see MPU6050_ACCEL_FS_2
 * @see MPU6050_RA_ACCEL_CONFIG
 * @see MPU6050_ACONFIG_AFS_SEL_BIT
 * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
 */
uint8_t clMPU6050::GetFullScaleAccelRange()
{
	uint8_t tmp;
	ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
	return tmp;
}
/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see MPU6050_GetFullScaleAccelRange()
 */
void clMPU6050::SetFullScaleAccelRange(uint8_t range)
{
	WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
	// Memorize range:
	float fullRangeLSB = 32768;
	switch(range){
	case MPU6050_ACCEL_FS_2:
		this->sensors.accelerometerScaleFactor = fullRangeLSB/2.0;
		break;
	case MPU6050_ACCEL_FS_4:
		this->sensors.accelerometerScaleFactor = fullRangeLSB/4.0;
		break;
	case MPU6050_ACCEL_FS_8:
		this->sensors.accelerometerScaleFactor = fullRangeLSB/8.0;
		break;
	case MPU6050_ACCEL_FS_16:
		this->sensors.accelerometerScaleFactor = fullRangeLSB/16.0;
		break;
	default:
		// To avoid erroneous measurements:
		this->sensors.accelerometerScaleFactor = 0;
	}
}

/** Get sleep mode status.
 * Setting the SLEEP bit in the register puts the device into very low power
 * sleep mode. In this mode, only the serial interface and internal registers
 * remain active, allowing for a very low standby current. Clearing this bit
 * puts the device back into normal mode. To save power, the individual standby
 * selections for each of the gyros should be used if any gyro axis is not used
 * by the application.
 * @return Current sleep mode enabled status
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
bool clMPU6050::GetSleepModeStatus()
{
	uint8_t tmp;
	ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
	if(tmp == 0x00)
		return false;
	else
		return true;
}
/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see MPU6050_GetSleepModeStatus()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void clMPU6050::SetSleepModeStatus(FunctionalState NewState)
{
	WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
}

/** Write multiple bits in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 */
void clMPU6050::WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
	//      010 value to write
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t tmp;
	if(!I2C_BufferRead(slaveAddr, &tmp, regAddr, 1)){
		restartI2CBus();
		return;
	}
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tmp &= ~(mask); // zero all important bits in existing byte
	tmp |= data; // combine data with existing byte
	I2C_ByteWrite(slaveAddr,&tmp,regAddr);
}
/** write a single bit in an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 */
void clMPU6050::WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t tmp;
	if(!I2C_BufferRead(slaveAddr, &tmp, regAddr, 1)){
		restartI2CBus();
		return;
	}
	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
	I2C_ByteWrite(slaveAddr,&tmp,regAddr);
}
/** Read multiple bits from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void clMPU6050::ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
	// 01101001 read byte
	// 76543210 bit numbers
	//    xxx   args: bitStart=4, length=3
	//    010   masked
	//   -> 010 shifted
	uint8_t tmp;
	if(!I2C_BufferRead(slaveAddr, &tmp, regAddr, 1)){
		restartI2CBus();
		return;
	}
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	tmp &= mask;
	tmp >>= (bitStart - length + 1);
	*data = tmp;
}

/** Read a single bit from an 8-bit device register.
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
 */
void clMPU6050::ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
	uint8_t tmp;
	if(!I2C_BufferRead(slaveAddr, &tmp, regAddr, 1)){
		restartI2CBus();
		return;
	}
	*data = tmp & (1 << bitNum);
}

/**
 * @brief  Writes one byte to the  MPU6050.
 * @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
 * @param  writeAddr : address of the register in which the data will be written
 * @return true, if the write operation could finished successfully, false if not (e.g. if the I2C bus was blocked by the sensor).
 */
bool clMPU6050::I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 writeAddr)
{

	// Disable automatic generation of STOP condition.
	I2C_GenerateSTOP(this->i2c, DISABLE);

	/* Send START condition */
	I2C_GenerateSTART(this->i2c, ENABLE);

	// Check if START condition has been sent by checking the SB bit in SR 1,
	// this flag is cleared by reading SR1 and writing to DR, this will be done when sending the slave address.
	bool startConditionSent = false;
	this->restartTimeoutTimer();
	while(!startConditionSent){
		startConditionSent = this->i2c->SR1 & BIT0;
		if(this->getTimeoutTimerTime() > 20*this->timeForOneByte){
			return false;
		}
	}

	// Send MPU6050 address for write
	I2C_Send7bitAddress(this->i2c, slaveAddr, I2C_Direction_Transmitter);

	// Wait until the slave has acknowledged the address. To do this, we check the ADDR bit
	// in SR1. This bit is cleared by subsequently reading SR1 and SR2.
	this->restartTimeoutTimer();
	bool addressHasBeenReceived = false;
	while(!addressHasBeenReceived){
		addressHasBeenReceived = this->i2c->SR1 & BIT1;
		if(this->getTimeoutTimerTime() > 20*this->timeForOneByte){
			return false;
		}
	}
	// Clear ADDR flag by reading SR2; variable is declared as volatile to keep the compiler from optimizing it away.
	volatile uint16_t tmp = this->i2c->SR2;

	// Send the MPU6050's internal address to write to
	I2C_SendData(this->i2c, writeAddr);

	// Wait until the byte transfer has been finished by checking on the BTF bit in SR1. It will
	// be cleared by the next read operation of the DR register.
	this->restartTimeoutTimer();
	bool byteHasBeenTransmitted = false;
	while(!byteHasBeenTransmitted){
		byteHasBeenTransmitted = this->i2c->SR1 & BIT2;
		if(this->getTimeoutTimerTime() > 20*this->timeForOneByte){
			return false;
		}
	}
	/* Send the byte to be written */
	I2C_SendData(this->i2c, *pBuffer);

	// Wait until the byte transfer has been finished by checking on the BTF bit in SR1. It will
	// be cleared by the final STOP condition.
	this->restartTimeoutTimer();
	byteHasBeenTransmitted = false;
	while(!byteHasBeenTransmitted){
		byteHasBeenTransmitted = this->i2c->SR1 & BIT2;
		if(this->getTimeoutTimerTime() > 20*this->timeForOneByte){
			return false;
		}
	}

	I2C_GenerateSTOP(this->i2c, ENABLE);
	return true;

}

/**
 * @brief  Reads a block of bytes from the MPU6050.
 * @param  slaveAddr  : slave address
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param  readAddr : MPU6050's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Magnetometer readinf).
 * @return true, if the data could be read- false, if reading data failed, e.g. because the MPU6050 did not answer.
 */

bool clMPU6050::I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
	// Disable automatic generation of STOP condition.
	I2C_GenerateSTOP(this->i2c, DISABLE);

	// Wait while the bus is busy
	this->restartTimeoutTimer();
	while(I2C_GetFlagStatus(this->i2c, I2C_FLAG_BUSY)){
		if(this->getTimeoutTimerTime() > this->timeForOneByte){
			return false;
		}
	}

	// Send START condition
	I2C_GenerateSTART(this->i2c, ENABLE);

	// Check if START condition has been sent by checking the SB bit in SR 1,
	// this flag is cleared by reading SR1 and writing to DR, this will be done when sending the slave address.
	bool startConditionSent = false;
	this->restartTimeoutTimer();
	while(!startConditionSent){
		startConditionSent = (this->i2c->SR1 & BIT0) > 0;
		if(this->getTimeoutTimerTime() > this->timeForOneByte){
			return false;
		}
	}

	// Send MPU6050 address for write
	I2C_Send7bitAddress(this->i2c, slaveAddr, I2C_Direction_Transmitter);

	// Wait until the slave has acknowledged the address. To do this, we check the ADDR bit
	// in SR1. This bit is cleared by subsequently reading SR1 and SR2.
	this->restartTimeoutTimer();
	bool addressHasBeenReceived = false;
	while(!addressHasBeenReceived){
		addressHasBeenReceived = (this->i2c->SR1 & BIT1) > 0;
		if(this->getTimeoutTimerTime() > 2*this->timeForOneByte){
			return false;
		}
	}
	// Clear ADDR flag by reading SR2; variable is declared as volatile to keep the compiler from optimizing it away.
	volatile uint16_t tmp = this->i2c->SR2;
	// Send the MPU6050's internal address to read from
	I2C_SendData(this->i2c, readAddr);

	// Wait until the byte transfer has been finished by checking on the BTF bit in SR1. It will
	// be cleared by the next read operation of the DR register.
	this->restartTimeoutTimer();
	bool byteHasBeenTransmitted = false;
	while(!byteHasBeenTransmitted){
		byteHasBeenTransmitted = (this->i2c->SR1 & BIT2) > 0;
		if(this->getTimeoutTimerTime() > 2*this->timeForOneByte){
			return false;
		}
	}

	// Send START condition a second time
	I2C_GenerateSTART(this->i2c, ENABLE);

	// Check if START condition has been sent by checking the SB bit in SR1,
	// this flag is cleared by reading SR1 and writing to DR, this will be done when sending the slave address.
	startConditionSent = false;
	this->restartTimeoutTimer();
	while(!startConditionSent){
		startConditionSent = (this->i2c->SR1 & BIT0) > 0;
		if(this->getTimeoutTimerTime() > this->timeForOneByte){
			return false;
		}
	}

	// Send MPU6050 address for read
	I2C_Send7bitAddress(this->i2c, slaveAddr, I2C_Direction_Receiver);

	// Wait until the slave has acknowledged the address. To do this, we check the ADDR bit
	// in SR1. This bit is cleared by subsequently reading SR1 and SR2.
	this->restartTimeoutTimer();
	addressHasBeenReceived = false;
	while(!addressHasBeenReceived){
		addressHasBeenReceived = (this->i2c->SR1 & BIT1) > 0;
		if(this->getTimeoutTimerTime() > 2*this->timeForOneByte){
			return false;
		}
	}
	// Clear ADDR flag by reading SR2:
	tmp = this->i2c->SR2;

	// Read in bytes sent by the slave:
	// If we are about to receive 1 single byte, there is no need to acknowledge it, since the last byte
	// is not acknowledged. If we will receive more than one byte, every byte except the last one has to be acknowledged
	// by the master.
	I2C_AcknowledgeConfig(this->i2c, ENABLE);
	if(NumByteToRead == 1){
		I2C_AcknowledgeConfig(this->i2c, DISABLE);
	}
	// While there is data to be read:
	uint32_t timeoutTime = 2* NumByteToRead * this->timeForOneByte;
	this->restartTimeoutTimer();
	while(NumByteToRead > 0)
	{
		// Check for timeout:
		if(this->getTimeoutTimerTime() > timeoutTime){
			return false;
		}

		// When we are about to receive the last byte:
		if(NumByteToRead == 1)
		{
			// Disable Acknowledgement:
			I2C_AcknowledgeConfig(this->i2c, DISABLE);
		}

		// Check whether a byte has been received:
		bool newByteReceived = (this->i2c->SR1 & BIT6) > 0;
		if(newByteReceived)
		{
			newByteReceived = false;
			// Read a byte from the MPU6050
			*pBuffer = this->i2c->DR;

			// Point to the location where the next byte will be saved.
			pBuffer++;

			// Decrement the read bytes counter.
			NumByteToRead--;
		}
	}
	// We have received all bytes, send STOP condition.
	I2C_GenerateSTOP(this->i2c, ENABLE);

	return true;
}

/**
 * \brief Call this method before you start an operation that could time out.
 */
void clMPU6050::restartTimeoutTimer(){
	TimeBase::getSystemTime(this->timeoutTimerStartTime);
}

/**
 * \brief Return value indicates how much time [µs] has passed since the timeout timer has been restarted.
 */
uint32_t clMPU6050::getTimeoutTimerTime(){
	static uint32_t now[2] = {0};
	static uint32_t returnValue = 0;
	TimeBase::getSystemTime(now);
	returnValue = (now[0] - this->timeoutTimerStartTime[0])*1000 + (now[1] - this->timeoutTimerStartTime[1]);
	return returnValue;
}

/**
 * \brief Return value indicates how much time [time for one bit] has passed since the timeout timer has been restarted.
 */
uint32_t clMPU6050::getTimeoutTimerTimeInBits(){
	static uint32_t now[2] = {0};
	static uint32_t timeInMicrosecs = 0;
	TimeBase::getSystemTime(now);
	timeInMicrosecs = (now[0] - this->timeoutTimerStartTime[0])*1000 + (now[1] - this->timeoutTimerStartTime[1]);
	return timeInMicrosecs/this->timeForOneBit;
}

/**
 * Writes 8 bit of data to the bus.
 */
void clMPU6050::writeByte(uint8_t byte){

}

/**
 * \brief This method sends a stop condition to make all slaves restart their bus state machines. It has
 * become necessary because sometimes the MPU6050 blocks the bus.
 */
void clMPU6050::restartI2CBus(){

	//Disable I2C
	I2C_Cmd(this->i2c, DISABLE);

	// Configure SDA and SDL as GPIOs t be able to send a STOP condition manually.
	static GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = this->sck_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(this->sck_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = this->sda_pin;
	GPIO_Init(this->sda_port, &GPIO_InitStructure);

	/*
	// Reset and restart I2C peripheral
	I2C_SoftwareResetCmd(this->i2c, ENABLE);
	this->restartTimeoutTimer();
	while(this->getTimeoutTimerTime() < this->timeForOneByte){
	}
	I2C_SoftwareResetCmd(this->i2c, DISABLE);
	 */

	// Generate STOP condition to reset slave I2C interfaces.
	GPIO_ResetBits(this->sda_port, this->sda_pin);
	TimeBase::waitMicrosec(this->timeForOneBit);
	GPIO_SetBits(this->sck_port, this->sck_pin);
	TimeBase::waitMicrosec(this->timeForOneBit);
	GPIO_SetBits(this->sda_port, this->sda_pin);
	TimeBase::waitMicrosec(this->timeForOneBit);
	GPIO_ResetBits(this->sck_port, this->sck_pin);
	TimeBase::waitMicrosec(this->timeForOneBit);
	GPIO_ResetBits(this->sda_port, this->sda_pin);

	// Reconfigure pins for I2C:
	GPIO_InitStructure.GPIO_Pin = this->sck_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(this->sck_port, &GPIO_InitStructure);

	/*!< Configure pins SDA pin*/
	GPIO_InitStructure.GPIO_Pin = this->sda_pin;
	GPIO_Init(this->sda_port, &GPIO_InitStructure);

	// Re-enable I2c peripheral.
	I2C_Cmd(this->i2c, ENABLE);

	return;
}

/**
 * \brief Initializes all values of a measurement structure to zero.
 * \param s - s structure this method is supposed to initialize.
 */
void clMPU6050::initStructure(mpu6050Output* s){
	for(uint8_t i=0; i<3; i++){
		s->rawAcc[i] = 0;
		s->rawGyro[i] = 0;
	}
	s->temp = 0;
}
