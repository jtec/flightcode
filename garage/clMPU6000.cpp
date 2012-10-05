/**
 *\file clMPU6000.cpp
 *\brief The Device driver uses an SPI interface to get rotation rate and
 *		 acceleration measurements data from an IMU chip MPU6000 (by Invensense).
 *\author Jan
 */

//Inclusions
#include "../inc/clMPU6000.h"
#include "../inc/clServiceFunctions.h"
#include "../inc/clTimebase.h"

/**
 * \brief Constructor, configures the IMU and the necessary peripherals.
 * TODO Hand pins and peripheral (SPI1, SPI2, ...) over to the constructor.
 */
clMPU6000::clMPU6000()
{
	this->miso_port = GPIOA;
	this->miso_pin = GPIO_Pin_6;

	this->mosi_port = GPIOA;
	this->mosi_pin = GPIO_Pin_7;

	this->sclk_port = GPIOA;
	this->sclk_pin = GPIO_Pin_5;

	this->cs_port = GPIOC;
	this->cs_pin = GPIO_Pin_3;

	this->spi = SPI1;

	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable the SPI clock */
	// TODO Integrate SPI2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable CS, SCK, MOSI and MISO GPIO clocks */
	Service::enableGPIOClock(this->miso_port);
	Service::enableGPIOClock(this->mosi_port);
	Service::enableGPIOClock(this->sclk_port);
	Service::enableGPIOClock(this->cs_port);

	GPIO_PinAFConfig(this->sclk_port, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(this->miso_port, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(this->mosi_port, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = this->sclk_pin;
	GPIO_Init(this->sclk_port, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  this->mosi_pin;
	GPIO_Init(this->mosi_port, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = this->miso_pin;
	GPIO_Init(this->miso_port, &GPIO_InitStructure);

	// CS pin configuration
	this->csPin = new LED(this->cs_port, this->cs_pin, LED::state_OFF);

	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(this->spi);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(this->spi, &SPI_InitStructure);

	/* Enable SPI1  */
	SPI_Cmd(this->spi, ENABLE);

	/* Deselect : Chip Select high */
	this->csPin->set(LED::state_OFF);

	this->configMPU();
}//eof

/**
 * \brief Configures the MPU6000 sensor.
 */
void clMPU6000::configMPU(){

	uint16_t waittime = 1000; // [µs]
	// Chip reset
	this->spiWrite(MPUREG_PWR_MGMT_1, BIT_H_RESET);
	TimeBase::waitMicrosec(100000);;  // Startup time delay
	// Wake Up device and select GyroZ clock (better performance)
	this->spiWrite(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
	TimeBase::waitMicrosec(waittime);
	// Disable I2C bus (recommended on datasheet)
	this->spiWrite(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
	TimeBase::waitMicrosec(waittime);
	// SAMPLE RATE
	this->spiWrite(MPUREG_SMPLRT_DIV,0x04);     // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz
	TimeBase::waitMicrosec(waittime);
	// FS & DLPF   FS=1000º/s, DLPF = 42Hz (low pass filter)
	this->spiWrite(MPUREG_CONFIG, BITS_DLPF_CFG_42HZ);
	TimeBase::waitMicrosec(waittime);
	this->spiWrite(MPUREG_GYRO_CONFIG,BITS_FS_1000DPS);  // Gyro scale 1000º/s
	TimeBase::waitMicrosec(waittime);
	this->spiWrite(MPUREG_ACCEL_CONFIG,0x08);   // Accel scale 4g (4096LSB/g)
	TimeBase::waitMicrosec(waittime);
	// INT CFG => Interrupt on Data Ready
	this->spiWrite(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);        // INT: Raw data ready
	TimeBase::waitMicrosec(waittime);
	this->spiWrite(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);        // INT: Clear on any read
	TimeBase::waitMicrosec(waittime);
}


/**
 *\brief Destructor.
 */
clMPU6000::~clMPU6000()
{
	// TODO	Free memory?
}//eof

/**
 * \brief TODO doc
 */
void clMPU6000::initStructure(mpu6000Output* s){
	for(uint8_t i=0; i<3; i++){
		s->rawAcc[i] = 0;
		s->rawGyro[i] = 0;
	}
	s->temp = 0;
}

// MPU6000 SPI functions
uint8_t clMPU6000::spiRead(uint8_t reg)
{
  uint8_t dump;
  uint8_t  return_value;
  uint8_t addr = reg | 0x80; // Set most significant bit

  this->csPin->set(LED::state_ON);

  	  /* Loop while DR register is not empty */
    while (SPI_I2S_GetFlagStatus(this->spi, SPI_I2S_FLAG_TXE) == RESET)
    {
    }
    /* Send a Byte defining the register that is supposed to be read */
    SPI_I2S_SendData(this->spi, addr);
    /* Wait to receive a Byte */
    while (SPI_I2S_GetFlagStatus(this->spi, SPI_I2S_FLAG_RXNE) == RESET)
    {
    }
    dump = SPI_I2S_ReceiveData(this->spi);

	  /* Loop while DR register is not empty */
  while (SPI_I2S_GetFlagStatus(this->spi, SPI_I2S_FLAG_TXE) == RESET)
  {
  }
  /* Send a Byte to receive a byte */
  SPI_I2S_SendData(this->spi, 0);
  /* Wait to receive a Byte */
  while (SPI_I2S_GetFlagStatus(this->spi, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }
  return_value = SPI_I2S_ReceiveData(this->spi);

  this->csPin->set(LED::state_OFF);

  return(return_value);
}

void clMPU6000::spiWrite(uint8_t reg, uint8_t data)
{
	this->csPin->set(LED::state_ON);
   	  /* Loop while DR register is not empty */
     while (SPI_I2S_GetFlagStatus(this->spi, SPI_I2S_FLAG_TXE) == RESET)
     {
     }
     /* Send the target register */
     SPI_I2S_SendData(this->spi, reg);
  	  /* Loop while DR register is not empty */
    while (SPI_I2S_GetFlagStatus(this->spi, SPI_I2S_FLAG_TXE) == RESET)
    {
    }
    /* Send the new target register content*/
    SPI_I2S_SendData(this->spi, data);

    this->csPin->set(LED::state_OFF);
}

/**
 * \brief Puts the latest sensor outputs into a given structure.
 * \param[in] o pointer to a structure to write the data to.
 */
void clMPU6000::read(mpu6000Output* o)
{
  uint8_t byte_H;
  uint8_t byte_L;
  uint16_t accelX;
  uint16_t accelY;
  uint16_t accelZ;
  uint16_t gyroX;
  uint16_t gyroY;
  uint16_t gyroZ;
  uint16_t temp;

 // if (MPU6000_newdata == true)   // Wait until we have new data from MPU6000 (INT6 interrupt)

  // Read AccelX
    byte_H = this->spiRead(MPUREG_ACCEL_XOUT_H);
    byte_L = this->spiRead(MPUREG_ACCEL_XOUT_L);
    accelX = (byte_H<<8)| byte_L;
    // Read AccelY
    byte_H = this->spiRead(MPUREG_ACCEL_YOUT_H);
    byte_L = this->spiRead(MPUREG_ACCEL_YOUT_L);
    accelY = (byte_H<<8)| byte_L;
    // Read AccelZ
    byte_H = this->spiRead(MPUREG_ACCEL_ZOUT_H);
    byte_L = this->spiRead(MPUREG_ACCEL_ZOUT_L);
    accelZ = (byte_H<<8)| byte_L;

    // Read Temp
    byte_H = this->spiRead(MPUREG_TEMP_OUT_H);
    byte_L = this->spiRead(MPUREG_TEMP_OUT_L);
    temp = (byte_H<<8)| byte_L;

    // Read GyroX
    byte_H = this->spiRead(MPUREG_GYRO_XOUT_H);
    byte_L = this->spiRead(MPUREG_GYRO_XOUT_L);
    gyroX = (byte_H<<8)| byte_L;
    // Read GyroY
    byte_H = this->spiRead(MPUREG_GYRO_YOUT_H);
    byte_L = this->spiRead(MPUREG_GYRO_YOUT_L);
    gyroY = (byte_H<<8)| byte_L;
    // Read GyroZ
    byte_H = this->spiRead(MPUREG_GYRO_ZOUT_H);
    byte_L = this->spiRead(MPUREG_GYRO_ZOUT_L);
    gyroZ = (byte_H<<8)| byte_L;

    o->rawAcc[0] = (float)accelX;
    o->rawAcc[1] = (float)accelY;
    o->rawAcc[2] = (float)accelZ;
    o->rawGyro[0] = (float)gyroX;
    o->rawGyro[1] = (float)gyroY;
    o->rawGyro[2] = (float)gyroZ;
    o->temp = (float)temp;

}//eof

/**
 * \brief To test the SPI communication: send one arbitrary byte.
 */
void clMPU6000::sendOneByte(){
	this->csPin->toggle();

	  	  /* Loop while DR register is not empty */
	    while (SPI_I2S_GetFlagStatus(this->spi, SPI_I2S_FLAG_TXE) == RESET)
	    {
	    }

	    SPI_I2S_SendData(this->spi, 3);
	    /* Wait to receive a Byte */
	    while (SPI_I2S_GetFlagStatus(this->spi, SPI_I2S_FLAG_RXNE) == RESET)
	    {
	    }
	   uint8_t dump = SPI_I2S_ReceiveData(this->spi);

//	   this->csPin->set(LED::state_ON);

}
