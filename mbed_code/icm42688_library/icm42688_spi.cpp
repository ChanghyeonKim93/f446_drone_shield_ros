#include "icm42688_spi.h"

ICM42688_SPI::ICM42688_SPI() 
: spi_(ICM42688_SPI_MOSI,ICM42688_SPI_MISO, ICM42688_SPI_CLK), 
cs_(ICM42688_SPI_CS)
{
    cs_ = 1; // deselect
    spi_.format( 8, 0 );
    spi_.frequency( 2000000 );

    printf("CHIP ID: %x \r\n", getChipID());
};


float ICM42688_SPI::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      break;
  }
  return _aRes;
};

float ICM42688_SPI::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_15_625DPS:
      _gRes = 15.625f/32768.0f;
      break;
    case GFS_31_25DPS:
      _gRes = 31.25f/32768.0f;
      break;
    case GFS_62_50DPS:
      _gRes = 62.5f/32768.0f;
      break;
    case GFS_125DPS:
      _gRes = 125.0f/32768.0f;
      break;
    case GFS_250DPS:
      _gRes = 250.0f/32768.0f;
      break;
    case GFS_500DPS:
      _gRes = 500.0f/32768.0f;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f/32768.0f;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f/32768.0f;
      break;
  }
  return _gRes;
};


void ICM42688_SPI::reset()
{
    // reset device
    uint8_t temp = _readRegister(ICM42688_DEVICE_CONFIG);
    _writeRegister(ICM42688_DEVICE_CONFIG, temp | 0x01); // Set bit 0 to 1 to reset ICM42688
    wait_us(2000); // Wait for all registers to reset
};

uint8_t ICM42688_SPI::status()
{
  // reset device
  uint8_t temp = _readRegister(ICM42688_INT_STATUS);
  return temp;
};

void ICM42688_SPI::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR, uint8_t aMode, uint8_t gMode)
{
  uint8_t temp;

    // select register bank 0
  // temp = _readRegister(ICM42688_REG_BANK_SEL);
  // _writeRegister(ICM42688_REG_BANK_SEL, temp | 0x00 ); // select Bank 0

  // temp = _readRegister(ICM42688_PWR_MGMT0); // make sure not to disturb reserved bit values
  // _writeRegister(ICM42688_PWR_MGMT0, gMode << 2 | aMode);  // enable gyro and accel

  // temp = _readRegister(ICM42688_ACCEL_CONFIG0);
  // _writeRegister(ICM42688_ACCEL_CONFIG0, temp | AODR | Ascale << 5); // set accel ODR and FS

  //  temp = _readRegister(ICM42688_GYRO_CONFIG0);
  // _writeRegister(ICM42688_GYRO_CONFIG0, temp | GODR | Gscale << 5); // set gyro ODR and FS

  //  temp = _readRegister(ICM42688_GYRO_ACCEL_CONFIG0);
  // _writeRegister(ICM42688_GYRO_ACCEL_CONFIG0, temp | 0x44); // set gyro and accel bandwidth to ODR/10
 
  // // interupt handling
  //  temp = _readRegister(ICM42688_INT_CONFIG);
  // _writeRegister(ICM42688_INT_CONFIG, temp | 0x18 | 0x03 ); // set both interrupts active high, push-pull, pulsed

  //  temp = _readRegister(ICM42688_INT_CONFIG1);
  // _writeRegister(ICM42688_INT_CONFIG1, temp & ~(0x10) ); // set bit 4 to zero for proper function of INT1 and INT2
 
  //  temp = _readRegister(ICM42688_INT_SOURCE0);
  // _writeRegister(ICM42688_INT_SOURCE0, temp | 0x08 ); // route data ready interrupt to INT1
 
  // // Select Bank 0
  //  temp = _readRegister(ICM42688_REG_BANK_SEL);
  // _writeRegister(ICM42688_REG_BANK_SEL, temp | 0x00 ); // select Bank 0

    temp = _readRegister(ICM42688_PWR_MGMT0); // make sure not to disturb reserved bit values
  _writeRegister(ICM42688_PWR_MGMT0, temp | 0x0F);  // enable gyro and accel in low noise mode

   temp = _readRegister(ICM42688_GYRO_CONFIG0);
  _writeRegister(ICM42688_GYRO_CONFIG0, (temp & 0x1F) | Gscale << 5); // gyro full scale and data rate

   temp = _readRegister(ICM42688_GYRO_CONFIG0);
  _writeRegister(ICM42688_GYRO_CONFIG0, (temp & 0xF0) | GODR); // gyro full scale and data rate

   temp = _readRegister(ICM42688_ACCEL_CONFIG0);
  _writeRegister(ICM42688_ACCEL_CONFIG0, (temp & 0x1F) | Ascale << 5); // set accel full scale and data rate

   temp = _readRegister(ICM42688_ACCEL_CONFIG0);
  _writeRegister(ICM42688_ACCEL_CONFIG0, (temp & 0xF0) | AODR); // set accel full scale and data rate

   temp = _readRegister(ICM42688_GYRO_CONFIG1);
  _writeRegister(ICM42688_GYRO_CONFIG1, temp | 0xD0); // set temperature sensor low pass filter to 5Hz, use first order gyro filter

   temp = _readRegister(ICM42688_INT_CONFIG);
  _writeRegister(ICM42688_INT_CONFIG, temp | 0x18 | 0x03 ); // set both interrupts active high, push-pull, pulsed

   temp = _readRegister(ICM42688_INT_CONFIG1);
  _writeRegister(ICM42688_INT_CONFIG1, temp & ~(0x10) ); // set bit 4 to zero for proper function of INT1 and INT2
 
   temp = _readRegister(ICM42688_INT_SOURCE0);
  _writeRegister(ICM42688_INT_SOURCE0, temp | 0x18 ); // route data ready interrupt to INT1
 
  // Select Bank 4
   temp = _readRegister(ICM42688_REG_BANK_SEL);
  _writeRegister(ICM42688_REG_BANK_SEL, temp | 0x04 ); // select Bank 4

   temp = _readRegister(ICM42688_APEX_CONFIG5);
  _writeRegister(ICM42688_APEX_CONFIG5, temp & ~(0x07) ); // select unitary mounting matrix

   temp = _readRegister(ICM42688_REG_BANK_SEL);
  _writeRegister(ICM42688_REG_BANK_SEL, temp & 0x00 ); // select Bank 0
};

void ICM42688_SPI::offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

  printf("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  wait_us(2000000);

  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    wait_us(50000);
  }

  dest1[0] = sum[1] * _aRes / 128.0f;
  dest1[1] = sum[2] * _aRes / 128.0f;
  dest1[2] = sum[3] * _aRes / 128.0f;
  dest2[0] = sum[4] * _gRes / 128.0f;
  dest2[1] = sum[5] * _gRes / 128.0f;
  dest2[2] = sum[6] * _gRes / 128.0f;

  if (dest1[0] > 0.8f)  {
    dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[0] < -0.8f) {
    dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[1] > 0.8f)  {
    dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[1] < -0.8f) {
    dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[2] > 0.8f)  {
    dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest1[2] < -0.8f) {
    dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

};


void ICM42688_SPI::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  _readBuffer(ICM42688_TEMP_DATA1, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}



uint8_t ICM42688_SPI::getChipID()
{
    uint8_t c = _readRegister(ICM42688_WHO_AM_I);
    if(c == 0x42) printf("ICM42688 is online...\r\n");
    return c;
};

uint8_t ICM42688_SPI::_readRegister( uint8_t addr ){
    uint8_t ret;
    cs_ =   0;
    ret = spi_.write( addr | 0x80 );        // send address
    ret = spi_.write( 0x00 );
    cs_ =   1;
    wait_us(WAIT_US_FOR_SPI);

    return ret;
};
 
uint8_t ICM42688_SPI::_writeRegister( uint8_t addr, uint8_t data ){
    cs_ =   0;
    spi_.write( addr );
    spi_.write( data );    
    cs_ =   1;
    wait_us(WAIT_US_FOR_SPI);

    return 0;
};
 
uint8_t ICM42688_SPI::_readBuffer( uint8_t addr, uint8_t len, uint8_t* buf ){
    cs_ =   0;
    spi_.write( addr | 0x80 );                  // send address
    while( len-- ){
        *(buf++) = spi_.write( 0x00 );          // read data
    }
    cs_ =   1;
    wait_us(WAIT_US_FOR_SPI);
    return 0;
};

uint8_t ICM42688_SPI::_writeBuffer( uint8_t addr, uint8_t len, uint8_t* buf ){
    cs_ =   0;
    spi_.write( addr );                         // send address
    while( len-- ){
        spi_.write( *(buf++) );                 // send data
    }
    cs_ =   1;
    wait_us(WAIT_US_FOR_SPI);
    return 0;
};
 