#include "sensors.h"
#include "twi.h"

#define GYRO_ADDRESS  0b1101011
#define ACC_ADDRESS   0b0011001

int16_t g[3];
int16_t a[3];

void gyro_init(void)
{
  gyro_write_reg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  gyro_write_reg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
}

void gyro_write_reg(uint8_t reg, uint8_t value)
{
  twi_writeTo(GYRO_ADDRESS, (uint8_t[]){reg, value}, 2, 1, 1);
}

void gyro_read(void)
{
  uint8_t buf[6];
  
  // assert the MSB of the address to get the gyro
  // to do slave-transmit subaddress updating.
  twi_writeTo(GYRO_ADDRESS, (uint8_t[]){L3G_OUT_X_L | (1 << 7)}, 1, 1, 0);
  twi_readFrom(GYRO_ADDRESS, buf, 6, 1);

  // combine high and low bytes
  g[X] = (int16_t)(buf[1] << 8 | buf[0]);
  g[Y] = (int16_t)(buf[3] << 8 | buf[2]);
  g[Z] = (int16_t)(buf[5] << 8 | buf[4]);
}

void acc_init(void)
{
  acc_write_reg(LSM303_CTRL_REG1_A, 0x47); // normal power mode, all axes enabled, 50 Hz
  acc_write_reg(LSM303_CTRL_REG4_A, 0x08); // high resolution output mode
  acc_write_reg(LSM303_CTRL_REG4_A, 0x20); // 8 g full scale: FS = 10 on DLHC
}

void acc_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t data[2];
  
  data[0] = reg;
  data[1] = value;
  
  twi_writeTo(ACC_ADDRESS, data, 2, 1, 1);
}

void acc_read(void)
{
  uint8_t buf[6];
  
  // assert the MSB of the address to get the accelerometer
  // to do slave-transmit subaddress updating.
  twi_writeTo(ACC_ADDRESS, (uint8_t[]){LSM303_OUT_X_L_A | (1 << 7)}, 1, 1, 0);
  twi_readFrom(ACC_ADDRESS, buf, 6, 1);
  
  // combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
  // GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
  // if you port it to a compiler that does a logical right shift instead.
  a[X] = ((int16_t)(buf[1] << 8 | buf[0])) >> 4;
  a[Y] = ((int16_t)(buf[3] << 8 | buf[2])) >> 4;
  a[Z] = ((int16_t)(buf[5] << 8 | buf[4])) >> 4;
}
