#ifndef LSM6DSOX_SENSOR_H
#define LSM6DSOX_SENSOR_H

#include "driver/i2c.h"

#define LSM6DSOX_I2C_SCL_IO         1
#define LSM6DSOX_I2C_SDA_IO         2
#define LSM6DSOX_I2C_NUM            I2C_NUM_1
#define LSM6DSOX_I2C_FREQ_HZ        100000
#define LSM6DSOX_ADDR               0x6B

// Rejestry czujnika
#define LSM6DSOX_WHO_AM_I          0x0F
#define LSM6DSOX_CTRL1_XL          0x10
#define LSM6DSOX_CTRL2_G           0x11
#define LSM6DSOX_OUTX_L_XL         0x28
#define LSM6DSOX_OUTX_H_XL         0x29
#define LSM6DSOX_OUTY_L_XL         0x2A
#define LSM6DSOX_OUTY_H_XL         0x2B
#define LSM6DSOX_OUTZ_L_XL         0x2C
#define LSM6DSOX_OUTZ_H_XL         0x2D
#define LSM6DSOX_OUTX_L_G          0x22
#define LSM6DSOX_OUTX_H_G          0x23
#define LSM6DSOX_OUTY_L_G          0x24
#define LSM6DSOX_OUTY_H_G          0x25
#define LSM6DSOX_OUTZ_L_G          0x26
#define LSM6DSOX_OUTZ_H_G          0x27

void lsm6dsox_sensor_init(void);
void lsm6dsox_task(void *pvParameters);

#endif