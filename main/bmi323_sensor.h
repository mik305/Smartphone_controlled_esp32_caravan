#ifndef BMI323_SENSOR_H
#define BMI323_SENSOR_H

#include "driver/i2c.h"

#define BMI323_I2C_SCL_IO         39
#define BMI323_I2C_SDA_IO         38
#define BMI323_I2C_NUM            I2C_NUM_0
#define BMI323_I2C_FREQ_HZ        100000
#define BMI323_ADDR               0x69

// Rejestry czujnika
#define BMI323_ACC_X_LSB          0x12
#define BMI323_ACC_X_MSB          0x13
#define BMI323_ACC_Y_LSB          0x14
#define BMI323_ACC_Y_MSB          0x15
#define BMI323_ACC_Z_LSB          0x16
#define BMI323_ACC_Z_MSB          0x17
#define BMI323_GYR_X_LSB          0x18
#define BMI323_GYR_X_MSB          0x19
#define BMI323_GYR_Y_LSB          0x1A
#define BMI323_GYR_Y_MSB          0x1B
#define BMI323_GYR_Z_LSB          0x1C
#define BMI323_GYR_Z_MSB          0x1D
#define BMI323_PWR_CTRL           0x03
#define BMI323_ACC_CONF           0x20
#define BMI323_GYR_CONF           0x21

// Zmienne globalne
extern volatile float bmi323_accel_g[3];
extern volatile float bmi323_gyro_dps[3];

void bmi323_sensor_init(void);
void bmi323_task(void *pvParameters);

#endif