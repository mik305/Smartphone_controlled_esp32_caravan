#ifndef BMI323_SENSOR_H
#define BMI323_SENSOR_H

#include "driver/i2c.h"

#define BMI323_I2C_ADDR             0x69
#define BMI323_I2C_NUM              I2C_NUM_0

// Rejestry BMI323 (zgodne z dokumentacją)
#define BMI323_CHIP_ID              0x00
#define BMI323_ERR_REG              0x01
#define BMI323_STATUS               0x02
#define BMI323_ACC_DATA_X           0x03
#define BMI323_ACC_DATA_Y           0x04
#define BMI323_ACC_DATA_Z           0x05
#define BMI323_GYR_DATA_X           0x06
#define BMI323_GYR_DATA_Y           0x07
#define BMI323_GYR_DATA_Z           0x08
#define BMI323_TEMP_DATA            0x09
#define BMI323_ACC_CONF             0x20
#define BMI323_GYR_CONF             0x21
#define BMI323_PWR_CONF             0x7C
#define BMI323_PWR_CTRL             0x7D
#define BMI323_CMD                  0x7E

// Komendy
#define BMI323_SOFT_RESET_CMD       0xDE
#define BMI323_NORMAL_MODE          0x03

// Konfiguracje (zgodne z dokumentacją)
#define BMI323_ACC_RANGE_8G         (0x01 << 4)
#define BMI323_ACC_ODR_100HZ        (0x0A << 0)
#define BMI323_GYR_RANGE_1000DPS    (0x01 << 4)
#define BMI323_GYR_ODR_100HZ        (0x0A << 0)

// Zmienne globalne
extern volatile float bmi323_accel_g[3];
extern volatile float bmi323_gyro_dps[3];
extern volatile float bmi323_temp;

void bmi323_init(void);
void bmi323_task(void *pvParameters);
void bmi323_print_debug_info(void);

#endif


