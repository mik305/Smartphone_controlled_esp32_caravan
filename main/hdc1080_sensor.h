#ifndef HDC1080_SENSOR_H
#define HDC1080_SENSOR_H

#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           39
#define I2C_MASTER_SDA_IO           38
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000

#define HDC1080_ADDR                0x40
#define HDC1080_TEMP_REG            0x00
#define HDC1080_HUMI_REG            0x01

extern volatile float latest_temp;
extern volatile float latest_hum;

void hdc1080_sensor_init(void);
void hdc1080_task(void *pvParameters);

#endif