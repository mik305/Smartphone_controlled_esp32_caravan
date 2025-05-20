#ifndef VL6180X_SENSOR_H
#define VL6180X_SENSOR_H

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_log.h"

/* —— stałe ogólne —— */
#define VL6180X_ADDR                     0x29

/* —— kluczowe rejestry —— */
#define VL6180X_SYSRANGE__START          0x0018
#define VL6180X_SYSRANGE__MAX_CONV_TIME  0x001C
#define VL6180X_RESULT__RANGE_STATUS     0x004D
#define VL6180X_RESULT__RANGE_VAL        0x0062
#define VL6180X_SYSTEM__INT_CLEAR        0x0015
#define VL6180X_SYSTEM__MODE_GPIO1       0x0011
#define VL6180X_REGISTER_SCALING         0x0030

/* —— konfiguracja zasięgu —— */
#define VL6180X_SCALING                  1

/* —— piny SHUT —— */
#define VL6180X_SHUT1_GPIO               18
#define VL6180X_SHUT2_GPIO               46
#define VL6180X_SHUT3_GPIO               41
#define VL6180X_SHUT4_GPIO               45

/* —— API —— */
void vl6180x_sensor_init(void);
void vl6180x_set_active_sensor(uint8_t sensor_index);
uint8_t vl6180x_get_active_sensor(void);
void vl6180x_task(void *pvParameters);

#endif /* VL6180X_SENSOR_H */