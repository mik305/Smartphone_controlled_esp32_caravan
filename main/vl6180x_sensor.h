#pragma once
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/task.h"
#include "esp_log.h"

/* ── stałe ─────────────────────────────────────────────────────────── */
#define VL6180X_DEFAULT_ADDR            0x29

/* adres 16-bitowy rejestru z nowym adresem I²C */
#define VL6180X_REG_SLAVE_DEVICE_ADDR   0x0212
#define VL6180X_REG_SLAVE_ADDR           0x0212  
#define VL6180X_SENSOR_COUNT             4   
/* kluczowe rejestry pomiaru (datasheet AN4545) */
#define VL6180X_SYSRANGE__START         0x0018
#define VL6180X_RESULT__RANGE_STATUS    0x004D
#define VL6180X_RESULT__RANGE_VAL       0x0062
#define VL6180X_SYSTEM__INT_CLEAR       0x0015

/* —— kluczowe rejestry —— */
#define VL6180X_SYSRANGE__START          0x0018
#define VL6180X_SYSRANGE__MAX_CONV_TIME  0x001C   //  <-- potrzebne makro
#define VL6180X_RESULT__RANGE_STATUS     0x004D
#define VL6180X_RESULT__RANGE_VAL        0x0062
#define VL6180X_SYSTEM__INT_CLEAR        0x0015
#define VL6180X_SYSTEM__MODE_GPIO1       0x0011   //  <-- potrzebne makro
#define VL6180X_REGISTER_SCALING         0x0030   //  <-- potrzebne makro

/* —— konfiguracja zasięgu —— */
#define VL6180X_SCALING                  1        //  <-- potrzebne makro



/* piny SHUT (przykład) */
#define VL6180X_SHUT1_GPIO              18
#define VL6180X_SHUT2_GPIO              46
#define VL6180X_SHUT3_GPIO              41
#define VL6180X_SHUT4_GPIO              45

#define NUM_VL6180X                     4

void vl6180x_task(void *pv);
