#pragma once
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"

/* tryb wzmacniacza */
typedef enum {
    HX711_GAIN_128 = 1,   /* kanał A, gain 128 – domyślny */
    HX711_GAIN_64  = 3,   /* kanał A, gain 64  */
    HX711_GAIN_32  = 2    /* kanał B, gain 32  */
} hx711_gain_t;

/* uchwyt pojedynczego wzmacniacza */
typedef struct {
    gpio_num_t clk;
    gpio_num_t data;
    hx711_gain_t gain;
} hx711_t;

/* API */
esp_err_t hx711_init        (hx711_t *dev);
esp_err_t hx711_is_ready    (hx711_t *dev, TickType_t tout);
esp_err_t hx711_read_raw    (hx711_t *dev, int32_t *out);
esp_err_t hx711_set_gain    (hx711_t *dev, hx711_gain_t g);

void hx_start_task(void);