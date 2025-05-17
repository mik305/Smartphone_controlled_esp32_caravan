#include "hx711.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

#define TAG "LOADCELL"

/* ───────── mapping pinów ─────────
 *  Ustawić zgodnie ze schematem.
 *  CLK może być wspólny lub osobny – tu każdy ma własny.
 DAT1 - IO4
 DAT2 - IO15
 DAT3 - IO16
 DAT4 - IO17
 CLK HX - IO5
 */
static hx711_t cells[4] = {
    { .clk = GPIO_NUM_5,  .data = GPIO_NUM_4,  .gain = HX711_GAIN_128 },
    { .clk = GPIO_NUM_5,  .data = GPIO_NUM_15,  .gain = HX711_GAIN_128 },
    { .clk = GPIO_NUM_5,  .data = GPIO_NUM_16,  .gain = HX711_GAIN_128 },
    { .clk = GPIO_NUM_5,  .data = GPIO_NUM_17,  .gain = HX711_GAIN_128 },
};

/* globalne wyniki (RAW) */
volatile int32_t hx_raw[4] = {0};
//int32_t temp;

esp_err_t hx_set_calibration(hx711_t *dev,
                             int32_t obciazony_output,
                             int32_t raw_output,
                             float   known_mass)
{
    if (raw_output == obciazony_output) return ESP_FAIL;

    dev->offset    = obciazony_output;
    dev->lsb_per_g = (float)(raw_output - obciazony_output) / known_mass;

    ESP_LOGI("HX", "Cal set offset=%ld  lsb/g=%.2f",
             (long)dev->offset, dev->lsb_per_g);
    return ESP_OK;
}


float hx_get_grams(hx711_t *dev)
{
    int32_t raw;
    if (hx711_read_raw(dev, &raw) != ESP_OK) return NAN;
    return (raw - dev->offset) / dev->lsb_per_g;
}

static void hx_task(void *pv)
{
    for (int i=0;i<4;i++) hx711_init(&cells[i]);

    hx_set_calibration(&cells[0], -131605, -99778, 267.0f);
    hx_set_calibration(&cells[1], -50000, -30000, 267.0f);
    hx_set_calibration(&cells[2], -131605, -99778, 267.0f);
    hx_set_calibration(&cells[3], -131605, -99778, 267.0f);

while (true)
{
    for (int i = 0; i < 4; i++) {
        int32_t raw;
        if (hx711_read_raw(&cells[i], &raw) == ESP_OK) {

            /* przechowuj, jeśli potrzebne gdzie indziej */
            hx_raw[i]  = raw;
            hx_grams[i] = (raw - cells[i].offset) / cells[i].lsb_per_g;

            /* ---------- LOG ---------- */
            ESP_LOGI("HX711", "Ch%d: %ld raw  →  %.2f g",
                     i + 1, (long)raw, hx_grams[i]);
        } else {
            ESP_LOGW("HX711", "Ch%d: timeout / odczyt błędny", i + 1);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));   /* odświeżenie co 1 s */
}
}

void hx_start_task(void)
{
    xTaskCreatePinnedToCore(hx_task, "hx711_task", 4096,
                            NULL, 5, NULL, tskNO_AFFINITY);
}
