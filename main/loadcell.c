#include "hx711.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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


static void hx_task(void *pv)
{
    for (int i=0;i<4;i++) hx711_init(&cells[i]);

    while (true) {
        for (int i=0;i<4;i++) {
            if (hx711_read_raw(&cells[i], (int32_t *)&hx_raw[i]) == ESP_OK) {
                ESP_LOGI(TAG, "Ch%d: %ld", i+1, hx_raw[i]);
               /* if(i == 0){
                    temp = (hx_raw[i] + 131605) / 130;
                    ESP_LOGI(TAG, "Ch%d: %ld", i+1, temp);
                }
                else if (i == 1){
                    temp = (hx_raw[i] + 57532) / 130;
                    ESP_LOGI(TAG, "Ch%d: %ld", i+1, hx_raw[i]);
                }*/
                    
            } else {
                ESP_LOGW(TAG, "Ch%d timeout", i+1);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));   /* odczyt co 1 s */
    }
}

void hx_start_task(void)
{
    xTaskCreatePinnedToCore(hx_task, "hx711_task", 4096,
                            NULL, 5, NULL, tskNO_AFFINITY);
}
