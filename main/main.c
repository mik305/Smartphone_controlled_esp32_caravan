#include "wifi_ap.h"
#include "http_server.h"
#include "actuator_control.h"
#include "hdc1080_sensor.h"
#include "nvs_flash.h"
#include "lsm6dsox_sensor.h"
#include "hcsr04_sensor.h"
#include "bmi323_sensor.h"
#include "hx711.h"
#include "vl6180x_sensor.h"
#include "current_sensor.h"
#include "esp_log.h"               /*  <--  do logowania */

#define TAG "APP_MAIN"

static const uint8_t adc_addr = 0x48;
static float i[CURRENT_SENSOR_NUM_CHANNELS];

/* ───── zadanie wysyłające prąd ─────────────────────────────────────── */
static void current_monitor_task(void *pv)
{
    while (true) {
        if (current_sensor_read_current_multi(I2C_NUM_0, adc_addr, i) == ESP_OK) {
            ESP_LOGI(TAG, "M1: %.3f A | M2: %.3f A | M3: %.3f A | M4: %.3f A",
                     i[0], i[1], i[2], i[3]);
            /*  ➜  zamiast ESP_LOGI wstaw tu wysyłanie przez sieć,
                np. websocket_broadcast(curr_json), mqtt_publish(), … */
        } else {
            ESP_LOGE(TAG, "Błąd pomiaru prądu!");
        }
        vTaskDelay(pdMS_TO_TICKS(500));   /* 0,5 s okres */
    }
}

/* ───── app_main ─────────────────────────────────────────────────────── */
void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    actuator_pwm_init();
    hcsr04_sensor_init();
    hdc1080_sensor_init();          /* I²C init */
    lsm6dsox_sensor_init();         /* ta sama magistrala */
    //hx_start_task();
    start_webserver();

    xTaskCreate(hcsr04_task,   "hcsr04_task",   4096, NULL, 5, NULL);
    xTaskCreate(hdc1080_task,  "hdc1080_task",  4096, NULL, 5, NULL);
    xTaskCreate(lsm6dsox_task, "lsm6dsox_task", 4096, NULL, 5, NULL);
    if (bmi323_temp_init() == ESP_OK)
        xTaskCreate(bmi323_temp_task, "bmi323T", 4096, NULL, 5, NULL);

    /* NOWE – monitor prądu */
   // xTaskCreate(current_monitor_task, "curr_mon", 4096, NULL, 5, NULL);

    //xTaskCreatePinnedToCore(vl6180x_task, "vl6180x", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}