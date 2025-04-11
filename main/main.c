#include "wifi_ap.h"
#include "http_server.h"
#include "actuator_control.h"
#include "hdc1080_sensor.h"
#include "nvs_flash.h"

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    actuator_pwm_init();
    hdc1080_sensor_init();
    start_webserver();

    xTaskCreate(hdc1080_task, "hdc1080_task", 4096, NULL, 5, NULL);
}


