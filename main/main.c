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

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    actuator_pwm_init();
    hcsr04_sensor_init();
    hdc1080_sensor_init();  // Inicjalizacja I2C tutaj
    lsm6dsox_sensor_init(); // Używa tej samej magistrali I2C
    hx_start_task(); 
    start_webserver();


    xTaskCreate(hcsr04_task, "hcsr04_task", 4096, NULL, 5, NULL);
    xTaskCreate(hdc1080_task, "hdc1080_task", 4096, NULL, 5, NULL);
    xTaskCreate(lsm6dsox_task, "lsm6dsox_task", 4096, NULL, 5, NULL);
    if (bmi323_temp_init() == ESP_OK)
        xTaskCreate(bmi323_temp_task, "bmi323T", 4096, NULL, 5, NULL);
    
    xTaskCreate(vl6180x_task, "vl6180_task", 4096, NULL, 5, NULL);

}










