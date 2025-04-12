#include "wifi_ap.h"
#include "http_server.h"
#include "actuator_control.h"
#include "hdc1080_sensor.h"
#include "i2c_scanner.h"  // Dodaj ten nagłówek
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
    
    // Inicjalizacja i skanowanie I2C (na pinach 1 i 2)
    //lsm6dsox_scanner_init();
    //lsm6dsox_scan_i2c();
      // Wypisze znalezione urządzenia w terminalu
    
    hdc1080_sensor_init();  // HDC1080 używa innych pinów (zdefiniowanych w hdc1080_sensor.h)
    start_webserver();

    xTaskCreate(hdc1080_task, "hdc1080_task", 4096, NULL, 5, NULL);
}

