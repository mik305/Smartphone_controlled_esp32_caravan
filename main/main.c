#include "wifi_ap.h"
#include "http_server.h"
#include "actuator_control.h"
#include "nvs_flash.h"
#include "lsm6dsox_sensor.h"
#include "driver/i2c.h"

/**
 * @brief Inicjalizuje magistralę I2C w trybie master.
 * * Konfiguruje piny SDA i SCL oraz prędkość zegara dla komunikacji I2C,
 * a następnie instaluje sterownik.
 */
static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LSM6DSOX_I2C_SDA_IO,
        .scl_io_num = LSM6DSOX_I2C_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LSM6DSOX_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(LSM6DSOX_I2C_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(LSM6DSOX_I2C_NUM, conf.mode, 0, 0, 0));
}

void app_main(void)
{
    // Inicjalizacja pamięci NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicjalizacja kluczowych komponentów
    wifi_init_softap();
    actuator_pwm_init();
    i2c_master_init();      // Inicjalizacja magistrali I2C
    lsm6dsox_sensor_init(); // Inicjalizacja rejestrów czujnika
    start_webserver();      // Uruchomienie serwera WWW

    // Utworzenie zadania do odczytu danych z akcelerometru
    xTaskCreate(lsm6dsox_task, "lsm6dsox_task", 4096, NULL, 5, NULL);
}