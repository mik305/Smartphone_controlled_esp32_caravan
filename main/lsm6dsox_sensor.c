#include "lsm6dsox_sensor.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "LSM6DSOX";

// Definicje zmiennych globalnych
volatile float accel_g[3] = {0.0f, 0.0f, 0.0f};
volatile float gyro_dps[3] = {0.0f, 0.0f, 0.0f};

void lsm6dsox_sensor_init() {
    // Inicjalizacja I2C jest już wykonana w hdc1080_sensor_init(), więc tutaj nie jest potrzebna.
    // Konfiguracja akcelerometru (416 Hz, ±8g)
    uint8_t config[2] = {LSM6DSOX_CTRL1_XL, 0x6C};
    ESP_ERROR_CHECK(i2c_master_write_to_device(LSM6DSOX_I2C_NUM, LSM6DSOX_ADDR, config, sizeof(config), 100 / portTICK_PERIOD_MS));

    // Konfiguracja żyroskopu (416 Hz, ±2000 dps)
    config[0] = LSM6DSOX_CTRL2_G;
    config[1] = 0x7C;
    ESP_ERROR_CHECK(i2c_master_write_to_device(LSM6DSOX_I2C_NUM, LSM6DSOX_ADDR, config, sizeof(config), 100 / portTICK_PERIOD_MS));
}

static void read_sensor_data(int16_t *accel, int16_t *gyro) {
    uint8_t data[12];
    
    // Odczyt akcelerometru
    uint8_t reg = LSM6DSOX_OUTX_L_XL;
    ESP_ERROR_CHECK(i2c_master_write_read_device(LSM6DSOX_I2C_NUM, LSM6DSOX_ADDR, &reg, 1, data, 6, 100 / portTICK_PERIOD_MS));
    accel[0] = (data[1] << 8) | data[0];
    accel[1] = (data[3] << 8) | data[2];
    accel[2] = (data[5] << 8) | data[4];

    // Odczyt żyroskopu
    reg = LSM6DSOX_OUTX_L_G;
    ESP_ERROR_CHECK(i2c_master_write_read_device(LSM6DSOX_I2C_NUM, LSM6DSOX_ADDR, &reg, 1, data, 6, 100 / portTICK_PERIOD_MS));
    gyro[0] = (data[1] << 8) | data[0];
    gyro[1] = (data[3] << 8) | data[2];
    gyro[2] = (data[5] << 8) | data[4];
}

void lsm6dsox_task(void *pvParameters) {
    int16_t accel[3], gyro[3];
    
    while(1) {
        read_sensor_data(accel, gyro);
        
        // Konwersja jednostek i aktualizacja zmiennych globalnych
        for(int i=0; i<3; i++) {
            accel_g[i] = accel[i] * 0.244e-3; // ±8g zakres
            gyro_dps[i] = gyro[i] * 70.0e-3;  // ±2000 dps zakres
        }
        
        ESP_LOGI(TAG, "Accel: X:%.2fg Y:%.2fg Z:%.2fg | Gyro: X:%.2f°/s Y:%.2f°/s Z:%.2f°/s",
                accel_g[0], accel_g[1], accel_g[2],
                gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}