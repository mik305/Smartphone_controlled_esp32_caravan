#include "lsm6dsox_sensor.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "LSM6DSOX";

void lsm6dsox_sensor_init() {
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
    float accel_g[3], gyro_dps[3];
    
    while(1) {
        read_sensor_data(accel, gyro);
        
        // Konwersja jednostek
        for(int i=0; i<3; i++) {
            accel_g[i] = accel[i] * 0.244e-3; // ±8g zakres
            gyro_dps[i] = gyro[i] * 70.0e-3;  // ±2000 dps zakres
        }
        
        ESP_LOGI(TAG, "Accel: X:%.2fg Y:%.2fg Z:%.2fg | Gyro: X:%.2f°/s Y:%.2f°/s Z:%.2f°/s",
                accel_g[0], accel_g[1], accel_g[2],
                gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}