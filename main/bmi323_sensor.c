#include "bmi323_sensor.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "BMI323";

volatile float bmi323_accel_g[3] = {0.0f, 0.0f, 0.0f};
volatile float bmi323_gyro_dps[3] = {0.0f, 0.0f, 0.0f};
volatile float bmi323_temp = 0.0f;

static esp_err_t bmi323_write_reg(uint8_t reg, uint16_t value) {
    uint8_t data[3] = {reg, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8)};
    return i2c_master_write_to_device(BMI323_I2C_NUM, BMI323_I2C_ADDR, data, sizeof(data), 100 / portTICK_PERIOD_MS);
}

static esp_err_t bmi323_read_reg(uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    esp_err_t ret = i2c_master_write_read_device(BMI323_I2C_NUM, BMI323_I2C_ADDR, &reg, 1, data, 2, 100 / portTICK_PERIOD_MS);
    *value = (data[1] << 8) | data[0];
    return ret;
}

static void bmi323_print_registers(void) {
    uint16_t reg_values[0x30];
    
    ESP_LOGI(TAG, "BMI323 Register Dump:");
    for (int i = 0; i < 0x30; i++) {
        if (bmi323_read_reg(i, &reg_values[i]) == ESP_OK) {
            ESP_LOGI(TAG, "0x%02X: 0x%04X", i, reg_values[i]);
        } else {
            ESP_LOGE(TAG, "Failed to read register 0x%02X", i);
        }
    }
}

void bmi323_init(void) {
    uint16_t chip_id;
    
    // Sprawdzenie ID czujnika (powinno być 0x0043)
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_CHIP_ID, &chip_id));
    ESP_LOGI(TAG, "BMI323 Chip ID: 0x%04X", chip_id);

    if ((chip_id & 0xFF) != 0x43) {
        ESP_LOGE(TAG, "Invalid BMI323 Chip ID!");
        return;
    }

    // Reset
    ESP_ERROR_CHECK(bmi323_write_reg(BMI323_CMD, BMI323_SOFT_RESET_CMD));
    vTaskDelay(pdMS_TO_TICKS(100));

    // Konfiguracja akcelerometru (100Hz, ±8g)
    uint16_t acc_conf = BMI323_ACC_RANGE_8G | BMI323_ACC_ODR_100HZ;
    ESP_ERROR_CHECK(bmi323_write_reg(BMI323_ACC_CONF, acc_conf));
    
    // Konfiguracja żyroskopu (100Hz, ±1000dps)
    uint16_t gyr_conf = BMI323_GYR_RANGE_1000DPS | BMI323_GYR_ODR_100HZ;
    ESP_ERROR_CHECK(bmi323_write_reg(BMI323_GYR_CONF, gyr_conf));

    // Włączenie normalnego trybu pracy
    ESP_ERROR_CHECK(bmi323_write_reg(BMI323_PWR_CTRL, BMI323_NORMAL_MODE));
    vTaskDelay(pdMS_TO_TICKS(50));

    // Wyświetlenie informacji diagnostycznych
    bmi323_print_registers();
    ESP_LOGI(TAG, "BMI323 initialized");
}

static void bmi323_read_sensor_data() {
    uint16_t acc_x, acc_y, acc_z;
    uint16_t gyr_x, gyr_y, gyr_z;
    uint16_t temp;

    // Odczyt danych akcelerometru
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_ACC_DATA_X, &acc_x));
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_ACC_DATA_Y, &acc_y));
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_ACC_DATA_Z, &acc_z));
    
    // Odczyt danych żyroskopu
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_GYR_DATA_X, &gyr_x));
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_GYR_DATA_Y, &gyr_y));
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_GYR_DATA_Z, &gyr_z));
    
    // Odczyt temperatury
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_TEMP_DATA, &temp));

    // Konwersja na wartości fizyczne
    // Akcelerometr: ±8g, 16-bit (32768 = 8g)
    bmi323_accel_g[0] = (int16_t)acc_x * (8.0f / 32768.0f);
    bmi323_accel_g[1] = (int16_t)acc_y * (8.0f / 32768.0f);
    bmi323_accel_g[2] = (int16_t)acc_z * (8.0f / 32768.0f);
    
    // Żyroskop: ±1000dps, 16-bit (32768 = 1000dps)
    bmi323_gyro_dps[0] = (int16_t)gyr_x * (1000.0f / 32768.0f);
    bmi323_gyro_dps[1] = (int16_t)gyr_y * (1000.0f / 32768.0f);
    bmi323_gyro_dps[2] = (int16_t)gyr_z * (1000.0f / 32768.0f);
    
    // Temperatura: 0x8000 = 23°C, LSB = 0.125°C
    bmi323_temp = 23.0f + ((int16_t)temp - 0x8000) * 0.125f;
}

void bmi323_print_debug_info(void) {
    uint16_t status, err_reg;
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_STATUS, &status));
    ESP_ERROR_CHECK(bmi323_read_reg(BMI323_ERR_REG, &err_reg));

    ESP_LOGI(TAG, "Status: 0x%04X, Error: 0x%04X", status, err_reg);
    ESP_LOGI(TAG, "Accel: X=%.2fg, Y=%.2fg, Z=%.2fg", 
             bmi323_accel_g[0], bmi323_accel_g[1], bmi323_accel_g[2]);
    ESP_LOGI(TAG, "Gyro: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s", 
             bmi323_gyro_dps[0], bmi323_gyro_dps[1], bmi323_gyro_dps[2]);
    ESP_LOGI(TAG, "Temperature: %.2f°C", bmi323_temp);
}

void bmi323_task(void *pvParameters) {
    while (1) {
        bmi323_read_sensor_data();
        
        // Wyświetlanie danych co 2 sekundy (opcjonalne)
        static int count = 0;
        if (++count >= 200) {
            bmi323_print_debug_info();
            count = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz update rate
    }
}

