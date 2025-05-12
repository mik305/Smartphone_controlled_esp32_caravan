#include "bmi323_sensor.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "BMI323";

// Definicje zmiennych globalnych
volatile float bmi323_accel_g[3] = {0.0f, 0.0f, 0.0f};
volatile float bmi323_gyro_dps[3] = {0.0f, 0.0f, 0.0f};

void bmi323_print_data() {
    printf("\n=== BMI323 Sensor Data ===\n");
    printf("Accelerometer:\n");
    printf("  X: %8.2f g\n", bmi323_accel_g[0]);
    printf("  Y: %8.2f g\n", bmi323_accel_g[1]);
    printf("  Z: %8.2f g\n", bmi323_accel_g[2]);
    printf("Gyroscope:\n");
    printf("  X: %8.2f dps\n", bmi323_gyro_dps[0]);
    printf("  Y: %8.2f dps\n", bmi323_gyro_dps[1]);
    printf("  Z: %8.2f dps\n", bmi323_gyro_dps[2]);
    printf("========================\n");
}

void bmi323_sensor_init() {
    // Reset czujnika (wymagany dla BMI323)
    uint8_t reset_cmd[2] = {0x7E, 0xB6};
    ESP_ERROR_CHECK(i2c_master_write_to_device(BMI323_I2C_NUM, BMI323_ADDR, reset_cmd, sizeof(reset_cmd), 100 / portTICK_PERIOD_MS));
    vTaskDelay(pdMS_TO_TICKS(200)); // Czekaj aż się zresetuje

    // Włącz czujnik
    uint8_t config[2] = {BMI323_PWR_CTRL, 0x0E}; // Włącz akcelerometr i żyroskop
    ESP_ERROR_CHECK(i2c_master_write_to_device(BMI323_I2C_NUM, BMI323_ADDR, config, sizeof(config), 100 / portTICK_PERIOD_MS));
    
    // Konfiguracja akcelerometru (100 Hz, ±8g)
    config[0] = BMI323_ACC_CONF;
    config[1] = 0xA8; // ODR 100Hz, zakres ±8g
    ESP_ERROR_CHECK(i2c_master_write_to_device(BMI323_I2C_NUM, BMI323_ADDR, config, sizeof(config), 100 / portTICK_PERIOD_MS));
    
    // Konfiguracja żyroskopu (100 Hz, ±2000 dps)
    config[0] = BMI323_GYR_CONF;
    config[1] = 0xA9; // ODR 100Hz, zakres ±2000dps
    ESP_ERROR_CHECK(i2c_master_write_to_device(BMI323_I2C_NUM, BMI323_ADDR, config, sizeof(config), 100 / portTICK_PERIOD_MS));
}

static void read_sensor_data(int16_t *accel, int16_t *gyro) {
    uint8_t data[12];
    
    // Odczyt akcelerometru (X, Y, Z)
    uint8_t reg = BMI323_ACC_X_LSB;
    ESP_ERROR_CHECK(i2c_master_write_read_device(BMI323_I2C_NUM, BMI323_ADDR, &reg, 1, data, 6, 100 / portTICK_PERIOD_MS));
    
    accel[0] = (int16_t)((data[1] << 8) | data[0]);
    accel[1] = (int16_t)((data[3] << 8) | data[2]);
    accel[2] = (int16_t)((data[5] << 8) | data[4]);
    
    // Odczyt żyroskopu (X, Y, Z)
    reg = BMI323_GYR_X_LSB;
    ESP_ERROR_CHECK(i2c_master_write_read_device(BMI323_I2C_NUM, BMI323_ADDR, &reg, 1, data, 6, 100 / portTICK_PERIOD_MS));
    
    gyro[0] = (int16_t)((data[1] << 8) | data[0]);
    gyro[1] = (int16_t)((data[3] << 8) | data[2]);
    gyro[2] = (int16_t)((data[5] << 8) | data[4]);
}

void bmi323_task(void *pvParameters) {
    int16_t accel[3], gyro[3];
    int counter = 0;
    
    while(1) {
        read_sensor_data(accel, gyro);
        
        // Konwersja jednostek i aktualizacja zmiennych globalnych
        for(int i=0; i<3; i++) {
            bmi323_accel_g[i] = accel[i] * 0.000244f; // ±8g zakres (8g / 32768)
            bmi323_gyro_dps[i] = gyro[i] * 0.061f;    // ±2000 dps zakres (2000/32768)
        }
        
        // Wyświetl dane co 50 odczytów (co ~0.5s przy 10ms delay)
        if(++counter % 50 == 0) {
            bmi323_print_data();
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}