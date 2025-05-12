#include "hdc1080_sensor.h"
#include "freertos/task.h"
#include "esp_log.h"

volatile float latest_temp = 0.0f;
volatile float latest_hum = 0.0f;

void hdc1080_sensor_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static void hdc1080_read(float *temperature, float *humidity) {
    uint8_t data[4];
    uint8_t reg = HDC1080_TEMP_REG;

    i2c_master_write_to_device(I2C_MASTER_NUM, HDC1080_ADDR, &reg, 1, 100 / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(20));

    i2c_master_read_from_device(I2C_MASTER_NUM, HDC1080_ADDR, data, 4, 100 / portTICK_PERIOD_MS);

    uint16_t raw_temp = (data[0] << 8) | data[1];
    uint16_t raw_humi = (data[2] << 8) | data[3];

    *temperature = ((float)raw_temp / 65536.0f) * 165.0f - 40.0f;
    *humidity = ((float)raw_humi / 65536.0f) * 100.0f;
}

void hdc1080_task(void *pvParameters) {
    float temp, hum;
    while (1) {
        hdc1080_read(&temp, &hum);
        latest_temp = temp;
        latest_hum = hum;
        vTaskDelay(pdMS_TO_TICKS(980));
    }
}