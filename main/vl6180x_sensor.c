#include "vl6180x_sensor.h"

static const char *TAG = "VL6180X";
#define I2C_MASTER_NUM 0

static uint8_t current_sensor_index = 3;
static const gpio_num_t shut_pins[] = {
    VL6180X_SHUT1_GPIO, VL6180X_SHUT2_GPIO,
    VL6180X_SHUT3_GPIO, VL6180X_SHUT4_GPIO
};

/* ——— narzędziówka I²C ——— */
static esp_err_t vl6180x_write_byte(uint16_t reg, uint8_t value)
{
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, value };
    return i2c_master_write_to_device(I2C_MASTER_NUM, VL6180X_ADDR,
                                      buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
}

static esp_err_t vl6180x_read_byte(uint16_t reg, uint8_t *data)
{
    uint8_t addr[2] = { reg >> 8, reg & 0xFF };
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, VL6180X_ADDR,
                                               addr, sizeof(addr), 100 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    return i2c_master_read_from_device(I2C_MASTER_NUM, VL6180X_ADDR,
                                       data, 1, 100 / portTICK_PERIOD_MS);
}

/* ——— aktualizacja pinów SHUT na podstawie current_sensor_index ——— */
static void update_shut_pins(void)
{
    for (size_t i = 0; i < sizeof(shut_pins)/sizeof(shut_pins[0]); ++i) {
        gpio_set_level(shut_pins[i], i == current_sensor_index);
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // >1 ms po datasheet
}

/* ——— API do zmiany czujnika ——— */
void vl6180x_set_active_sensor(uint8_t sensor_index)
{
    if (sensor_index >= sizeof(shut_pins)/sizeof(shut_pins[0])) {
        ESP_LOGE(TAG, "Nieprawidłowy indeks czujnika: %d", sensor_index);
        return;
    }
    current_sensor_index = sensor_index;
    update_shut_pins();
}

uint8_t vl6180x_get_active_sensor(void)
{
    return current_sensor_index;
}

/* ——— jednorazowa inicjalizacja ——— */
void vl6180x_sensor_init(void)
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL<<VL6180X_SHUT1_GPIO) |
                         (1ULL<<VL6180X_SHUT2_GPIO) |
                         (1ULL<<VL6180X_SHUT3_GPIO) |
                         (1ULL<<VL6180X_SHUT4_GPIO)) };
    gpio_config(&io_conf);

    // Aktywujemy domyślny czujnik (0)
    update_shut_pins();

    /* podstawowy zestaw inicjujący */
    const struct { uint16_t reg; uint8_t val; } init_seq[] = {
        { VL6180X_SYSTEM__MODE_GPIO1, 0x00 },
        { VL6180X_SYSRANGE__MAX_CONV_TIME, 30 },
        { VL6180X_REGISTER_SCALING, VL6180X_SCALING },
    };
    for (size_t i = 0; i < sizeof(init_seq)/sizeof(init_seq[0]); ++i) {
        vl6180x_write_byte(init_seq[i].reg, init_seq[i].val);
    }
}

/* ——— zadanie FreeRTOS – cykliczny pomiar ——— */
void vl6180x_task(void *pvParameters)
{
    vl6180x_sensor_init();

    while (1) {
        // Zawsze używamy aktualnej wartości current_sensor_index
        uint8_t active_sensor = vl6180x_get_active_sensor();

        /* start pojedynczego pomiaru */
        vl6180x_write_byte(VL6180X_SYSRANGE__START, 0x01);

        /* czekaj aż bit 0 w RESULT__RANGE_STATUS = 1 (gotowe) */
        uint8_t status;
        TickType_t t0 = xTaskGetTickCount();
        do {
            vTaskDelay(pdMS_TO_TICKS(2));
            vl6180x_read_byte(VL6180X_RESULT__RANGE_STATUS, &status);
        } while ((status & 0x01) == 0 &&
                 (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(40));

        /* odczytaj wynik */
        uint8_t raw_range = 0;
        if (vl6180x_read_byte(VL6180X_RESULT__RANGE_VAL, &raw_range) == ESP_OK) {
            uint16_t distance_mm = raw_range * VL6180X_SCALING;
            ESP_LOGI(TAG, "Czujnik %d → %u mm", active_sensor + 1, distance_mm);
        }

        /* skasuj przerwanie */
        vl6180x_write_byte(VL6180X_SYSTEM__INT_CLEAR, 0x01);

        vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz
    }
}