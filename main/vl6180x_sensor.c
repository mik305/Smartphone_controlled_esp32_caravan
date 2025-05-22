
#include "esp_check.h"
/*************************************************************************
 *  vl6180x_sensor.c  – obsługa czterech czujników VL6180X na jednej szynie
 *  autor: ChatGPT (maj 2025)
 *************************************************************************/
#include "vl6180x_sensor.h"

static const char *TAG = "VL6180X";
#define I2C_MASTER_NUM          0
#define SENSOR_CNT              4
#define VL6180X_DEFAULT_ADDR    0x29          // fabryczny adres każdego VL6180X
#define VL6180X_NEW_ADDR_BASE   0x30          // 0x30-0x33 – nowe adresy
#define VL6180X_REG_SLAVE_ADDR  0x0212        // rejestr zmiany adresu (datasheet)

/* ——— tablice opisujące sprzęt ——— */
static const gpio_num_t shut_pins[SENSOR_CNT] = {
    VL6180X_SHUT1_GPIO, VL6180X_SHUT2_GPIO,
    VL6180X_SHUT3_GPIO, VL6180X_SHUT4_GPIO
};
static uint8_t sensor_addr[SENSOR_CNT] = {   // docelowe adresy I²C
    VL6180X_NEW_ADDR_BASE + 0,
    VL6180X_NEW_ADDR_BASE ,
    VL6180X_NEW_ADDR_BASE ,
    VL6180X_NEW_ADDR_BASE + 3
};

/* ——— niskopoziomowe I²C ——— */
static esp_err_t write8(uint8_t i2c_addr, uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { reg >> 8, reg & 0xFF, val };
    return i2c_master_write_to_device(I2C_MASTER_NUM, i2c_addr,
                                      buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
}
static esp_err_t read8(uint8_t i2c_addr, uint16_t reg, uint8_t *val)
{
    uint8_t addr[2] = { reg >> 8, reg & 0xFF };
    ESP_ERROR_CHECK(i2c_master_write_to_device(I2C_MASTER_NUM, i2c_addr,
                                               addr, sizeof(addr), 100 / portTICK_PERIOD_MS));
    return i2c_master_read_from_device(I2C_MASTER_NUM, i2c_addr,
                                       val, 1, 100 / portTICK_PERIOD_MS);
}

/* ——— inicjalizacja wszystkich czujników ——— */
static void vl6180x_init_all(void)
{
    /* 1. Piny SHUT ustawiamy jako wyjścia i gasimy wszystkie czujniki */
    gpio_config_t io = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL<<VL6180X_SHUT1_GPIO) |
                         (1ULL<<VL6180X_SHUT2_GPIO) |
                         (1ULL<<VL6180X_SHUT3_GPIO) |
                         (1ULL<<VL6180X_SHUT4_GPIO))
    };
    gpio_config(&io);
    for (int i = 0; i < SENSOR_CNT; ++i) gpio_set_level(shut_pins[i], 0);
    vTaskDelay(pdMS_TO_TICKS(10));          // >0.6 ms według datasheet

    /* 2. Uruchamiamy kolejno każdy czujnik i nadajemy mu unikalny adres */
    for (int i = 0; i < SENSOR_CNT; ++i)
    {
        gpio_set_level(shut_pins[i], 1);    // włącz zasilanie
        vTaskDelay(pdMS_TO_TICKS(2));       // boot-loader potrzebuje ≈1 ms

        /* pierwszy kontakt pod adresem domyślnym */
        if (write8(VL6180X_DEFAULT_ADDR, VL6180X_REG_SLAVE_ADDR, sensor_addr[i]) != ESP_OK) {
            ESP_LOGE(TAG, "Nie można ustawić adresu czujnika %d", i+1);
            continue;
        }
        /* po zmianie adresu musimy od teraz używać nowego */
        /* podstawowa konfiguracja zgodna z Twoim pierwotnym kodem */
        write8(sensor_addr[i], VL6180X_SYSTEM__MODE_GPIO1,      0x00);
        write8(sensor_addr[i], VL6180X_SYSRANGE__MAX_CONV_TIME, 30);
        write8(sensor_addr[i], VL6180X_REGISTER_SCALING,        VL6180X_SCALING);
    }

    /* 3. Wszystkie czujniki pozostają cały czas włączone */
    ESP_LOGI(TAG, "VL6180X – inicjalizacja zakończona");
}

/* ——— zadanie FreeRTOS ——— */
void vl6180x_task(void *pvParameters)
{
    vl6180x_init_all();

    while (1)
    {
        for (int i = 0; i < SENSOR_CNT; ++i)
        {
            /* pojedynczy pomiar */
            write8(sensor_addr[i], VL6180X_SYSRANGE__START, 0x01);

            /* czekamy na gotowy wynik (bit 0 = 1) max 40 ms */
            uint8_t sts;  TickType_t t0 = xTaskGetTickCount();
            do {
                vTaskDelay(pdMS_TO_TICKS(2));
                read8(sensor_addr[i], VL6180X_RESULT__RANGE_STATUS, &sts);
            } while (!(sts & 0x01) && (xTaskGetTickCount()-t0) < pdMS_TO_TICKS(40));

            /* odczyt końcowy */
            uint8_t raw = 0;
            if (read8(sensor_addr[i], VL6180X_RESULT__RANGE_VAL, &raw) == ESP_OK) {
                if(i==3 || i ==0)
                    ESP_LOGI(TAG, "S%d = %3u mm", i+1, (uint16_t)raw*VL6180X_SCALING);
            }
            write8(sensor_addr[i], VL6180X_SYSTEM__INT_CLEAR, 0x01);

            vTaskDelay(pdMS_TO_TICKS(25));  // ~25 ms przerwy → ≈10 Hz na sensor
        }
    }
}
