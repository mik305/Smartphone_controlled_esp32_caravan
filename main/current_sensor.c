#include "current_sensor.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "CURRENT_SENSOR";

/* ───────────────────────────────── ADS1115 register map ─────────────────── */
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

/*  Bit pola CONFIG   */
#define ADS1115_CFG_OS_SINGLE        (1U << 15)
#define ADS1115_CFG_PGA_4_096        (1U << 9)    /* ±4.096 V */
#define ADS1115_CFG_MODE_SINGLESHOT  (1U << 8)
#define ADS1115_CFG_DR_860SPS        (7U << 5)    /* 860 SPS  → 1,2 ms */
#define ADS1115_CFG_COMP_QUE_DISABLE (0x03)

/* Maski do MUX (bits 14‑12) → AINx vs GND */
#define ADS1115_MUX_SINGLE(ch)   ((uint16_t)((4 + (ch)) << 12))

/* LSB przy FS ±4.096 V – 125 µV */
#define ADS1115_LSB_VOLTAGE (4.096f / 32768.0f)

/* ───────────────────────────── INA180 parametry ─────────────────────────── */
#define INA180_GAIN         50.0f   /* INA180A2: 50 V/V */
#define SHUNT_RESISTOR_OHM  0.05f   /* 50 mΩ */

/* ───────────────────────────── pomocnicze I²C ───────────────────────────── */
static esp_err_t ads1115_write_reg(i2c_port_t i2c_num, uint8_t i2c_addr,
                                   uint8_t reg, uint16_t val)
{
    uint8_t tx[3] = {reg, (uint8_t)(val >> 8), (uint8_t)val};
    return i2c_master_write_to_device(i2c_num, i2c_addr, tx, sizeof(tx),
                                      pdMS_TO_TICKS(50));
}

static esp_err_t ads1115_read_reg(i2c_port_t i2c_num, uint8_t i2c_addr,
                                  uint8_t reg, uint16_t *out)
{
    esp_err_t err = i2c_master_write_read_device(i2c_num, i2c_addr, &reg, 1,
                                                 (uint8_t *)out, 2, pdMS_TO_TICKS(50));
    if (err == ESP_OK) {
        *out = (uint16_t)((*out << 8) | (*out >> 8)); /* MSB first → host */
    }
    return err;
}

/* ───────────────────────────── wewnętrzne pomoce ────────────────────────── */
static uint16_t ads1115_build_cfg(uint8_t channel)
{
    return ADS1115_CFG_OS_SINGLE       |
           ADS1115_MUX_SINGLE(channel) |
           ADS1115_CFG_PGA_4_096       |
           ADS1115_CFG_MODE_SINGLESHOT |
           ADS1115_CFG_DR_860SPS       |
           ADS1115_CFG_COMP_QUE_DISABLE;
}

static esp_err_t ads1115_start_single(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t ch)
{
    uint16_t cfg = ads1115_build_cfg(ch);
    return ads1115_write_reg(i2c_num, i2c_addr, ADS1115_REG_CONFIG, cfg);
}

static esp_err_t ads1115_wait_ready(i2c_port_t i2c_num, uint8_t i2c_addr)
{
    /* Poll bit OS – gdy 0 → konwersja trwa, gdy 1 → gotowe */
    const TickType_t timeout = pdMS_TO_TICKS(20); /* ~ 1,2 ms konwersji  */
    TickType_t start = xTaskGetTickCount();
    while (1) {
        uint16_t cfg;
        ESP_RETURN_ON_ERROR(ads1115_read_reg(i2c_num, i2c_addr, ADS1115_REG_CONFIG, &cfg), TAG, "read cfg");
        if (cfg & ADS1115_CFG_OS_SINGLE) {
            return ESP_OK; /* gotowe */
        }
        if ((xTaskGetTickCount() - start) > timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static esp_err_t ads1115_read_conversion(i2c_port_t i2c_num, uint8_t i2c_addr, int16_t *raw)
{
    uint16_t tmp;
    ESP_RETURN_ON_ERROR(ads1115_read_reg(i2c_num, i2c_addr, ADS1115_REG_CONVERSION, &tmp), TAG, "read conv");
    *raw = (int16_t)tmp;
    return ESP_OK;
}

static inline float raw_to_current(int16_t raw)
{
    float v_out = raw * ADS1115_LSB_VOLTAGE;
    return v_out / (INA180_GAIN * SHUNT_RESISTOR_OHM);
}

/* ─────────────────────────────  API ─────────────────────────────────────── */

esp_err_t current_sensor_read_current_channel(i2c_port_t i2c_num,
                                              uint8_t     i2c_addr,
                                              uint8_t     channel,
                                              float      *current)
{
    if (channel >= CURRENT_SENSOR_NUM_CHANNELS || !current) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(ads1115_start_single(i2c_num, i2c_addr, channel), TAG, "start");
    ESP_RETURN_ON_ERROR(ads1115_wait_ready(i2c_num, i2c_addr), TAG, "ready");

    int16_t raw;
    ESP_RETURN_ON_ERROR(ads1115_read_conversion(i2c_num, i2c_addr, &raw), TAG, "raw");

    *current = raw_to_current(raw);
    return ESP_OK;
}

esp_err_t current_sensor_read_current_multi(i2c_port_t i2c_num,
                                            uint8_t     i2c_addr,
                                            float       currents[CURRENT_SENSOR_NUM_CHANNELS])
{
    if (!currents) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t ch = 0; ch < CURRENT_SENSOR_NUM_CHANNELS; ++ch) {
        esp_err_t err = current_sensor_read_current_channel(i2c_num, i2c_addr, ch, &currents[ch]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "channel %u failed (%s)", ch, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}