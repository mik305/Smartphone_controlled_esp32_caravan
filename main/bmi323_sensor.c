/* ------------------------------------------------------------------
 * bmi323_sensor.h – header (v6) – fixed I²C address 0x69 + continuous mode
 * ------------------------------------------------------------------*/
#ifndef BMI323_SENSOR_H
#define BMI323_SENSOR_H

#include "driver/i2c.h"
#include "esp_err.h"

/* ===== I²C bus configuration ===== */
#define BMI323_I2C_NUM   I2C_NUM_0          /* change if you use I2C_NUM_1 */
#define BMI323_I2C_ADDR  0x69               /* SDO pulled high → 0x69 */

/* ===== Register map (16‑bit, little‑endian) ===== */
#define BMI323_CHIP_ID        0x00
#define BMI323_ERR_REG        0x01
#define BMI323_STATUS         0x02
#define BMI323_ACC_DATA_X     0x03
#define BMI323_ACC_DATA_Y     0x04
#define BMI323_ACC_DATA_Z     0x05
#define BMI323_GYR_DATA_X     0x06
#define BMI323_GYR_DATA_Y     0x07
#define BMI323_GYR_DATA_Z     0x08
#define BMI323_TEMP_DATA      0x09
#define BMI323_ACC_CONF_L     0x20  /* low byte */
#define BMI323_GYR_CONF_L     0x22  /* low byte */
#define BMI323_CMD            0x7E

/* ===== Commands ===== */
#define BMI323_SOFT_RESET_CMD 0xDEAFu

/* ===== Bit‑field helpers (16 bit words) ===== */
/* Accelerometer */
#define BMI323_ACC_ODR_100HZ        0x0008u
#define BMI323_ACC_RANGE_8G         (0x0002u << 4)
#define BMI323_ACC_BW_ODR_2         (0x0000u << 7)
#define BMI323_ACC_MODE_CONT        (0x0004u << 12)  /* continuous‑low‑power */
/* Gyroscope */
#define BMI323_GYR_ODR_100HZ        0x0008u
#define BMI323_GYR_RANGE_1000DPS    (0x0004u << 4)
#define BMI323_GYR_BW_ODR_2         (0x0000u << 7)
#define BMI323_GYR_MODE_CONT        (0x0004u << 12)
/* Status bits */
#define BMI323_STATUS_DRDY_ACC      (1u << 7)
#define BMI323_STATUS_DRDY_GYR      (1u << 6)
#define BMI323_STATUS_DRDY_TEMP     (1u << 5)

#ifdef __cplusplus
extern "C" {
#endif

void bmi323_init(void);
void bmi323_task(void *pvParameters);

extern volatile float bmi323_accel_g[3];
extern volatile float bmi323_gyro_dps[3];
extern volatile float bmi323_temp;

#ifdef __cplusplus
}
#endif

#endif /* BMI323_SENSOR_H */

/* ==================================================================
 * bmi323_sensor.c – implementation (v6) – continuous mode + raw dump
 * ==================================================================*/
#include "bmi323_sensor.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "BMI323";

volatile float bmi323_accel_g[3] = {0};
volatile float bmi323_gyro_dps[3]  = {0};
volatile float bmi323_temp = 0;

/* -------------------- I²C helpers -------------------- */
static esp_err_t bmi323_write_byte(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(BMI323_I2C_NUM, BMI323_I2C_ADDR,
                                      buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
}
static esp_err_t bmi323_read_bytes(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(BMI323_I2C_NUM, BMI323_I2C_ADDR,
                                        &reg, 1, data, len, 100 / portTICK_PERIOD_MS);
}
static esp_err_t bmi323_read_word(uint8_t reg_lsb, uint16_t *val)
{
    uint8_t buf[4] = {0};
    esp_err_t ret = bmi323_read_bytes(reg_lsb, buf, sizeof(buf));
    if (ret == ESP_OK)
        *val = (uint16_t)(buf[3] << 8) | buf[2];  /* discard 2 dummy bytes */
    return ret;
}
static esp_err_t bmi323_write_word(uint8_t reg_lsb, uint16_t val)
{
    esp_err_t ret = bmi323_write_byte(reg_lsb, (uint8_t)(val & 0xFF)); /* LSB first */
    if (ret != ESP_OK) return ret;
    return bmi323_write_byte(reg_lsb + 1, (uint8_t)(val >> 8));        /* MSB */
}
#define BMI323_RD_WORD(reg, dst)  bmi323_read_word(reg, dst)
#define BMI323_WR_WORD(reg, val)  bmi323_write_word(reg, val)

/* -------------------- Conversion helpers -------------------- */
static inline float acc_raw_to_g(int16_t raw){ return raw / 4096.0f; }
static inline float gyr_raw_to_dps(int16_t raw){ return raw / 32.768f; }
static inline float temp_raw_to_degC(int16_t r){ return 23.0f + ((float)r - 0x8000) * 0.125f; }

/* -------------------- Diagnostic dump -------------------- */
static void bmi323_dump_regs(void)
{
    for (uint8_t a = 0x00; a < 0x24; a += 2){
        uint16_t v; if (BMI323_RD_WORD(a, &v) == ESP_OK)
            ESP_LOGI(TAG, "Reg 0x%02X = 0x%04X", a, v);
    }
}

/* -------------------- Init -------------------- */
void bmi323_init(void)
{
    uint16_t chip_id;
    if (BMI323_RD_WORD(BMI323_CHIP_ID, &chip_id) != ESP_OK || (chip_id & 0xFF) != 0x43){
        ESP_LOGE(TAG, "BMI323 not responding at 0x%02X; CHIP_ID=0x%04X", BMI323_I2C_ADDR, chip_id);
        return; }

    /* 1. Soft reset */
    if (bmi323_write_word(BMI323_CMD, BMI323_SOFT_RESET_CMD) != ESP_OK){
        ESP_LOGE(TAG, "soft‑reset failed");
        return; }
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 2. Continuous‑low‑power mode */
    uint16_t acc_conf = BMI323_ACC_ODR_100HZ | BMI323_ACC_RANGE_8G | BMI323_ACC_BW_ODR_2 | BMI323_ACC_MODE_CONT;
    uint16_t gyr_conf = BMI323_GYR_ODR_100HZ | BMI323_GYR_RANGE_1000DPS | BMI323_GYR_BW_ODR_2 | BMI323_GYR_MODE_CONT;

    if (BMI323_WR_WORD(BMI323_ACC_CONF_L, acc_conf) != ESP_OK ||
        BMI323_WR_WORD(BMI323_GYR_CONF_L, gyr_conf) != ESP_OK){
        ESP_LOGE(TAG, "config write failed");
        return; }

    vTaskDelay(pdMS_TO_TICKS(5));
    uint16_t err; if (BMI323_RD_WORD(BMI323_ERR_REG, &err) == ESP_OK && (err & 0x0060)){
        ESP_LOGW(TAG, "ERR_REG = 0x%04X – configuration errors", err);
        bmi323_dump_regs();
        return; }

    ESP_LOGI(TAG, "BMI323 initialised (cont‑LP) at 0x%02X", BMI323_I2C_ADDR);
}

/* -------------------- Sample read -------------------- */
static void bmi323_read_sample(void)
{
    uint16_t st; if (BMI323_RD_WORD(BMI323_STATUS, &st) != ESP_OK) return;
    if (!(st & BMI323_STATUS_DRDY_ACC) || !(st & BMI323_STATUS_DRDY_GYR)) return;

    uint16_t ax, ay, az, gx, gy, gz, tp;
    if (BMI323_RD_WORD(BMI323_ACC_DATA_X, &ax) != ESP_OK) return;
    if (BMI323_RD_WORD(BMI323_ACC_DATA_Y, &ay) != ESP_OK) return;
    if (BMI323_RD_WORD(BMI323_ACC_DATA_Z, &az) != ESP_OK) return;
    if (BMI323_RD_WORD(BMI323_GYR_DATA_X, &gx) != ESP_OK) return;
    if (BMI323_RD_WORD(BMI323_GYR_DATA_Y, &gy) != ESP_OK) return;
    if (BMI323_RD_WORD(BMI323_GYR_DATA_Z, &gz) != ESP_OK) return;
    if (BMI323_RD_WORD(BMI323_TEMP_DATA, &tp) != ESP_OK) return;

    bmi323_accel_g[0] = acc_raw_to_g((int16_t)ax);
    bmi323_accel_g[1] = acc_raw_to_g((int16_t)ay);
    bmi323_accel_g[2] = acc_raw_to_g((int16_t)az);

    bmi323_gyro_dps[0] = gyr_raw_to_dps((int16_t)gx);
    bmi323_gyro_dps[1] = gyr_raw_to_dps((int16_t)gy);
    bmi323_gyro_dps[2] = gyr_raw_to_dps((int16_t)gz);

    bmi323_temp = temp_raw_to_degC((int16_t)tp);
}

/* -------------------- Debug -------------------- */
static void bmi323_dbg_print(void)
{
    ESP_LOGI(TAG, "ACC [g]  : %6.3f %6.3f %6.3f", bmi323_accel_g[0], bmi323_accel_g[1], bmi323_accel_g[2]);
    ESP_LOGI(TAG, "GYR [dps]: %7.2f %7.2f %7.2f", bmi323_gyro_dps[0], bmi323_gyro_dps[1], bmi323_gyro_dps[2]);
    ESP_LOGI(TAG, "TMP [°C] : %6.2f",            bmi323_temp);
}

/* -------------------- FreeRTOS task -------------------- */
void bmi323_task(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(10); /* 100 Hz polling */
    uint32_t dbg = 0;

    while (true)
    {
        bmi323_read_sample();
        if (++dbg >= 100) /* ~1 s */
        {
            bmi323_dbg_print();
            dbg = 0;
        }
        vTaskDelay(period);
    }
}
