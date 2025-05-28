#include "bmi323_sensor.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>


#define TAG           "BMI323"
#define BMI323_ADDR   0x69
#define I2C_PORT      I2C_NUM_0

static struct bmi3_dev dev;

volatile float bmi323_accel_g[3]  = {0};   /* X Y Z g */
volatile float bmi323_gyro_dps[3] = {0};   /* X Y Z °/s */
volatile float bmi323_temp_c      = 0.0f;  
/* ─ filtr wygładzający (EMA) ─*/
//static float ema_acc[3] = {0};
//#define EMA_ALPHA   0.10f          /* 0…1  (mniejsza → mocniejsze wygł.) */

/* ───────── I²C callbacks ───────── */
static int8_t i2c_read(uint8_t reg, uint8_t *data,
                       uint32_t len, void *intf_ptr)
{
    return i2c_master_write_read_device(I2C_PORT, BMI323_ADDR,
                                        &reg, 1, data, len,
                                        100 / portTICK_PERIOD_MS) == ESP_OK ? 0 : -1;
}
static int8_t i2c_write(uint8_t reg, const uint8_t *data,
                        uint32_t len, void *intf_ptr)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_write_to_device(I2C_PORT, BMI323_ADDR,
                                      buf, len + 1,
                                      100 / portTICK_PERIOD_MS) == ESP_OK ? 0 : -1;
}
static void delay_us(uint32_t us, void *ctx) { esp_rom_delay_us(us); }

/* ───────── init ───────── */
esp_err_t bmi323_temp_init(void)
{
    uint8_t addr = BMI323_ADDR;

    dev.intf           = BMI3_I2C_INTF;
    dev.read           = i2c_read;
    dev.write          = i2c_write;
    dev.delay_us       = delay_us;
    dev.intf_ptr       = &addr;
    dev.read_write_len = 32;

    if (bmi323_init(&dev) != BMI323_OK) {
        ESP_LOGE(TAG, "bmi323_init failed");
        return ESP_FAIL;
    }

    /* Akcelerometr NORMAL 100 Hz ±2 g */
    struct bmi3_sens_config cfg_acc = { 0 };
    cfg_acc.type             = BMI323_ACCEL;
    cfg_acc.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;
    cfg_acc.cfg.acc.odr      = BMI3_ACC_ODR_100HZ;
    cfg_acc.cfg.acc.range    = BMI3_ACC_RANGE_2G;
    cfg_acc.cfg.acc.bwp      = BMI3_ACC_BW_ODR_QUARTER;
    cfg_acc.cfg.acc.avg_num    = BMI3_ACC_AVG64;

    /* Żyroskop NORMAL 100 Hz ±1000 dps */
    struct bmi3_sens_config cfg_gyr = { 0 };
    cfg_gyr.type             = BMI323_GYRO;
    cfg_gyr.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
    cfg_gyr.cfg.gyr.odr      = BMI3_GYR_ODR_100HZ;
    cfg_gyr.cfg.gyr.range    = BMI3_GYR_RANGE_1000DPS;
    //cfg_gyr.cfg.gyr.bwp      = BMI3_GYR_BW_ODR_1;

    if (bmi323_set_sensor_config(&cfg_acc, 1, &dev) != BMI323_OK ||
        bmi323_set_sensor_config(&cfg_gyr, 1, &dev) != BMI323_OK) {
        ESP_LOGE(TAG, "set cfg failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BMI323 OK: temp, acc, gyro aktywne");
    return ESP_OK;
}

/* ───────── pomocnicze konwersje ───────── */
static inline float raw_to_g(int16_t raw)     { return raw / 16384.0f; }   /* ±2 g */
static inline float raw_to_dps(int16_t raw)   { return raw / 32.768f;  }   /* ±1000 dps */

/* ───────── task ───────── */
void bmi323_temp_task(void *pv)
{
    (void)pv;

    struct bmi3_sensor_data acc = { .type = BMI3_ACCEL };
    struct bmi3_sensor_data gyr = { .type = BMI3_GYRO  };
    uint16_t raw_t = 0;
    bool first = true;
    while (true)
    {
        //pominięcie pierwszego printa, bo zawierał domyslne wartosci rejestrow
        if (first) {
            first = false;
            // poczekaj 100 ms, wyrzuć początkowe “śmieci”
            vTaskDelay(pdMS_TO_TICKS(100));
            // opcjonalnie: jeden dummy-read, żeby wyrzucić rejestry
            bmi323_get_temperature_data(&raw_t, &dev);
            bmi323_get_sensor_data(&acc, 1, &dev);
            bmi323_get_sensor_data(&gyr, 1, &dev);
            continue;
        }

        /* temperatura */
        if (bmi323_get_temperature_data(&raw_t, &dev) != BMI323_OK) {
            ESP_LOGW(TAG, "temp read err");
        }
        float T = (int16_t)raw_t / 512.0f + 17.0f;
        bmi323_temp_c = T;

        
        
        /* akcelerometr ------------------------------------------------------ */
        float ax = NAN, ay = NAN, az = NAN;
        /* if (bmi323_get_sensor_data(&acc, 1, &dev) == BMI323_OK) {
            ax = raw_to_g(acc.sens_data.acc.x);
            ay = raw_to_g(acc.sens_data.acc.y);
            az = raw_to_g(acc.sens_data.acc.z);

            
            ema_acc[0] += EMA_ALPHA * (ax - ema_acc[0]);
            ema_acc[1] += EMA_ALPHA * (ay - ema_acc[1]);
            ema_acc[2] += EMA_ALPHA * (az - ema_acc[2]);

            bmi323_accel_g[0] = ema_acc[0];
            bmi323_accel_g[1] = ema_acc[1];
            bmi323_accel_g[2] = ema_acc[2];
        }*/

         if (bmi323_get_sensor_data(&acc, 1, &dev) == BMI323_OK) {
            ax = raw_to_g(acc.sens_data.acc.x);
            ay = raw_to_g(acc.sens_data.acc.y);
            az = raw_to_g(acc.sens_data.acc.z);
            bmi323_accel_g[0]  = ax + 0.0393;
            bmi323_accel_g[1] = ay + 0.0115 ;
            bmi323_accel_g[2]  = az;
        }

        /* żyroskop */
        float gx = NAN, gy = NAN, gz = NAN;
        if (bmi323_get_sensor_data(&gyr, 1, &dev) == BMI323_OK) {
            gx = raw_to_dps(gyr.sens_data.gyr.x);
            gy = raw_to_dps(gyr.sens_data.gyr.y);
            gz = raw_to_dps(gyr.sens_data.gyr.z);
            bmi323_gyro_dps[0] = gx;
            bmi323_gyro_dps[1] = gy;
            bmi323_gyro_dps[2] = gz;
        }

        /* log */
        /*ESP_LOGI(TAG,
         "T=%.1f°C | ACC=%.4f %.4f %.4f g | GYR=%.1f %.1f %.1f dps",
         T, ema_acc[0], ema_acc[1], ema_acc[2], gx, gy, gz);*/
       /* ESP_LOGI(TAG,
                 "T=%.1f°C | ACC=%.3f %.3f %.3f g | GYR=%.1f %.1f %.1f dps",
                 T, ax, ay, az, gx, gy, gz);*/

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
