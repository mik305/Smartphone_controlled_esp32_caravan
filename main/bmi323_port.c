#include "bmi323.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <string.h>
#include "esp_rom_sys.h"  
#define I2C_PORT    I2C_NUM_0
#define BMI323_ADDR 0x69            // SDO = HIGH

/* --- Transport callbacks --- */
static int8_t bmi3_i2c_read(uint8_t reg, uint8_t *data,
                            uint32_t len, void *intf_ptr)
{
    return i2c_master_write_read_device(I2C_PORT, BMI323_ADDR,
                                        &reg, 1, data, len,
                                        100 / portTICK_PERIOD_MS) == ESP_OK ? 0 : -1;
}
static int8_t bmi3_i2c_write(uint8_t reg, const uint8_t *data,
                             uint32_t len, void *intf_ptr)
{
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_write_to_device(I2C_PORT, BMI323_ADDR,
                                      buf, len + 1,
                                      100 / portTICK_PERIOD_MS) == ESP_OK ? 0 : -1;
}
static void bmi3_delay_us(uint32_t period, void *intf_ptr)
{
    esp_rom_delay_us(period);  
}

/* --- Public handle (export, by ref) --- */
struct bmi3_dev bmi323_dev;
