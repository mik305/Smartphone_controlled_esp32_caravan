#include "i2c_scanner.h"
#include "esp_log.h"

#include "esp_log.h"
#include "stdio.h"

static const char* TAG = "LSM6DSOX_SCANNER";

// Inicjalizacja I2C
void lsm6dsox_scanner_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = LSM6DSOX_SDA_PIN,
        .scl_io_num = LSM6DSOX_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = LSM6DSOX_I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(LSM6DSOX_I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(LSM6DSOX_I2C_PORT, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C Scanner zainicjalizowany (SCL: %d, SDA: %d)", LSM6DSOX_SCL_PIN, LSM6DSOX_SDA_PIN);
}

// Skanowanie I2C i wyszukiwanie LSM6DSOX
void lsm6dsox_scan_i2c(void) {
    ESP_LOGI(TAG, "Rozpoczęcie skanowania I2C w poszukiwaniu LSM6DSOX...");
    printf("\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");

    bool lsm6dsox_found = false;

    for (uint8_t row = 0; row < 0x80; row += 0x10) {
        printf("%02x:", row);
        for (uint8_t col = 0; col < 0x10; col++) {
            uint8_t addr = row + col;
            
            // Pomijamy adresy zarezerwowane (0x00-0x07 i 0x78-0x7F)
            if (addr < 0x08 || addr > 0x77) {
                printf(" --");
                continue;
            }

            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(cmd);

            esp_err_t ret = i2c_master_cmd_begin(LSM6DSOX_I2C_PORT, cmd, 100 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            if (ret == ESP_OK) {
                printf(" %02x", addr);
                if (addr == LSM6DSOX_ADDR_SA0_LOW || addr == LSM6DSOX_ADDR_SA0_HIGH) {
                    lsm6dsox_found = true;
                    ESP_LOGI(TAG, "Znaleziono LSM6DSOX pod adresem: 0x%02X", addr);
                }
            } else {
                printf(" --");
            }
        }
        printf("\n");
    }

    if (!lsm6dsox_found) {
        ESP_LOGW(TAG, "Nie znaleziono LSM6DSOX (sprawdź połączenia)");
    }
    ESP_LOGI(TAG, "Skanowanie zakończone\n");
}