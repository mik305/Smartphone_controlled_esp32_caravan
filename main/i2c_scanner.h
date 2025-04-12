#ifndef I2C_SCANNER_H
#define I2C_SCANNER_H


#include "driver/i2c.h"

#define LSM6DSOX_SCL_PIN     1   // GPIO 1 dla SCL
#define LSM6DSOX_SDA_PIN     2   // GPIO 2 dla SDA
#define LSM6DSOX_I2C_PORT    I2C_NUM_0  // Użyj I2C_NUM_1, aby uniknąć konfliktu z HDC1080
#define LSM6DSOX_I2C_FREQ    100000     // 100 kHz

// Adresy LSM6DSOX
#define LSM6DSOX_ADDR_SA0_LOW  0x6A
#define LSM6DSOX_ADDR_SA0_HIGH 0x6B

void lsm6dsox_scanner_init(void);
void lsm6dsox_scan_i2c(void);

#endif