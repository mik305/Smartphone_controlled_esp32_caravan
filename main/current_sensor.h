// current_sensor.h
#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Ilość dostępnych kanałów ADS1115 w trybie single‑ended */
#define CURRENT_SENSOR_NUM_CHANNELS 4

/**
 * @brief Odczyt prądu dla pojedynczego kanału.
 *
 * @param i2c_num  Port I²C ESP‑IDF (np. I2C_NUM_0)
 * @param i2c_addr 7‑bitowy adres ADS1115 (zależny od pinu ADDR)
 * @param channel  Numer kanału 0‑3 (AIN0…AIN3)
 * @param current  Wynik [A]
 */
esp_err_t current_sensor_read_current_channel(i2c_port_t i2c_num,
                                              uint8_t     i2c_addr,
                                              uint8_t     channel,
                                              float      *current);

/**
 * @brief Jednoczesny odczyt prądu na 4 kanałach.
 *
 * Funkcja wykonuje sekwencyjnie cztery pojedyncze konwersje i zwraca tablicę
 * wyników w amperach.
 *
 * @param currents  Tablica [4] – wypełniona kolejno AIN0, AIN1, AIN2, AIN3.
 */
esp_err_t current_sensor_read_current_multi(i2c_port_t i2c_num,
                                            uint8_t     i2c_addr,
                                            float       currents[CURRENT_SENSOR_NUM_CHANNELS]);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSOR_H */