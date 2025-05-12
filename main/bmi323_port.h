/*  bmi323_port.h
 *  Mostek I²C dla Bosch BMI323 – deklaracje widoczne poza bmi323_port.c
 */
#pragma once                /* zapobiega podwójnemu włączeniu */

#include "bmi323.h"         /* struktury i definicje API */

/* ------------------------------------------------------------------ */
/*  Globalna struktura urządzenia                                     */
/*  Zdefiniowana w  bmi323_port.c  i używana w  main.c  (extern)      */
/* ------------------------------------------------------------------ */
extern struct bmi3_dev bmi323_dev;

/* ------------------------------------------------------------------ */
/*  (opcjonalnie) prototypy funkcji transportowych;                   */
/*  przydadzą się, jeżeli chcesz ich używać poza bmi323_port.c        */
/* ------------------------------------------------------------------ */
int8_t bmi3_i2c_read(uint8_t reg, uint8_t *data,
                     uint32_t len, void *intf_ptr);

int8_t bmi3_i2c_write(uint8_t reg, const uint8_t *data,
                      uint32_t len, void *intf_ptr);

void   bmi3_delay_us(uint32_t period, void *intf_ptr);
