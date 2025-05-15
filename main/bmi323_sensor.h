#pragma once
#include "bmi323.h"
#include "esp_err.h"

/* globalnie dostępne wyniki */
extern volatile float bmi323_accel_g[3];   /* g */
extern volatile float bmi323_gyro_dps[3];  /* °/s */
extern volatile float bmi323_temp_c; 

/* init + uruchom task; zwraca ESP_OK gdy BMI323 startuje */
esp_err_t bmi323_temp_init(void);
void      bmi323_temp_task(void *pv);
