#ifndef LEVELLING_H
#define LEVELLING_H

#include "esp_err.h"

esp_err_t level_http_start(void);   /* /auto_level_start  */
esp_err_t level_http_stop(void);    /* /auto_level_stop   */
esp_err_t simple_level_http_start(void);

#endif