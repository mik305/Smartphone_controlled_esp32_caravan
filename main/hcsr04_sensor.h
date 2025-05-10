#ifndef HCSR04_SENSOR_H
#define HCSR04_SENSOR_H

#include <stdint.h>

#define NUM_HCSR04_SENSORS 4
#define HCSR04_TIMEOUT_US 30000 // 30ms timeout (~5m max distance)
#define HCSR04_INVALID_DISTANCE 0

typedef struct {
    uint8_t trigger_pin;
    uint8_t echo_pin;
    float distance_cm;
    uint32_t last_update;
} hcsr04_sensor_t;

void hcsr04_sensor_init(void);
void hcsr04_task(void *pvParameters);
extern hcsr04_sensor_t hcsr04_sensors[NUM_HCSR04_SENSORS];

#endif