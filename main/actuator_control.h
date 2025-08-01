#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#include "driver/ledc.h"

#define MOTOR_IN1_GPIO 10
#define MOTOR_IN2_GPIO 11
#define MOTOR_IN3_GPIO 14
#define MOTOR_IN4_GPIO 21
#define MOTOR_IN5_GPIO 6
#define MOTOR_IN6_GPIO 7
#define MOTOR_IN7_GPIO 12
#define MOTOR_IN8_GPIO 13

#define PWM_FREQ_HZ 5000
#define PWM_DUTY 4095
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define PWM_TIMER LEDC_TIMER_0

#define PWM_CHANNEL_IN1 LEDC_CHANNEL_0
#define PWM_CHANNEL_IN2 LEDC_CHANNEL_1
#define PWM_CHANNEL_IN3 LEDC_CHANNEL_2
#define PWM_CHANNEL_IN4 LEDC_CHANNEL_3
#define PWM_CHANNEL_IN5 LEDC_CHANNEL_4
#define PWM_CHANNEL_IN6 LEDC_CHANNEL_5
#define PWM_CHANNEL_IN7 LEDC_CHANNEL_6
#define PWM_CHANNEL_IN8 LEDC_CHANNEL_7

void actuator_pwm_init(void);
void actuators_all_stop(void);
#endif