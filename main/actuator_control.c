#include "actuator_control.h"

void actuator_pwm_init(void) {
    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channels[] = {
        {.gpio_num = MOTOR_IN1_GPIO, .speed_mode = PWM_MODE, .channel = PWM_CHANNEL_IN1, .timer_sel = PWM_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = MOTOR_IN2_GPIO, .speed_mode = PWM_MODE, .channel = PWM_CHANNEL_IN2, .timer_sel = PWM_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = MOTOR_IN3_GPIO, .speed_mode = PWM_MODE, .channel = PWM_CHANNEL_IN3, .timer_sel = PWM_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = MOTOR_IN4_GPIO, .speed_mode = PWM_MODE, .channel = PWM_CHANNEL_IN4, .timer_sel = PWM_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = MOTOR_IN5_GPIO, .speed_mode = PWM_MODE, .channel = PWM_CHANNEL_IN5, .timer_sel = PWM_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = MOTOR_IN6_GPIO, .speed_mode = PWM_MODE, .channel = PWM_CHANNEL_IN6, .timer_sel = PWM_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = MOTOR_IN7_GPIO, .speed_mode = PWM_MODE, .channel = PWM_CHANNEL_IN7, .timer_sel = PWM_TIMER, .duty = 0, .hpoint = 0},
        {.gpio_num = MOTOR_IN8_GPIO, .speed_mode = PWM_MODE, .channel = PWM_CHANNEL_IN8, .timer_sel = PWM_TIMER, .duty = 0, .hpoint = 0}
    };

    for (int i = 0; i < 8; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&channels[i]));
    }
}