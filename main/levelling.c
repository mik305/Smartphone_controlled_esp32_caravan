
#include "actuator_control.h"   /* PWM_CHANNEL_*  i  ledc_*()        */
#include "bmi323_sensor.h"      /* globalne bmi323_accel_g[]          */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "esp_err.h"
#include "hcsr04_sensor.h"
#include "current_sensor.h"
#include "hdc1080_sensor.h"

#define TAG "LEVELLING"

// regulator P
#define LEVEL_TOL          0.01f 
#define KP                 100000.0f  
#define MAX_DUTY_CHANGE    2047      /* zabezp. krok‑po‑kroku (0…1023)    */
#define CYCLE_MS           150      /* takt regulatora                   */
#define TIMEOUT_SEC        25

static TaskHandle_t level_task_h   = NULL; 
void level_start(void);
void level_stop(void);

static const uint8_t act_pwm_fwd[4] = { PWM_CHANNEL_IN1, PWM_CHANNEL_IN1+2,
                                        PWM_CHANNEL_IN1+4, PWM_CHANNEL_IN1+6 };
static const uint8_t act_pwm_rev[4] = { PWM_CHANNEL_IN2, PWM_CHANNEL_IN2+2,
                                        PWM_CHANNEL_IN2+4, PWM_CHANNEL_IN2+6 };

static void set_actuator_speed(int idx, int16_t duty) 
{
    const uint8_t ch_fwd = act_pwm_fwd[idx];
    const uint8_t ch_rev = act_pwm_rev[idx];

    if (duty > 0) {
        if (duty > 4095) duty = 4095;
        ledc_set_duty(PWM_MODE, ch_fwd, duty);
        ledc_set_duty(PWM_MODE, ch_rev, 0);
        ledc_update_duty(PWM_MODE, ch_fwd);
        ledc_update_duty(PWM_MODE, ch_rev);
    } else if (duty < 0) {
        duty = -duty;
        if (duty > 4095) duty = 4095;
        ledc_set_duty(PWM_MODE, ch_fwd, 0);
        ledc_set_duty(PWM_MODE, ch_rev, duty);
        ledc_update_duty(PWM_MODE, ch_fwd);
        ledc_update_duty(PWM_MODE, ch_rev);
    } else {
        ledc_set_duty(PWM_MODE, ch_fwd, 0);
        ledc_set_duty(PWM_MODE, ch_rev, 0);
        ledc_update_duty(PWM_MODE, ch_fwd);
        ledc_update_duty(PWM_MODE, ch_rev);
    }
}

esp_err_t level_http_start(void)
{
    level_start();
    return ESP_OK;
}
esp_err_t level_http_stop(void)
{
    level_stop();
    return ESP_OK;
}

int16_t duty_lf_buf, duty_rf_buf, duty_lr_buf, duty_rr_buf;

static void levelling_task(void *pv)
{
    TickType_t t0 = xTaskGetTickCount();

    ESP_LOGI(TAG, "start");

    while (true)
    {
        float ex = bmi323_accel_g[0];   // dążymy do wartości 0 
        float ey = bmi323_accel_g[1];   // dążymy do wartości 0          

        if (fabsf(ex) < LEVEL_TOL && fabsf(ey) < LEVEL_TOL) {
            ESP_LOGI(TAG, "levelled: ax=%.4f ay=%.4f", ex, ey);
            break;                  
        }

        /* mapowanie
            ex>0 → pochylony w prawo, trzeba podnieść prawy bok i opuścić lewy
            ey>0 → pochylony w przód, trzeba podnieść przód i opuścic tył*/

        int16_t d_front_back  = (int16_t)( KP * ey); 
        int16_t d_left_right   = (int16_t)( KP * ex);  
        
        /* nałóż ograniczenie przyrostu */
        //#define CLAMP(d)  do{ if((d) >  MAX_DUTY_CHANGE) (d)= MAX_DUTY_CHANGE; 
                             //if((d) < -MAX_DUTY_CHANGE) (d)=-MAX_DUTY_CHANGE; }while(0)
        
        /* duty dla każdego siłownika  */
        int16_t duty_1 = -d_front_back - d_left_right;    // prawy-front
        int16_t duty_2 = -d_front_back + d_left_right; ;   // lewy-front
        int16_t duty_3 = d_front_back - d_left_right;    // prawy-tylny
        int16_t duty_4 = d_front_back + d_left_right;   // lewy-tylny
    
        
        //CLAMP(d_front); CLAMP(d_rear); CLAMP(d_left); CLAMP(d_right);

        // jakie PWMy dostają siłowniki 
        ESP_LOGI(TAG, "duty  LF=%d  RF=%d  LR=%d  RR=%d",
                duty_1, duty_2, duty_3, duty_4);

       
        set_actuator_speed(0, duty_1);
        set_actuator_speed(1, duty_2);
        set_actuator_speed(2, duty_3);
        set_actuator_speed(3, duty_4);

        // jeśli nie uda się wypoziomować po zadanym czasie, przerwij procedurę
        if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(TIMEOUT_SEC*1000)) {
            ESP_LOGW(TAG, "timeout – poziomowanie przerwane");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(CYCLE_MS));
    }

    /* wszystkie siłowniki OFF */
    for (int i = 0; i < 4; i++) set_actuator_speed(i, 0);

    ESP_LOGI(TAG, "done");
    vTaskDelete(level_task_h);
}

void level_start(void)
{
    // uruchom task, jeżeli nie istnieje
    if (xTaskCreate(levelling_task, "level", 4096, NULL, 6, NULL) != pdPASS)
        ESP_LOGW(TAG, "levelling already running");
}
void level_stop(void)     
{   
    actuators_all_stop();
    if (level_task_h) {
        vTaskDelete(level_task_h); 
        level_task_h = NULL;
    }
}
bool level_busy(void) { return level_task_h != NULL; }
