/* levelling.c – prosta procedura samopoziomowania
 * ------------------------------------------------
 * • wykorzystuje wygładzone odczyty z BMI323 (bmi323_accel_g[])
 *   aby sterować czterema siłownikami liniowymi.
 * • celem jest uzyskanie |ax|, |ay| ≤ LEVEL_TOL oraz az ≈ 1 g.
 * • algorytm: regulacja proporcjonalna + martwa strefa.
 * • procedura uruchamiana przez level_start();
 *   zatrzymuje się automatycznie po wstępnym wyrównaniu lub
 *   po przekroczeniu limitu czasu (fail‑safe).
 */

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

const uint8_t adc_addr = 0x48;
float i[CURRENT_SENSOR_NUM_CHANNELS];

/* ───── ustawienia PID (P‑only) i tolerancje ────────────────────────── */
#define LEVEL_TOL          0.01f   /* ±3 mg dokładność docelowa         */
#define KP                 100000.0f    /* [duty / g] – skalowanie błędu     */
#define MAX_DUTY_CHANGE    2047      /* zabezp. krok‑po‑kroku (0…1023)    */
#define CYCLE_MS           150      /* takt regulatora                   */
#define TIMEOUT_SEC        25

static TaskHandle_t level_task_h   = NULL; 
void level_start(void);
void level_stop(void);
/* mapowanie politej przyczepy → kanały PWM
 *  1: lewy‑przód, 2: prawy‑przód, 3: lewy‑tył, 4: prawy‑tył           */
static const uint8_t act_pwm_fwd[4] = { PWM_CHANNEL_IN1, PWM_CHANNEL_IN1+2,
                                        PWM_CHANNEL_IN1+4, PWM_CHANNEL_IN1+6 };
static const uint8_t act_pwm_rev[4] = { PWM_CHANNEL_IN2, PWM_CHANNEL_IN2+2,
                                        PWM_CHANNEL_IN2+4, PWM_CHANNEL_IN2+6 };

/* ───── pomocnicze: sterowanie jednym siłownikiem ───────────────────── */
static void set_actuator_speed(int idx, int16_t duty) /* duty signed: +extend */
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


/* ───── wątek poziomowania ──────────────────────────────────────────── */
static void levelling_task(void *pv)
{
    TickType_t t0 = xTaskGetTickCount();

    ESP_LOGI(TAG, "start");

    while (true)
    {
        /* 1. odczyt bieżącego nachylenia */
        float ex = bmi323_accel_g[0];   /* X‑błąd            */
        float ey = bmi323_accel_g[1];   /* Y‑błąd            */

        /* 2. czy już w tolerancji? */
        if (fabsf(ex) < LEVEL_TOL && fabsf(ey) < LEVEL_TOL) {
            ESP_LOGI(TAG, "levelled: ax=%.4f ay=%.4f", ex, ey);
            break;                      /* gotowe */
        }

        /* 3. oblicz korekty – bardzo uproszczone mapowanie
         *    ex>0 → pochylony przód w dół, trzeba OPUŚCIĆ przód / PODNIEŚĆ tył
         *    ey>0 → prawa strona w dół, trzeba OPUŚCIĆ prawą / PODNIEŚĆ lewą */
        int16_t d_front  = (int16_t)(-KP * ex);  /* przód = 1 & 2 */
        int16_t d_rear   = (int16_t)( KP * ex);  /* tył  = 3 & 4 */
        int16_t d_left   = (int16_t)( -KP * ey);  /* lewe = 1 & 3 */
        int16_t d_right  = (int16_t)( -KP * ey);  /* prawe = 2 & 4 */

        /* nałóż ograniczenie przyrostu */
        #define CLAMP(d)  do{ if((d) >  MAX_DUTY_CHANGE) (d)= MAX_DUTY_CHANGE; \
                             if((d) < -MAX_DUTY_CHANGE) (d)=-MAX_DUTY_CHANGE; }while(0)
        CLAMP(d_front); CLAMP(d_rear); CLAMP(d_left); CLAMP(d_right);


        /* --- oblicz skuteczne duty dla każdego siłownika ------------------ */
        int16_t duty_lf = d_front + d_left;    /* left-front  (1)  */
        int16_t duty_rf = d_front + d_right;   /* right-front (2)  */
        int16_t duty_lr = d_rear  + d_left;    /* left-rear  (3)   */
        int16_t duty_rr = d_rear  + d_right;   /* right-rear (4)   */

        /* logowanie – widać jakie sygnały dostają siłowniki */
        //ESP_LOGI(TAG, "duty  LF=%d  RF=%d  LR=%d  RR=%d",
               // duty_lf, duty_rf, duty_lr, duty_rr);

        
        /* 4. wypadkowe komendy dla 4 siłowników */
        set_actuator_speed(0, duty_lf);
        set_actuator_speed(1, duty_rf);
        set_actuator_speed(2, -duty_lr);
        set_actuator_speed(3, -duty_rr);

       /* if (current_sensor_read_current_multi(I2C_NUM_0, adc_addr, i) == ESP_OK) {
            printf("M1: %.3f A | M2: %.3f A | M3: %.3f A | M4: %.3f A\n",
                   i[0], i[1], i[2], i[3]);
                   
        } else {
            printf("Błąd pomiaru!\n");
        }*/

        /* 5. warunek timeout */
        if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(TIMEOUT_SEC*1000)) {
            ESP_LOGW(TAG, "timeout – poziomowanie przerwane");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(CYCLE_MS));
    }

    /* stop – wszystkie siłowniki OFF */
    for (int i = 0; i < 4; i++) set_actuator_speed(i, 0);

    ESP_LOGI(TAG, "done");
    vTaskDelete(level_task_h);
}

/* ───── publiczne API ──────────────────────────────────────────────── */
void level_start(void)
{
    /* uruchom task tylko, jeżeli nie istnieje */
    if (xTaskCreate(levelling_task, "level", 4096, NULL, 6, NULL) != pdPASS)
        ESP_LOGW(TAG, "levelling already running");
}
void level_stop(void)              /* ←  NOWA  */
{   
    actuators_all_stop();
    if (level_task_h) {
                    /* jeśli task działa…           */
        vTaskDelete(level_task_h); /*  …zabij go                    */
        level_task_h = NULL;
        /* opcjonalnie – natychmiast zatrzymaj silniki */
        
    }
}
bool level_busy(void) { return level_task_h != NULL; }

/* ───────── HTTP wrappers ───────── */
