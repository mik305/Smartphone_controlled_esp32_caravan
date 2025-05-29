#include "actuator_control.h"
#include "bmi323_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "esp_err.h"

#define TAG "LEVELLING"

// Parametry regulatora
#define LEVEL_TOL          0.012f
#define KP                 100000.0f
#define CYCLE_MS           150
#define TIMEOUT_SEC        25
#define VERIFICATION_TIME_MS 2000
#define ACTUATOR_TEST_DUTY  -4095
#define ACTUATOR_TEST_THRESHOLD 0.015f
#define ACTUATOR_RETEST_THRESHOLD 0.02f // 0.015 * 0.75

static TaskHandle_t level_task_h = NULL;
static volatile bool level_stop_flag = false;

static const uint8_t act_pwm_fwd[4] = { PWM_CHANNEL_IN1, PWM_CHANNEL_IN1+2,
                                        PWM_CHANNEL_IN1+4, PWM_CHANNEL_IN1+6 };
static const uint8_t act_pwm_rev[4] = { PWM_CHANNEL_IN2, PWM_CHANNEL_IN2+2,
                                        PWM_CHANNEL_IN2+4, PWM_CHANNEL_IN2+6 };

static void set_actuator_speed(int idx, int16_t duty) 
{
    const uint8_t ch_fwd = act_pwm_fwd[idx];
    const uint8_t ch_rev = act_pwm_rev[idx];

    if (duty > 0) {
        duty = duty > 4095 ? 4095 : duty;
        ledc_set_duty(PWM_MODE, ch_fwd, duty);
        ledc_set_duty(PWM_MODE, ch_rev, 0);
    } else if (duty < 0) {
        duty = (-duty) > 4095 ? 4095 : -duty;
        ledc_set_duty(PWM_MODE, ch_fwd, 0);
        ledc_set_duty(PWM_MODE, ch_rev, duty);
    } else {
        ledc_set_duty(PWM_MODE, ch_fwd, 0);
        ledc_set_duty(PWM_MODE, ch_rev, 0);
    }
    ledc_update_duty(PWM_MODE, ch_fwd);
    ledc_update_duty(PWM_MODE, ch_rev);
}

static bool verify_stability() {
    TickType_t verification_start = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - verification_start) < pdMS_TO_TICKS(VERIFICATION_TIME_MS)) {
        float ex = bmi323_accel_g[0];
        float ey = bmi323_accel_g[1];
        
        if (fabsf(ex) > LEVEL_TOL || fabsf(ey) > LEVEL_TOL) {
            ESP_LOGI(TAG, "Weryfikacja nieudana: ax=%.4f ay=%.4f", ex, ey);
            return false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(CYCLE_MS));
    }
    return true;
}

static bool perform_levelling() {
    TickType_t t0 = xTaskGetTickCount();
    bool is_levelled = false;

    ESP_LOGI(TAG, "Rozpoczęcie fazy poziomowania");

    while (!is_levelled && (xTaskGetTickCount() - t0) < pdMS_TO_TICKS(TIMEOUT_SEC*1000)) {
        float ex = bmi323_accel_g[0];
        float ey = bmi323_accel_g[1];
        
        if (fabsf(ex) < LEVEL_TOL && fabsf(ey) < LEVEL_TOL) {
            ESP_LOGI(TAG, "Wstępnie wypoziomowano: ax=%.4f ay=%.4f", ex, ey);
            
            // Wyłącz wszystkie siłowniki
            for (int i = 0; i < 4; i++) set_actuator_speed(i, 0);
            
            // Weryfikacja stabilności
            if (verify_stability()) {
                ESP_LOGI(TAG, "Poziomowanie zakończone pomyślnie");
                is_levelled = true;
            } else {
                ESP_LOGI(TAG, "Ponowne poziomowanie...");
            }
        } else {
            // Kontynuuj poziomowanie
            int16_t d_front_back = (int16_t)(KP * ey);
            int16_t d_left_right = (int16_t)(KP * ex);
            
            int16_t duty_1 = -d_front_back - d_left_right;
            int16_t duty_2 = -d_front_back + d_left_right;
            int16_t duty_3 = d_front_back - d_left_right;
            int16_t duty_4 = d_front_back + d_left_right;
            
            set_actuator_speed(0, duty_1);
            set_actuator_speed(1, duty_2);
            set_actuator_speed(2, duty_3);
            set_actuator_speed(3, duty_4);
        }
        
        vTaskDelay(pdMS_TO_TICKS(CYCLE_MS));
    }

    // Wyłącz wszystkie siłowniki na koniec
    for (int i = 0; i < 4; i++) set_actuator_speed(i, 0);

    return is_levelled;
}

static void test_actuators() {
    ESP_LOGI(TAG, "Rozpoczęcie testu siłowników");
    bool platform_lifted = false;
    
    for (int i = 0; i < 4 && !platform_lifted; i++) {
        // 1. Odczyt początkowych wartości akcelerometru
        float initial_x = bmi323_accel_g[0];
        float initial_y = bmi323_accel_g[1];
        ESP_LOGI(TAG, "Testowanie siłownika %d, start: ax=%.4f ay=%.4f", 
                i+1, initial_x, initial_y);
        
        // 2. Uruchom siłownik i czekaj na zmianę > ACTUATOR_TEST_THRESHOLD
        set_actuator_speed(i, ACTUATOR_TEST_DUTY);
        TickType_t test_start = xTaskGetTickCount();
        bool threshold_reached = false;
        float current_x = 0, current_y = 0;
        
        while ((xTaskGetTickCount() - test_start) < pdMS_TO_TICKS(13000)) {
            current_x = bmi323_accel_g[0];
            current_y = bmi323_accel_g[1];
            
            if (fabsf(current_x - initial_x) > ACTUATOR_TEST_THRESHOLD || 
                fabsf(current_y - initial_y) > ACTUATOR_TEST_THRESHOLD) {
                threshold_reached = true;
                break;
            }
            
            vTaskDelay(pdMS_TO_TICKS(CYCLE_MS));
        }
        
        // 3. Zatrzymaj siłownik niezależnie od wyniku
        set_actuator_speed(i, 0);
        
        if (!threshold_reached) {
            ESP_LOGW(TAG, "Brak odpowiedzi siłownika %d", i+1);
            continue;
        }
        
        // 4. Przerwa 3 sekundy przed odczytem końcowym
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // 5. Odczyt nowych wartości akcelerometru po przerwie
        float new_x = bmi323_accel_g[0];
        float new_y = bmi323_accel_g[1];
        
        // Nowy warunek: sprawdzenie czy różnica między początkowym a końcowym odczytem > 0.05
        bool significant_change = (fabsf(new_x - initial_x) > 0.05f) || 
                                (fabsf(new_y - initial_y) > 0.05f);
        
        ESP_LOGI(TAG, "Siłownik %d odpowiedział, nowe wartości: ax=%.4f ay=%.4f", 
                i+1, new_x, new_y);
        ESP_LOGW(TAG, "Znaczna zmiana (0.05g): %s", significant_change ? "TAK" : "NIE");
        
        if (significant_change) {
            ESP_LOGW(TAG, "Wykryto znaczną zmianę - platforma prawdopodobnie w powietrzu");
            ESP_LOGW(TAG, "Ponowne testowanie siłownika %d z mniejszym progiem", i+1);
            
            // 6. Ponowne testowanie tego samego siłownika z mniejszym progiem
            initial_x = new_x;
            initial_y = new_y;
            
            set_actuator_speed(i, ACTUATOR_TEST_DUTY);
            test_start = xTaskGetTickCount();
            threshold_reached = false;
            
            while ((xTaskGetTickCount() - test_start) < pdMS_TO_TICKS(13000)) {
                current_x = bmi323_accel_g[0];
                current_y = bmi323_accel_g[1];
                
                if (fabsf(current_x - initial_x) > ACTUATOR_RETEST_THRESHOLD || 
                    fabsf(current_y - initial_y) > ACTUATOR_RETEST_THRESHOLD) {
                    threshold_reached = true;
                    break;
                }
                
                vTaskDelay(pdMS_TO_TICKS(CYCLE_MS));
            }
            
            set_actuator_speed(i, 0);
            
            if (threshold_reached) {
                ESP_LOGW(TAG, "Platforma uniesiona - przerwanie testu wszystkich siłowników");
                platform_lifted = true;
            } else {
                ESP_LOGW(TAG, "Platforma nie osiągnęła docelowego położenia - kontynuacja testu");
            }
        }
    }
    
    // Wyłącz wszystkie siłowniki na koniec
    for (int j = 0; j < 4; j++) {
        set_actuator_speed(j, 0);
    }
    
    if (platform_lifted) {
        ESP_LOGW(TAG, "Test przerwany - platforma została uniesiona");
    } else {
        ESP_LOGI(TAG, "Zakończono test wszystkich siłowników");
    }
}

static void levelling_task(void *pv)
{

    test_actuators();
    // 1. Faza początkowa poziomowania
    if (!perform_levelling()) {
        ESP_LOGW(TAG, "Początkowe poziomowanie nieudane - kontynuowanie testu");
    }

    // 2. Faza testowania siłowników
    test_actuators();

    // 3. Faza końcowa poziomowania
    if (!perform_levelling()) {
        ESP_LOGW(TAG, "Końcowe poziomowanie nieudane");
    }

    ESP_LOGI(TAG, "Zakończono pełną procedurę poziomowania i testowania");
    level_task_h = NULL;
    vTaskDelete(NULL);
}

esp_err_t level_http_start(void)
{
    if (level_task_h == NULL) {
        if (xTaskCreate(levelling_task, "level", 4096, NULL, 6, &level_task_h) != pdPASS) {
            ESP_LOGE(TAG, "Błąd tworzenia zadania poziomowania");
            return ESP_FAIL;
        }
        return ESP_OK;
    }
    ESP_LOGW(TAG, "Poziomowanie już działa");
    return ESP_OK;
}

esp_err_t level_http_stop(void)
{
    if (level_task_h != NULL) {
        vTaskDelete(level_task_h);
        level_task_h = NULL;
        for (int i = 0; i < 4; i++) set_actuator_speed(i, 0);
        ESP_LOGI(TAG, "Ręczne przerwanie poziomowania");
    }
    return ESP_OK;
}

static void simple_level_task(void *pv)
{
    perform_levelling();           /* ← bez testów siłowników itp. */
    level_task_h = NULL;           /* zwalniamy uchwyt            */
    vTaskDelete(NULL);
}

esp_err_t simple_level_http_start(void)
{
    /* jeżeli coś już pracuje – nie uruchamiamy równolegle */
    if (level_task_h != NULL) {
        ESP_LOGW(TAG, "Levelling already running");
        return ESP_ERR_INVALID_STATE;
    }

    if (xTaskCreate(simple_level_task, "simple_lvl", 4096,
                    NULL, 6, &level_task_h) != pdPASS) {
        ESP_LOGE(TAG, "Can't create simple_level_task");
        return ESP_FAIL;
    }
    return ESP_OK;
}