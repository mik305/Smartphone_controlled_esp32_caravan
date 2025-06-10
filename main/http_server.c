#include "http_server.h"
#include "actuator_control.h"
#include "lsm6dsox_sensor.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

// Uchwyt do zadania sterującego
static TaskHandle_t control_task_handle = NULL;
#define ACTUATOR_ID 1 // Sterowanie siłownikiem nr 1

// Referencje do pliku HTML
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

/**
 * @brief Zadanie sterujące siłownikiem na podstawie odczytu osi X akcelerometru.
 * * W pętli odczytuje globalną zmienną accel_g[0], mapuje jej wartość
 * z zakresu [-1, 1] na sygnał PWM i steruje siłownikiem.
 */
static void control_loop_task(void *pvParameters) {
    while (1) {
        float x_accel = accel_g[0];

        // Ograniczenie wartości do zakresu [-1.0, 1.0]
        if (x_accel > 1.0f) x_accel = 1.0f;
        if (x_accel < -1.0f) x_accel = -1.0f;

        // Obliczenie wypełnienia PWM na podstawie wartości bezwzględnej przyspieszenia
        uint32_t duty = (uint32_t)(fabs(x_accel) * PWM_DUTY);

        uint8_t ch_fwd = PWM_CHANNEL_IN1 + (ACTUATOR_ID - 1) * 2;
        uint8_t ch_rev = PWM_CHANNEL_IN2 + (ACTUATOR_ID - 1) * 2;

        if (x_accel > 0.05) { // Wysuwanie (z małą strefą nieczułości)
            ledc_set_duty(PWM_MODE, ch_fwd, duty);
            ledc_set_duty(PWM_MODE, ch_rev, 0);
        } else if (x_accel < -0.05) { // Chowanie (z małą strefą nieczułości)
            ledc_set_duty(PWM_MODE, ch_fwd, 0);
            ledc_set_duty(PWM_MODE, ch_rev, duty);
        } else { // Zatrzymanie
             ledc_set_duty(PWM_MODE, ch_fwd, 0);
             ledc_set_duty(PWM_MODE, ch_rev, 0);
        }
        
        ledc_update_duty(PWM_MODE, ch_fwd);
        ledc_update_duty(PWM_MODE, ch_rev);

        vTaskDelay(pdMS_TO_TICKS(20)); // Częstotliwość pętli sterowania
    }
}

// Handler HTTP uruchamiający zadanie sterujące
static esp_err_t start_control_handler(httpd_req_t *req) {
    if (control_task_handle == NULL) {
        xTaskCreate(control_loop_task, "control_task", 4096, NULL, 5, &control_task_handle);
    }
    httpd_resp_send(req, "Sterowanie uruchomione", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler HTTP zatrzymujący zadanie sterujące
static esp_err_t stop_control_handler(httpd_req_t *req) {
    if (control_task_handle != NULL) {
        vTaskDelete(control_task_handle);
        control_task_handle = NULL;
    }

    // Zapewnienie zatrzymania siłownika
    uint8_t ch_fwd = PWM_CHANNEL_IN1 + (ACTUATOR_ID - 1) * 2;
    uint8_t ch_rev = PWM_CHANNEL_IN2 + (ACTUATOR_ID - 1) * 2;
    ledc_set_duty(PWM_MODE, ch_fwd, 0);
    ledc_set_duty(PWM_MODE, ch_rev, 0);
    ledc_update_duty(PWM_MODE, ch_fwd);
    ledc_update_duty(PWM_MODE, ch_rev);

    httpd_resp_send(req, "Sterowanie zatrzymane", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Handler dla głównego zasobu (serwuje stronę HTML)
static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 4;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(server, &root_uri);
        
        httpd_uri_t start_uri = { .uri = "/start", .method = HTTP_GET, .handler = start_control_handler };
        httpd_register_uri_handler(server, &start_uri);

        httpd_uri_t stop_uri = { .uri = "/stop", .method = HTTP_GET, .handler = stop_control_handler };
        httpd_register_uri_handler(server, &stop_uri);
    }
    return server;
}