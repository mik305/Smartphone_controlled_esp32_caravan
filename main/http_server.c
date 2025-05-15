#include "http_server.h"
#include "hcsr04_sensor.h"
#include "hdc1080_sensor.h"
#include "lsm6dsox_sensor.h"
#include "actuator_control.h"
#include <string.h>
#include "esp_log.h"
#include "bmi323_sensor.h"

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, 
                   index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t sensor_get_handler(httpd_req_t *req)
{
    char resp[512];

    snprintf(resp, sizeof(resp),
        "{\"temperature\":%.2f,\"humidity\":%.2f,"
        "\"accel_x\":%.2f,\"accel_y\":%.2f,\"accel_z\":%.2f,"
        "\"gyro_x\":%.2f,\"gyro_y\":%.2f,\"gyro_z\":%.2f,"
        "\"bmi323_temp\":%.2f," 
        "\"bmi323_accel_x\":%.2f,\"bmi323_accel_y\":%.2f,\"bmi323_accel_z\":%.2f,"
        "\"bmi323_gyro_x\":%.2f,\"bmi323_gyro_y\":%.2f,\"bmi323_gyro_z\":%.2f,"
        "\"distance_1\":%.2f,\"distance_2\":%.2f,"
        "\"distance_3\":%.2f,\"distance_4\":%.2f}",
        (double)latest_temp, (double)latest_hum,
        (double)accel_g[0],  (double)accel_g[1],  (double)accel_g[2],
        (double)gyro_dps[0], (double)gyro_dps[1], (double)gyro_dps[2],
        (double)bmi323_temp_c,                               /* <-- NOWE */
        (double)bmi323_accel_g[0], (double)bmi323_accel_g[1], (double)bmi323_accel_g[2],
        (double)bmi323_gyro_dps[0], (double)bmi323_gyro_dps[1], (double)bmi323_gyro_dps[2],
        (double)hcsr04_sensors[0].distance_cm,
        (double)hcsr04_sensors[1].distance_cm,
        (double)hcsr04_sensors[2].distance_cm,
        (double)hcsr04_sensors[3].distance_cm);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}



static esp_err_t extend_get_handler(httpd_req_t *req) {
    int actuator_id = req->uri[8] - '0';
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2, PWM_DUTY);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2);
    httpd_resp_send(req, "Siłownik wysunięty", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t retract_get_handler(httpd_req_t *req) {
    int actuator_id = req->uri[strlen(req->uri) - 1] - '0';
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2, 0);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2, PWM_DUTY);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2);
    httpd_resp_send(req, "Siłownik chowany", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t stop_get_handler(httpd_req_t *req) {
    int actuator_id = req->uri[strlen(req->uri) - 1] - '0';
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2, 0);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2);
    httpd_resp_send(req, "Siłownik zatrzymany", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 20;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t sensor_uri = { .uri = "/sensor", .method = HTTP_GET, .handler = sensor_get_handler };
        httpd_register_uri_handler(server, &sensor_uri);

        for (int i = 1; i <= 4; i++) {
            char uri[20];
            snprintf(uri, sizeof(uri), "/extend_%d", i);
            httpd_uri_t u1 = { .uri = uri, .method = HTTP_GET, .handler = extend_get_handler };
            httpd_register_uri_handler(server, &u1);

            snprintf(uri, sizeof(uri), "/retract_%d", i);
            httpd_uri_t u2 = { .uri = uri, .method = HTTP_GET, .handler = retract_get_handler };
            httpd_register_uri_handler(server, &u2);

            snprintf(uri, sizeof(uri), "/stop_%d", i);
            httpd_uri_t u3 = { .uri = uri, .method = HTTP_GET, .handler = stop_get_handler };
            httpd_register_uri_handler(server, &u3);
        }
    }
    return server;
}