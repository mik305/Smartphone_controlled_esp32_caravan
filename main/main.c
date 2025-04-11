#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_http_server.h"
#include "driver/ledc.h"
#include "esp_mac.h"
#include "driver/i2c.h"

#define CONFIG_ESP_WIFI_SSID       "ESP32_WiFi"
#define CONFIG_ESP_WIFI_PASSWORD   "12345678"
#define CONFIG_ESP_WIFI_CHANNEL    1
#define CONFIG_ESP_MAX_STA_CONN    4
#define CONFIG_ESP_MAXIMUM_RETRY   5


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

#define I2C_MASTER_SCL_IO           39
#define I2C_MASTER_SDA_IO           38
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define HDC1080_ADDR                0x40
#define HDC1080_TEMP_REG            0x00
#define HDC1080_HUMI_REG            0x01
#define HDC1080_CONFIG_REG          0x02


volatile float latest_temp = 0.0f;
volatile float latest_hum = 0.0f;

const char* html_page =
    "<!DOCTYPE html><html><body><h1>ESP32 Sterowanie Silownikami</h1>"
    "<h3>Dane z czujnika:</h3>"
    "<div id='sensorData'>Ladowanie...</div>"
    "<h3>Silownik 1</h3>"
    "<label><input type=\"radio\" name=\"actuator_1\" onclick=\"toggleActuator(1, 'extend')\" /> Wysuwaj</label><br>"
    "<label><input type=\"radio\" name=\"actuator_1\" onclick=\"toggleActuator(1, 'retract')\" /> Chowaj</label><br>"
    "<label><input type=\"radio\" name=\"actuator_1\" onclick=\"toggleActuator(1, 'stop')\" checked /> Stop</label><br>"
    "<h3>Silownik 2</h3>"
    "<label><input type=\"radio\" name=\"actuator_2\" onclick=\"toggleActuator(2, 'extend')\" /> Wysuwaj</label><br>"
    "<label><input type=\"radio\" name=\"actuator_2\" onclick=\"toggleActuator(2, 'retract')\" /> Chowaj</label><br>"
    "<label><input type=\"radio\" name=\"actuator_2\" onclick=\"toggleActuator(2, 'stop')\" checked /> Stop</label><br>"
    "<h3>Silownik 3</h3>"
    "<label><input type=\"radio\" name=\"actuator_3\" onclick=\"toggleActuator(3, 'extend')\" /> Wysuwaj</label><br>"
    "<label><input type=\"radio\" name=\"actuator_3\" onclick=\"toggleActuator(3, 'retract')\" /> Chowaj</label><br>"
    "<label><input type=\"radio\" name=\"actuator_3\" onclick=\"toggleActuator(3, 'stop')\" checked /> Stop</label><br>"
    "<h3>Silownik 4</h3>"
    "<label><input type=\"radio\" name=\"actuator_4\" onclick=\"toggleActuator(4, 'extend')\" /> Wysuwaj</label><br>"
    "<label><input type=\"radio\" name=\"actuator_4\" onclick=\"toggleActuator(4, 'retract')\" /> Chowaj</label><br>"
    "<label><input type=\"radio\" name=\"actuator_4\" onclick=\"toggleActuator(4, 'stop')\" checked /> Stop</label><br>"
    "<script>"
    "function toggleActuator(id, action) {"
    "    fetch('/' + action + '_' + id);"
    "}"
    "function fetchSensorData() {"
    "    fetch('/sensor').then(resp => resp.json()).then(data => {"
    "        document.getElementById('sensorData').innerHTML = "
    "`Temperatura: ${data.temperature} C<br>Wilgotnosc: ${data.humidity} %`;"
    "    });"
    "}"
    "setInterval(fetchSensorData, 1000);"
    "fetchSensorData();"
    "</script></body></html>";

static const char *TAG = "wifi softAP";

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d", MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d", MAC2STR(event->mac), event->aid, event->reason);
    }
}

void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .ssid_len = strlen(CONFIG_ESP_WIFI_SSID),
            .channel = CONFIG_ESP_WIFI_CHANNEL,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .max_connection = CONFIG_ESP_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {.required = true,},
        },
    };
    if (strlen(CONFIG_ESP_WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
        CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD, CONFIG_ESP_WIFI_CHANNEL);
}

void pwm_init(void) {
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

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

void hdc1080_read(float *temperature, float *humidity) {
    uint8_t data[4];
    uint8_t reg = HDC1080_TEMP_REG;

    i2c_master_write_to_device(I2C_MASTER_NUM, HDC1080_ADDR, &reg, 1, 100 / portTICK_PERIOD_MS);
    vTaskDelay(pdMS_TO_TICKS(20));

    i2c_master_read_from_device(I2C_MASTER_NUM, HDC1080_ADDR, data, 4, 100 / portTICK_PERIOD_MS);

    uint16_t raw_temp = (data[0] << 8) | data[1];
    uint16_t raw_humi = (data[2] << 8) | data[3];

    *temperature = ((float)raw_temp / 65536.0f) * 165.0f - 40.0f;
    *humidity = ((float)raw_humi / 65536.0f) * 100.0f;
}

void hdc1080_task(void *pvParameters) {
    float temp, hum;
    while (1) {
        hdc1080_read(&temp, &hum);
        latest_temp = temp;
        latest_hum = hum;
        ESP_LOGI("HDC1080", "Temperatura: %.2f C, Wilgotność: %.2f %%", temp, hum);
        vTaskDelay(pdMS_TO_TICKS(980));
    }
}


esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t sensor_get_handler(httpd_req_t *req) {
    char resp[100];
    snprintf(resp, sizeof(resp), "{\"temperature\":%.2f,\"humidity\":%.2f}", latest_temp, latest_hum);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t extend_get_handler(httpd_req_t *req) {
    int actuator_id = req->uri[8] - '0';
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2, PWM_DUTY);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2, 0);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2);
    httpd_resp_send(req, "Siłownik wysunięty", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t retract_get_handler(httpd_req_t *req) {
    int actuator_id = req->uri[strlen(req->uri) - 1] - '0';
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2, 0);
    ledc_set_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2, PWM_DUTY);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN1 + (actuator_id - 1) * 2);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL_IN2 + (actuator_id - 1) * 2);
    httpd_resp_send(req, "Siłownik chowany", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t stop_get_handler(httpd_req_t *req) {
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

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    pwm_init();
    i2c_master_init();
    start_webserver();

    xTaskCreate(hdc1080_task, "hdc1080_task", 4096, NULL, 5, NULL);
}


