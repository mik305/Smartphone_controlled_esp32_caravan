#include "hcsr04_sensor.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "HCSR04";

// KONFIGURACJA - TYLKO TUTAJ ZMIENIASZ AKTYWNY CZUJNIK (0-3)
#define ACTIVE_SENSOR_INDEX 1  // 0=pierwszy, 1=drugi, 2=trzeci, 3=czwarty

hcsr04_sensor_t hcsr04_sensors[NUM_HCSR04_SENSORS] = {
    {42, 1, HCSR04_INVALID_DISTANCE, 0},  // Czujnik 0 - trigger:42, echo:1
    {42, 2, HCSR04_INVALID_DISTANCE, 0},  // Czujnik 1 - trigger:42, echo:2
    {42, 8, HCSR04_INVALID_DISTANCE, 0},  // Czujnik 2 - trigger:42, echo:8
    {42, 9, HCSR04_INVALID_DISTANCE, 0}   // Czujnik 3 - trigger:42, echo:9
};

static uint8_t current_active_sensor = ACTIVE_SENSOR_INDEX;

static void send_trigger_pulse(void) {
    gpio_set_level(hcsr04_sensors[current_active_sensor].trigger_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level(hcsr04_sensors[current_active_sensor].trigger_pin, 0);
}

static float measure_distance(uint8_t echo_pin) {
    uint32_t start_time, end_time;
    uint32_t timeout = 0;

    // Czekaj na stan niski pinu echo
    while (gpio_get_level(echo_pin) && timeout++ < 1000) {
        esp_rom_delay_us(1);
    }

    // Wyślij impuls trigger
    send_trigger_pulse();

    // Czekaj na echo (stan wysoki)
    timeout = 0;
    while (!gpio_get_level(echo_pin)) {
        if (timeout++ >= HCSR04_TIMEOUT_US) {
            return HCSR04_INVALID_DISTANCE;
        }
        esp_rom_delay_us(1);
    }
    start_time = esp_timer_get_time();

    // Czekaj na koniec echa (stan niski)
    timeout = 0;
    while (gpio_get_level(echo_pin)) {
        if (timeout++ >= HCSR04_TIMEOUT_US) {
            return HCSR04_INVALID_DISTANCE;
        }
        esp_rom_delay_us(1);
    }
    end_time = esp_timer_get_time();

    // Oblicz odległość (cm)
    float distance = (end_time - start_time) * 0.0343 / 2;
    
    // Walidacja pomiaru
    if (distance < 2.0 || distance > 400.0) {
        return HCSR04_INVALID_DISTANCE;
    }

    return distance;
}

void hcsr04_sensor_init(void) {
    // Konfiguracja trigger pin (tylko dla aktywnego czujnika)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << hcsr04_sensors[current_active_sensor].trigger_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // Konfiguracja echo pin (tylko dla aktywnego czujnika)
    io_conf.pin_bit_mask = (1ULL << hcsr04_sensors[current_active_sensor].echo_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    ESP_LOGI(TAG, "HC-SR04 initialized. Active sensor: %d (Trigger:%d, Echo:%d)", 
             current_active_sensor,
             hcsr04_sensors[current_active_sensor].trigger_pin,
             hcsr04_sensors[current_active_sensor].echo_pin);
}

void hcsr04_set_active_sensor(uint8_t sensor_index) {
    if (sensor_index < NUM_HCSR04_SENSORS) {
        current_active_sensor = sensor_index;
        hcsr04_sensor_init(); // Rekonfiguruj piny dla nowego czujnika
        ESP_LOGI(TAG, "Active sensor changed to: %d", sensor_index);
    } else {
        ESP_LOGE(TAG, "Invalid sensor index: %d", sensor_index);
    }
}

/* ---------- w hcsr04_task() ------------------------------------------ */
void hcsr04_task(void *pv)
{
    /* --- jednorazowa konfiguracja wspólnych pinów trigger --- */
    for (int i = 0; i < NUM_HCSR04_SENSORS; i++) {
        gpio_config_t trig = {
            .pin_bit_mask = 1ULL << hcsr04_sensors[i].trigger_pin,
            .mode         = GPIO_MODE_OUTPUT,
            .intr_type    = GPIO_INTR_DISABLE
        };
        gpio_config(&trig);

        gpio_config_t ech  = {
            .pin_bit_mask = 1ULL << hcsr04_sensors[i].echo_pin,
            .mode         = GPIO_MODE_INPUT,
            .intr_type    = GPIO_INTR_DISABLE
        };
        gpio_config(&ech);
    }

    while (true)
    {
        for (int i = 0; i < NUM_HCSR04_SENSORS; i++) {

            /* 1) wyślij impuls trigger tylko na pin tego czujnika */
            gpio_set_level(hcsr04_sensors[i].trigger_pin, 1);
            esp_rom_delay_us(10);
            gpio_set_level(hcsr04_sensors[i].trigger_pin, 0);

            /* 2) zmierz czas echa */
            float d = measure_distance(hcsr04_sensors[i].echo_pin);
            if (d > 0) {

                hcsr04_sensor_t *s = &hcsr04_sensors[i];
                s->distance_cm = d;

                /* EMA */
                if (s->filt_cm == HCSR04_INVALID_DISTANCE)
                    s->filt_cm = d;                   /* pierwsza próbka           */
                else
                    s->filt_cm = HCSR04_ALPHA * d +   /* nowa                       */
                                 (1.0f - HCSR04_ALPHA) * s->filt_cm; /* stara */

                s->last_update = esp_timer_get_time();

                ESP_LOGI("HCSR04",
                         "Ch%d raw %.1f cm  |  filt %.1f cm",
                         i+1, d, s->filt_cm);
            }
            vTaskDelay(pdMS_TO_TICKS(60));  /*  ±60 ms przerwy między czujnikami */
        }

        /* odświeżenie całego cyklu co ok. 4 × 60 ms = 240 ms */
    }
}
