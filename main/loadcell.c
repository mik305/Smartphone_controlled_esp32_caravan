#include "hx711.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"

#define TAG "LOADCELL"
#define AUTO_TARE_SAMPLES   50
#define DELTA_THRESH_RAW   25000     /* dobierz patrząc na logi */
#define DELTA_HYST_RAW     5000      /* histereza ~15 % progu   */

/* ───────── mapping pinów ─────────
 *  Ustawić zgodnie ze schematem.
 *  CLK może być wspólny lub osobny – tu każdy ma własny.
 DAT1 - IO4
 DAT2 - IO15
 DAT3 - IO16
 DAT4 - IO17
 CLK HX - IO5
 */
static hx711_t cells[4] = {
    { .clk = GPIO_NUM_5,  .data = GPIO_NUM_4,  .gain = HX711_GAIN_128 },
    { .clk = GPIO_NUM_5,  .data = GPIO_NUM_15,  .gain = HX711_GAIN_128 },
    { .clk = GPIO_NUM_5,  .data = GPIO_NUM_16,  .gain = HX711_GAIN_128 },
    { .clk = GPIO_NUM_5,  .data = GPIO_NUM_17,  .gain = HX711_GAIN_128 },
};
static int32_t   zero_raw[4] = {0}; 
volatile bool hx_on_ground[4] = {false};
static void hx_autotare(void)
{
    for (int i = 0; i < 4; i++) zero_raw[i] = 0;

    for (int s = 0; s < AUTO_TARE_SAMPLES; s++) {
        for (int i = 0; i < 4; i++) {
            int32_t r;
            if (hx711_read_raw(&cells[i], &r) == ESP_OK)
                zero_raw[i] += r;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    for (int i = 0; i < 4; i++) {
        zero_raw[i] /= AUTO_TARE_SAMPLES;
        ESP_LOGI(TAG, "Ch%d zero_raw = %ld", i+1, (long)zero_raw[i]);
    }
}

/* progi i filtracja */
#define TOUCH_THRESHOLD_G   200.0f      /* poniżej 50 g uznajemy za „w powietrzu” */
#define TOUCH_HYST_G        30.0f      /* histereza, aby nie drgało przy progu   */

#define EMA_ALPHA           0.25f      /* 0…1 ;  im mniejsze, tym mocniej wygładza */
static float  hx_ema_g[4]  = {0};      /* przechowuje wygładzone wartości gramów  */

/* globalne wyniki (RAW) */
volatile int32_t hx_raw[4] = {0};
//int32_t temp;

esp_err_t hx_set_calibration(hx711_t *dev,
                             int32_t obciazony_output,
                             int32_t raw_output,
                             float   known_mass)
{
    if (raw_output == obciazony_output) return ESP_FAIL;

    dev->offset    = obciazony_output;
    dev->lsb_per_g = (float)(raw_output - obciazony_output) / known_mass;

    ESP_LOGI("HX", "Cal set offset=%ld  lsb/g=%.2f",
             (long)dev->offset, dev->lsb_per_g);
    return ESP_OK;
}


float hx_get_grams(hx711_t *dev)
{
    int32_t raw;
    if (hx711_read_raw(dev, &raw) != ESP_OK) return NAN;
    return (raw - dev->offset) / dev->lsb_per_g;
}

static void hx_task(void *pv)
{
    for (int i=0;i<4;i++) hx711_init(&cells[i]);

    hx_set_calibration(&cells[0], 636607, -744481, 750.0f);
    hx_set_calibration(&cells[1], 1299038,1287142  , 750.0f);
    hx_set_calibration(&cells[2], -250237, -139118, 750.0f);
    hx_set_calibration(&cells[3], 659941, 759968, 750.0f);
        hx_autotare();                 /* <---  DODAJ  */


    while (true) {
        for (int i = 0; i < 4; i++) {
            int32_t raw;
            if (hx711_read_raw(&cells[i], &raw) == ESP_OK) {

                int32_t delta = llabs(raw - zero_raw[i]);

                bool was = hx_on_ground[i];
                if (was) {
                    if (delta < (DELTA_THRESH_RAW - DELTA_HYST_RAW))
                        hx_on_ground[i] = false;      /* odrywa się */
                } else {
                    if (delta > (DELTA_THRESH_RAW + DELTA_HYST_RAW))
                        hx_on_ground[i] = true;       /* stoi */
                }

                ESP_LOGI(TAG, "Ch%d: raw=%ld  Δ=%ld  %s",
                         i + 1, (long)raw, (long)delta,
                         hx_on_ground[i] ? "[ON GROUND]" : "[IN AIR]");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));               /* 1 s */
    }

}
void hx_start_task(void)
{
    xTaskCreatePinnedToCore(hx_task, "hx711_task", 4096,
                            NULL, 5, NULL, tskNO_AFFINITY);
}

