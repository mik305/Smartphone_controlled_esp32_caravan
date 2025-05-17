#include "hx711.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "HX711";
volatile float  hx_grams[4] = {0};  
/*  pd_sck ↗ negedge → DATA wyrzuca kolejne bity (MSB first) */
static inline void clk_pulse(gpio_num_t clk)
{
    gpio_set_level(clk, 1);
    esp_rom_delay_us(1);
    gpio_set_level(clk, 0);
    esp_rom_delay_us(1);
}

/* ---------- API ------------------------------------------------------ */
esp_err_t hx711_init(hx711_t *dev)
{
    /* CLK  → wyjście push-pull */
    gpio_config_t clk_io = {
        .pin_bit_mask = 1ULL << dev->clk,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&clk_io);
    gpio_set_level(dev->clk, 0);

    /* DATA → wejście; HX711 sam trzyma linie w stanie HIGH */
    gpio_config_t dat_io = {
        .pin_bit_mask = 1ULL << dev->data,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    ret |= gpio_config(&dat_io);

    /* ustalenie wzmocnienia – trzy puste odczyty */
    for (int i = 0; i < 3; i++) {
        int32_t dummy;
        hx711_read_raw(dev, &dummy);
    }

    ESP_LOGI(TAG, "HX711 init OK  CLK=%d  DATA=%d", dev->clk, dev->data);
    return ret;
}

esp_err_t hx711_is_ready(hx711_t *dev, TickType_t tout)
{
   int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(dev->data)) {
        if ((esp_timer_get_time() - t0) > tout * 1000) return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t hx711_read_raw(hx711_t *dev, int32_t *out)
{
    if (hx711_is_ready(dev, 100) != ESP_OK) return ESP_ERR_TIMEOUT;

    int32_t val = 0;
    for (int i=0;i<24;i++) {
        clk_pulse(dev->clk);
        val = (val << 1) | gpio_get_level(dev->data);
    }
    /* 25-27 pulsów = wybór gain */
    for (int i=0;i<dev->gain;i++) clk_pulse(dev->clk);

    /* sign extend 24->32 bit */
    if (val & 0x800000) val |= 0xFF000000;
    *out = val;
    return ESP_OK;
}

esp_err_t hx711_set_gain(hx711_t *dev, hx711_gain_t g)
{
    dev->gain = g;
    return hx711_read_raw(dev, &(int32_t){0});   /* dummy read by spec */
}
