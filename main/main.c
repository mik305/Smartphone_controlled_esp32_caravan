#include "wifi_ap.h"
#include "http_server.h"
#include "actuator_control.h"
#include "hdc1080_sensor.h"
#include "nvs_flash.h"
#include "lsm6dsox_sensor.h"
#include "hcsr04_sensor.h"

#include "bmi323.h"   
//extern struct bmi3_dev bmi323_dev;
#include "bmi323_port.h" 

static void bmi323_init_api(void)
{
    /* init struct */
    bmi323_dev.intf           = BMI3_I2C_INTF;
    bmi323_dev.intf_ptr       = NULL;
    bmi323_dev.read           = bmi3_i2c_read;
    bmi323_dev.write          = bmi3_i2c_write;
    bmi323_dev.delay_us       = bmi3_delay_us;
    bmi323_dev.read_write_len = 32;

    bmi3_init(&bmi323_dev);

    /* włącz sam akcelerometr 100 Hz / ±8 g */
    struct bmi3_sens_config cfg = {
        .type = BMI3_ACCEL,
        .cfg.acc.odr   = BMI3_ACC_ODR_100HZ,
        .cfg.acc.range = BMI3_ACC_RANGE_8G
    };
    bmi3_set_sensor_config(&cfg, 1, &bmi323_dev);
   // bmi3_sensor_enable(BMI3_ACCEL, BMI3_ENABLE, &bmi323_dev);
   cfg.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;
}

/* prosta pętla odczytu */
static void bmi323_task(void *pv)
{
    struct bmi3_sensor_data d = { .type = BMI3_ACCEL };

    while (1) {
        bmi3_get_sensor_data(&d, 1, &bmi323_dev);
        
        printf("ax=%d ay=%d az=%d LSB\n",
       d.sens_data.acc.x,
       d.sens_data.acc.y,
       d.sens_data.acc.z);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_softap();
    actuator_pwm_init();
    hcsr04_sensor_init();
    hdc1080_sensor_init();  // Inicjalizacja I2C tutaj
    lsm6dsox_sensor_init(); // Używa tej samej magistrali I2C
    bmi323_init_api();                        // <‑‑ nowy init
    xTaskCreate(bmi323_task, "bmi323_task",
                4096, NULL, 5, NULL);
    //bmi323_init();
    

   start_webserver();


    xTaskCreate(hcsr04_task, "hcsr04_task", 4096, NULL, 5, NULL);
    xTaskCreate(hdc1080_task, "hdc1080_task", 4096, NULL, 5, NULL);
    xTaskCreate(lsm6dsox_task, "lsm6dsox_task", 4096, NULL, 5, NULL);
   // xTaskCreate(bmi323_task, "bmi323_task", 4096, NULL, 5, NULL);

}










