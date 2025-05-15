idf_component_register(
			SRCS 
			#"common.c"
			 "main.c" 
                      "wifi_ap.c" 
                      "http_server.c" 
                      "actuator_control.c" 
                      "hdc1080_sensor.c"
                      "lsm6dsox_sensor.c"
                      "hcsr04_sensor.c"
                      "bmi323_sensor.c"
			INCLUDE_DIRS "."
             REQUIRES
        bmi323_api      # <-- nasz nowy komponent
        driver          # i2c_master_*, ledc_*
        esp_http_server 
	esp_wifi 
	esp_netif
	nvs_flash
esp_timer
                              )

set(web_dir "${CMAKE_CURRENT_SOURCE_DIR}/../web")
file(WRITE "${CMAKE_BINARY_DIR}/web/index.html.txt" "")
file(COPY "${web_dir}/index.html" DESTINATION "${CMAKE_BINARY_DIR}/web")
target_add_binary_data(${COMPONENT_LIB} "${web_dir}/index.html" TEXT)
