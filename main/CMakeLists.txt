idf_component_register(SRCS "vl6180x_sensor.c" "main.c" 
                      "wifi_ap.c" 
                      "http_server.c" 
                      "actuator_control.c" 
                      "hdc1080_sensor.c"
                      "lsm6dsox_sensor.c"
                      "hcsr04_sensor.c"
                      INCLUDE_DIRS ".")

set(web_dir "${CMAKE_CURRENT_SOURCE_DIR}/../web")
file(WRITE "${CMAKE_BINARY_DIR}/web/index.html.txt" "")
file(COPY "${web_dir}/index.html" DESTINATION "${CMAKE_BINARY_DIR}/web")
target_add_binary_data(${COMPONENT_LIB} "${web_dir}/index.html" TEXT)