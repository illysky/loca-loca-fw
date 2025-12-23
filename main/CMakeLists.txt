idf_component_register(SRCS    "main.c" 
                               "wifi.c"
                               "http.c"
                               "tmc2209.c" 
                               "stepper.c"
                               "as5600.c"
                               "lock_state.c"
                       INCLUDE_DIRS    "."
                       PRIV_REQUIRES   esp_wifi
                                       esp_http_server
                                       mdns
                                       driver 
                                       esp_driver_gpio 
                                       esp_driver_uart 
                                       esp_driver_gptimer
                                       esp_driver_i2c
                                       nvs_flash)
