idf_component_register(SRCS "cpu_monitor.c" "nrf24.c" "main.c" "pmw3389.c" "led_status.c"
                       INCLUDE_DIRS "." 
                       PRIV_REQUIRES esp_timer esp_driver_gpio esp_hid led_strip)
