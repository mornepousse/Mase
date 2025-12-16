#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

typedef enum {
    MODE_USB,
    MODE_BT,
    MODE_NRF24
} connection_mode_t;

extern volatile connection_mode_t current_mode;

void led_task(void *pvParameters);
