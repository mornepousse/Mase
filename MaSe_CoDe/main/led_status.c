#include "led_status.h"
#include "led_strip.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define BLINK_GPIO 48 // Adjust if needed (48 for DevKitC-1, 38 for DevKitM-1)

static led_strip_handle_t led_strip;
volatile connection_mode_t current_mode = MODE_USB;

static void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number */
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LEDs off to clear all pixels */
    led_strip_clear(led_strip);
}

void led_task(void *pvParameters)
{
    configure_led();
    int brightness = 0;
    int delta = 1;
    while (1) {
        // Pulse logic
        brightness += delta;
        if (brightness >= 20) delta = -1; // Max brightness 20 (out of 255) is enough
        if (brightness <= 0) {
             delta = 1;
             brightness = 0;
        }

        switch (current_mode) {
            case MODE_USB:
                led_strip_set_pixel(led_strip, 0, 0, brightness, 0); // Green
                break;
            case MODE_BT:
                led_strip_set_pixel(led_strip, 0, 0, 0, brightness); // Blue
                break;
            case MODE_NRF24:
                led_strip_set_pixel(led_strip, 0, brightness, brightness, 0); // Yellow (Red + Green)
                break;
        }
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(40)); // Pulse speed
    }
}
