/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "pmw3389.h"
#include "led_status.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
// #include "esp_rom"

static const char *TAG = "Main";



#define PIN_LMB 21
#define PIN_RMB 37
#define PIN_CENTER_FUNCT_1_CLICK 36
#define PIN_CENTER_FUNCT_2_CLICK 44
//#define PIN_RIGHT_CLICK 37
#define PIN_RIGHT_FUNCT_CLICK 35
//#define PIN_LEFT_CLICK 21
#define PIN_LEFT_FUNCT_CLICK 45
#define PIN_MIDDLE_CLICK 38

#define PIN_ENCODER_A 2
#define PIN_ENCODER_B 42
#define PIN_ENCODER_C 41

static void encoder_init(void)
{
    gpio_config_t io_conf = {};
    // A and B as inputs with pull-up
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_ENCODER_A) | (1ULL << PIN_ENCODER_B);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // C as output low (Common Ground)
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_ENCODER_C);
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(PIN_ENCODER_C, 0);
}

static int8_t encoder_read_scroll(void)
{
    static const int8_t table[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
    static uint8_t last_ab = 0;
    static int8_t accum = 0;
    
    uint8_t curr_ab = (gpio_get_level(PIN_ENCODER_A) << 1) | gpio_get_level(PIN_ENCODER_B);
    uint8_t idx = (last_ab << 2) | curr_ab;
    last_ab = curr_ab;
    
    accum += table[idx];
    
    // Assuming 2 steps per detent. Adjust if needed (e.g. 4 or 1).
    if (accum >= 2) {
        accum -= 2;
        return -1;
    } else if (accum <= -2) {
        accum += 2;
        return 1;
    }
    return 0;
}

static void buttons_init(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_LMB) | (1ULL << PIN_RMB) | (1ULL << PIN_MIDDLE_CLICK) |
                           (1ULL << PIN_CENTER_FUNCT_1_CLICK) | (1ULL << PIN_CENTER_FUNCT_2_CLICK)   
                           | (1ULL << PIN_RIGHT_FUNCT_CLICK) | (1ULL << PIN_LEFT_FUNCT_CLICK);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

static uint8_t buttons_read(void)
{
    uint8_t buttons = 0;
    if (gpio_get_level(PIN_LMB) == 0) buttons |= TU_BIT(0);
    if (gpio_get_level(PIN_RMB) == 0) buttons |= TU_BIT(1);
    if (gpio_get_level(PIN_MIDDLE_CLICK) == 0) buttons |= TU_BIT(2);
    //if (gpio_get_level(PIN_LEFT_CLICK) == 0) buttons |= TU_BIT(3);
    //.if (gpio_get_level(PIN_RIGHT_CLICK) == 0) buttons |= TU_BIT(4);
    return buttons;
}












#define EPNUM_CDC_NOTIF 1
#define EPNUM_CDC_IN 2
#define EPNUM_CDC_OUT 2
#define EPNUM_VENDOR_IN 5
#define EPNUM_VENDOR_OUT 5
#define EPNUM_KEYBOARD 4
#define EPNUM_MOUSE 6

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + 1 * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE))};

/**
 * @brief String descriptor
 */
const char *hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},    // 0: is supported language is English (0x0409)
    "MornePousse",               // 1: Manufacturer
    "MaSe",        // 2: Product
    "123256",                // 3: Serials, should use chip ID
    "Mouse", // 4: HID
};

static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 1),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
}

/********* Application ***************/

/*
typedef enum
{
    MOUSE_DIR_RIGHT,
    MOUSE_DIR_DOWN,
    MOUSE_DIR_LEFT,
    MOUSE_DIR_UP,
    MOUSE_DIR_MAX,
} mouse_dir_t;

#define DISTANCE_MAX 125
#define DELTA_SCALAR 5
*/

/*
static void mouse_draw_square_next_delta(int8_t *delta_x_ret, int8_t *delta_y_ret)
{
    static mouse_dir_t cur_dir = MOUSE_DIR_RIGHT;
    static uint32_t distance = 0;

    // Calculate next delta
    if (cur_dir == MOUSE_DIR_RIGHT)
    {
        *delta_x_ret = DELTA_SCALAR;
        *delta_y_ret = 0;
    }
    else if (cur_dir == MOUSE_DIR_DOWN)
    {
        *delta_x_ret = 0;
        *delta_y_ret = DELTA_SCALAR;
    }
    else if (cur_dir == MOUSE_DIR_LEFT)
    {
        *delta_x_ret = -DELTA_SCALAR;
        *delta_y_ret = 0;
    }
    else if (cur_dir == MOUSE_DIR_UP)
    {
        *delta_x_ret = 0;
        *delta_y_ret = -DELTA_SCALAR;
    }

    // Update cumulative distance for current direction
    distance += DELTA_SCALAR;
    // Check if we need to change direction
    if (distance >= DISTANCE_MAX)
    {
        distance = 0;
        cur_dir++;
        if (cur_dir == MOUSE_DIR_MAX)
        {
            cur_dir = 0;
        }
    }
}
*/

/*
static void app_send_hid_demo(void)
{
    // Keyboard output: Send key 'a/A' pressed and released
    ESP_LOGI(TAG, "Sending Keyboard report");
    uint8_t keycode[6] = {HID_KEY_A};
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, keycode);
    vTaskDelay(pdMS_TO_TICKS(50));
    tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);

    // Mouse output: Move mouse cursor in square trajectory
    ESP_LOGI(TAG, "Sending Mouse report");
    int8_t delta_x;
    int8_t delta_y;
    for (int i = 0; i < (DISTANCE_MAX / DELTA_SCALAR) * 4; i++)
    {
        // Get the next x and y delta in the draw square pattern
        mouse_draw_square_next_delta(&delta_x, &delta_y);
        tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, delta_x, delta_y, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
*/



void app_main(void)
{
    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(); 
    tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
    tusb_cfg.descriptor.string = hid_string_descriptor;
    tusb_cfg.descriptor.string_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]);

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);

    pmw3389_init();

    buttons_init();
    encoder_init();

    pmw3389_selftest();

    // Ensuite tu peux garder une boucle de lecture continue
    // app_send_hid_demo();
    uint8_t last_buttons = 0;
    uint8_t last_dpi_btn = 1;
    uint8_t last_win_btn = 1;
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
        if (tud_mounted())
        {
            uint8_t burstBuffer[12];
            curTime = esp_timer_get_time();
            unsigned long elapsed = curTime - lastTS;
            // ESP_LOGI(TAG, "Elapsed since last burst: %lu ms", elapsed);
            if (!inBurst)
            {
                pmw3389_enable_burst_mode(); // start burst mode
                lastTS = curTime;
            }
            if (elapsed >= 1000)
            {
                pmw3389_read_burst(burstBuffer, 12);

                // DPI Button Check
                uint8_t dpi_btn = gpio_get_level(PIN_CENTER_FUNCT_2_CLICK);
                if (last_dpi_btn == 1 && dpi_btn == 0)
                {
                    pmw3389_cycle_dpi();
                }
                last_dpi_btn = dpi_btn;

                // Win Key Check
                uint8_t win_btn = gpio_get_level(PIN_RIGHT_FUNCT_CLICK);
                if (win_btn != last_win_btn)
                {
                    bool success = false;
                    if (win_btn == 0) {
                        success = tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, KEYBOARD_MODIFIER_LEFTGUI, NULL);
                    } else {
                        success = tud_hid_keyboard_report(HID_ITF_PROTOCOL_KEYBOARD, 0, NULL);
                    }
                    if (success) {
                        last_win_btn = win_btn;
                    }
                }

                int motion = (burstBuffer[0] & 0x80) > 0;
                // int surface = (burstBuffer[0] & 0x08) > 0; // 0 if on surface / 1 if off surface

                int xl = burstBuffer[2];
                int xh = burstBuffer[3];
                int yl = burstBuffer[4];
                int yh = burstBuffer[5];

                // int squal = burstBuffer[6];

                int16_t x = (int16_t)(xh << 8 | xl);
                int16_t y = (int16_t)(yh << 8 | yl);

                dx -= x;
                dy -= y;
                // ESP_LOGI(TAG, "Burst read: motion=%d surface=%d dx=%d dy=%d squal=%d", motion, surface, dx, dy, squal);

                uint8_t buttons = buttons_read();
                int8_t scroll = encoder_read_scroll();

                if (motion || buttons != last_buttons || scroll != 0)
                {
                    // signed char mdx = constrain(dx, -127, 127);
                    // signed char mdy = constrain(dy, -127, 127);

                    // Mouse.move(mdx, mdy, 0);
                    // mouse_draw_square_next_delta(&dx, &dy);
                    // mouse_draw_square_next_delta(&dx, &dy);
                    
                    int8_t report_x = (dx > 127) ? 127 : ((dx < -127) ? -127 : dx);
                    int8_t report_y = (dy > 127) ? 127 : ((dy < -127) ? -127 : dy);

                    if (tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, buttons, report_x, report_y, scroll, 0))
                    {
                        dx -= report_x;
                        dy -= report_y;
                        last_buttons = buttons;
                    }
                }

                lastTS = curTime;
            }
        }
    }
}
