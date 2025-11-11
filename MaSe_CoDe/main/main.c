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
#include "srom_pmw3389.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "class/hid/hid_device.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
// #include "esp_rom"

static const char *TAG = "Main";

#define PIN_NUM_MISO GPIO_NUM_7
#define PIN_NUM_MOSI GPIO_NUM_6
#define PIN_NUM_CLK GPIO_NUM_5
#define PIN_NUM_CS GPIO_NUM_15
#define PIN_NUM_MOTION GPIO_NUM_4

static spi_device_handle_t spi;
// délai entre adresse et donnée lors d'une lecture (tSRAD) — ~35 µs recommandé
#define PMW_TSRAD_US 35
// registres selon la datasheet
#define REG_DELTA_X_L 0x03
#define REG_DELTA_X_H 0x04
#define REG_DELTA_Y_L 0x05
#define REG_DELTA_Y_H 0x06

bool inBurst = false;  // in busrt mode
bool reportSQ = false; // report surface quality
int8_t dx, dy;
unsigned long lastTS;
unsigned long lastButtonCheck = 0;
unsigned long curTime;

// =============================
// Fonctions bas niveau
// =============================
static void pmw3389_write(uint8_t reg, uint8_t val)
{
    gpio_set_level(PIN_NUM_CS, 0);
    uint8_t tx[2] = {(uint8_t)(reg | 0x80), val};
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx;

    spi_device_transmit(spi, &t);

    esp_rom_delay_us(20);
    gpio_set_level(PIN_NUM_CS, 1);

    // small delay after write (datasheet impose des délais entre commandes)
    esp_rom_delay_us(100);
}

static uint8_t pmw3389_read(uint8_t reg)
{
    gpio_set_level(PIN_NUM_CS, 0);

    uint8_t addr = reg & 0x7F; // MSB = 0 -> read
    uint8_t rx = 0;
    spi_transaction_t t;

    // send address (8 bits)
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &addr;

    esp_rom_delay_us(1); // small settling
    spi_device_transmit(spi, &t);

    // wait tSRAD for sensor to prepare data
    esp_rom_delay_us(PMW_TSRAD_US);

    // read data (8 bits) - send dummy byte while clocking in data
    uint8_t dummy = 0x00;
    uint8_t buf = 0;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &dummy;
    t.rx_buffer = &buf;
    spi_device_transmit(spi, &t);
    esp_rom_delay_us(1);
    gpio_set_level(PIN_NUM_CS, 1);
    // small post-read delay (datasheet has tSRR/tSWR constraints)
    esp_rom_delay_us(19);

    return buf;
}
static uint8_t pmw3389_read_burst(uint8_t *buffer, size_t length)
{
    gpio_set_level(PIN_NUM_CS, 0);

    uint8_t addr = MOTION_BURST & 0x7F; // MSB = 0 -> read
    spi_transaction_t t;

    // send address (8 bits)
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &addr;

    esp_rom_delay_us(1); // small settling
    spi_device_transmit(spi, &t);

    // wait tSRAD for sensor to prepare data
    esp_rom_delay_us(PMW_TSRAD_US);

    // read burst data (12 bytes)
    memset(&t, 0, sizeof(t));
    t.length = length * 8;
    t.tx_buffer = NULL;
    t.rx_buffer = buffer;
    spi_device_transmit(spi, &t);

    esp_rom_delay_us(1);
    gpio_set_level(PIN_NUM_CS, 1);
    // small post-read delay (datasheet has tSRR/tSWR constraints)
    esp_rom_delay_us(19);

    return 0;
}
static uint8_t pmw3389_read_old(uint8_t reg)
{
    gpio_set_level(PIN_NUM_CS, 0);
    uint8_t addr = reg & 0x7F; // MSB = 0 -> read
    uint8_t rx = 0;
    spi_transaction_t t;

    // send address (8 bits)
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &addr;

    esp_rom_delay_us(1); // small settling
    spi_device_transmit(spi, &t);

    // wait tSRAD for sensor to prepare data
    esp_rom_delay_us(PMW_TSRAD_US);

    // read data (8 bits) - send dummy byte while clocking in data
    uint8_t dummy = 0x00;
    uint8_t buf = 0;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &dummy;
    t.rx_buffer = &buf;
    spi_device_transmit(spi, &t);
    esp_rom_delay_us(1);
    gpio_set_level(PIN_NUM_CS, 1);
    // small post-read delay (datasheet has tSRR/tSWR constraints)
    esp_rom_delay_us(19);

    return buf;
}
static int16_t pmw3389_read_delta(uint8_t reg_low, uint8_t reg_high)
{
    uint8_t low = pmw3389_read(reg_low);
    uint8_t high = pmw3389_read(reg_high);
    int16_t v = (int16_t)((high << 8) | low);
    return v;
}

static void pmw3389_upload_srom(void)
{
    ESP_LOGI(TAG, "Uploading SROM firmware...");

    // Step 1: Enter SROM download mode
    pmw3389_write(0x3A, 0x5A); // Power up reset
    vTaskDelay(pdMS_TO_TICKS(50));
    pmw3389_write(CONFIG_2, 0x00);
    pmw3389_write(SROM_ENABLE, 0x1D);
    esp_rom_delay_us(10);
    pmw3389_write(SROM_ENABLE, 0x18);
    // Step 2: Write SROM via burst
    gpio_set_level(PIN_NUM_CS, 0);
    uint8_t addr = SROM_LOAD_BURST | 0x80; // SROM_Load_Burst (write)
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &addr,
    };
    spi_device_transmit(spi, &t);

    esp_rom_delay_us(15);

    for (int i = 0; i < srom_length; i++)
    {
        spi_transaction_t t_fw = {
            .length = 8,
            .tx_buffer = &srom_firmware[i],
        };
        spi_device_transmit(spi, &t_fw);
        esp_rom_delay_us(15);
    }
    pmw3389_write(CONFIG_2, 0x00);
    gpio_set_level(PIN_NUM_CS, 1);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Step 3: Check SROM ID
    uint8_t srom_id = pmw3389_read(SROM_ID);
    ESP_LOGI(TAG, "SROM ID: 0x%02X", srom_id);
}

// =============================
// Init SPI + capteur
// =============================
static void spi_init(void)
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4000000, // 4 MHz
        .mode = 3,                 // PMW3389 = SPI mode 3
        .spics_io_num = -1,        // on gère CS manuellement
        .queue_size = 1,
    };

    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_NUM_CS, 1);
}

static void pmw3389_selftest(void)
{
    ESP_LOGI(TAG, "=== PMW3389 Self-Test ===");

    // Reset capteur
    pmw3389_write(POWER_UP_RESET, 0x5A); // Power-up reset
    vTaskDelay(pdMS_TO_TICKS(50));

    // Lecture Product ID
    uint8_t pid = pmw3389_read(PRODUCT_ID);
    ESP_LOGI(TAG, "Product ID: 0x%02X", pid);
    vTaskDelay(pdMS_TO_TICKS(50));
    // Charger le SROM
    pmw3389_upload_srom();

    vTaskDelay(pdMS_TO_TICKS(50));
    // Vérifier le SROM ID
    uint8_t srom_id = pmw3389_read(SROM_ID);
    ESP_LOGI(TAG, "SROM ID: 0x%02X", srom_id);

    // Lire le motion register
    uint8_t motion = pmw3389_read(MOTION);
    if (motion & 0x80)
    {
        int dx = (int8_t)pmw3389_read(DELTA_X_L);
        int dy = (int8_t)pmw3389_read(DELTA_Y_L);
        ESP_LOGI(TAG, "Mouvement détecté: dx=%d dy=%d", dx, dy);
    }
    else
    {
        ESP_LOGI(TAG, "Pas de mouvement détecté");
    }
}

#define EPNUM_CDC_NOTIF 1
#define EPNUM_CDC_IN 2
#define EPNUM_CDC_OUT 2
#define EPNUM_VENDOR_IN 5
#define EPNUM_VENDOR_OUT 5
#define EPNUM_KEYBOARD 4
#define EPNUM_MOUSE 6

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

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
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
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

void app_main(void)
{
    ESP_LOGI(TAG, "USB initialization");
    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(); 
    tusb_cfg.descriptor.full_speed_config = hid_configuration_descriptor;
    tusb_cfg.descriptor.string = hid_string_descriptor;
    tusb_cfg.descriptor.string_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]);

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    spi_init();

    pmw3389_selftest();
    // Ensuite tu peux garder une boucle de lecture continue
    app_send_hid_demo();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        if (tud_mounted())
        {
            uint8_t burstBuffer[12];
            curTime = esp_timer_get_time();
            unsigned long elapsed = curTime - lastTS;
            // ESP_LOGI(TAG, "Elapsed since last burst: %lu ms", elapsed);
            if (!inBurst)
            {
                pmw3389_write(MOTION_BURST, 0x00); // start burst mode
                lastTS = curTime;
                inBurst = true;
            }
            if (elapsed >= 1000)
            {
                pmw3389_read_burst(burstBuffer, 12);
                int motion = (burstBuffer[0] & 0x80) > 0;
                int surface = (burstBuffer[0] & 0x08) > 0; // 0 if on surface / 1 if off surface

                int xl = burstBuffer[2];
                int xh = burstBuffer[3];
                int yl = burstBuffer[4];
                int yh = burstBuffer[5];

                int squal = burstBuffer[6];

                int x = xh << 8 | xl;
                int y = yh << 8 | yl;

                dx -= y;
                dy -= x;
                // ESP_LOGI(TAG, "Burst read: motion=%d surface=%d dx=%d dy=%d squal=%d", motion, surface, dx, dy, squal);

                if (motion)
                {
                    // signed char mdx = constrain(dx, -127, 127);
                    // signed char mdy = constrain(dy, -127, 127);

                    // Mouse.move(mdx, mdy, 0);
                    // mouse_draw_square_next_delta(&dx, &dy);
                    mouse_draw_square_next_delta(&dx, &dy);
                    tud_hid_mouse_report(HID_ITF_PROTOCOL_MOUSE, 0x00, dx, dy, 0, 0);
                    // ESP_LOGI(TAG, "Mouse move: dx=%d dy=%d", dx, dy);
                    //  ESP_LOGI(TAG, "Mouse move: dx=%d dy=%d", mdx, mdy);
                    dx = 0;
                    dy = 0;
                }

                lastTS = curTime;
            }
        }
    }
}
