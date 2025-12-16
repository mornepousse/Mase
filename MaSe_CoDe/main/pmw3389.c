#include "pmw3389.h"
#include "srom_pmw3389.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "PMW3389";

static spi_device_handle_t spi;

bool inBurst = false;
bool reportSQ = false;
int16_t dx, dy;
unsigned long lastTS;
unsigned long curTime;

static uint32_t current_dpi = 1600;
static const uint32_t dpi_levels[] = {400, 800, 1600, 3200, 6400};
static int dpi_idx = 2;

void pmw3389_write(uint8_t reg, uint8_t val)
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

    // small delay after write (datasheet imposes delays between commands)
    esp_rom_delay_us(100);
}

uint8_t pmw3389_read(uint8_t reg)
{
    gpio_set_level(PIN_NUM_CS, 0);

    uint8_t addr = reg & 0x7F; // MSB = 0 -> read
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

uint8_t pmw3389_read_burst(uint8_t *buffer, size_t length)
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

uint8_t pmw3389_read_old(uint8_t reg)
{
    gpio_set_level(PIN_NUM_CS, 0);
    uint8_t addr = reg & 0x7F; // MSB = 0 -> read
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

int16_t pmw3389_read_delta(uint8_t reg_low, uint8_t reg_high)
{
    uint8_t low = pmw3389_read(reg_low);
    uint8_t high = pmw3389_read(reg_high);
    int16_t v = (int16_t)((high << 8) | low);
    return v;
}

void pmw3389_set_dpi(uint32_t dpi)
{
    uint16_t cpi = dpi / 50;
    pmw3389_write(RESOLUTION_L, cpi & 0xFF);
    pmw3389_write(RESOLUTION_H, (cpi >> 8) & 0xFF);
    ESP_LOGI(TAG, "DPI set to %lu", dpi);
}

void pmw3389_cycle_dpi(void)
{
    dpi_idx = (dpi_idx + 1) % (sizeof(dpi_levels) / sizeof(dpi_levels[0]));
    current_dpi = dpi_levels[dpi_idx];
    pmw3389_set_dpi(current_dpi);
    inBurst = false; // Writing to registers terminates burst mode
}

void pmw3389_upload_srom(void)
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

void pmw3389_selftest(void)
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
    
    pmw3389_set_dpi(current_dpi);
}

void pmw3389_init(void)
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

void pmw3389_enable_burst_mode(void)
{
    pmw3389_write(MOTION_BURST, 0x00);
    inBurst = true;
}
