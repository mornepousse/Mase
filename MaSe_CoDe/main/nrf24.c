#include "nrf24.h"
#include <string.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_heap_caps.h"

static const char *TAG = "NRF24";
static spi_device_handle_t nrf_spi;

void nrf24_init(void)
{
    esp_err_t ret;

    // Configure CE Pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << NRF_CE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(NRF_CE_GPIO, 0);

    // Configure CSN Pin
    io_conf.pin_bit_mask = (1ULL << NRF_CSN_GPIO);
    gpio_config(&io_conf);
    gpio_set_level(NRF_CSN_GPIO, 1);

    // Add device to SPI bus (Assuming SPI2_HOST is initialized)
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4000000, // 4 MHz
        .mode = 0,                 // NRF24L01 uses SPI Mode 0
        .spics_io_num = -1,        // Manual CS control
        .queue_size = 1,
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &nrf_spi);
    ESP_ERROR_CHECK(ret);

    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for NRF24 power up

    // Basic NRF24L01 Configuration
    // Enable CRC (2 bytes) to match standard libraries (Bit 3 EN_CRC | Bit 2 CRCO)
    nrf24_write_register(NRF_CONFIG, 0x0C); // Power Down, CRC 2 bytes
    nrf24_write_register(NRF_EN_AA, 0x00);  // Disable Auto Ack (for simple streaming)
    nrf24_write_register(NRF_EN_RXADDR, 0x01); // Enable Pipe 0
    nrf24_write_register(NRF_SETUP_AW, 0x03);  // 5 Bytes Address
    nrf24_write_register(NRF_SETUP_RETR, 0x00); // No Retransmission
    
    // Check if module is connected
    nrf24_write_register(NRF_RF_CH, 0x55);
    uint8_t check = nrf24_read_register(NRF_RF_CH);
    if (check != 0x55) {
        ESP_LOGE(TAG, "NRF24L01 NOT DETECTED! Read: 0x%02X. Check wiring (MISO/MOSI/CSN).", check);
    } else {
        ESP_LOGI(TAG, "NRF24L01 Detected successfully.");
    }

    nrf24_write_register(NRF_RF_CH, 76);       // Channel 76
    nrf24_write_register(NRF_RF_SETUP, 0x06);  // 1Mbps, 0dBm
    
    ESP_LOGI(TAG, "NRF24L01 Initialized");
}

static void cs_select(void) {
    gpio_set_level(NRF_CSN_GPIO, 0);
}

static void cs_deselect(void) {
    gpio_set_level(NRF_CSN_GPIO, 1);
}

static void ce_enable(void) {
    gpio_set_level(NRF_CE_GPIO, 1);
}

static void ce_disable(void) {
    gpio_set_level(NRF_CE_GPIO, 0);
}

void nrf24_write_register(uint8_t reg, uint8_t value)
{
    cs_select();
    uint8_t tx[2] = {NRF_W_REGISTER | (reg & 0x1F), value};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
    };
    spi_device_transmit(nrf_spi, &t);
    cs_deselect();
}

uint8_t nrf24_read_register(uint8_t reg)
{
    cs_select();
    // FIX: tx buffer must be 2 bytes to match length=16
    uint8_t tx[2] = {NRF_R_REGISTER | (reg & 0x1F), 0xFF};
    uint8_t rx[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_transmit(nrf_spi, &t);
    cs_deselect();
    return rx[1];
}

// FIX: Send command and data in a single SPI transaction (critical for NRF24!)
void nrf24_write_register_multi(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t *tx_buf = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    if (!tx_buf) {
        ESP_LOGE(TAG, "Failed to allocate tx_buf");
        return;
    }
    
    tx_buf[0] = NRF_W_REGISTER | (reg & 0x1F);
    memcpy(&tx_buf[1], data, len);
    
    cs_select();
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx_buf,
    };
    spi_device_transmit(nrf_spi, &t);
    cs_deselect();
    
    free(tx_buf);
}

void nrf24_set_tx_addr(uint8_t *addr, uint8_t len)
{
    nrf24_write_register_multi(NRF_TX_ADDR, addr, len);
    nrf24_write_register_multi(NRF_RX_ADDR_P0, addr, len); // Auto-ack requires P0 to match TX
    
    // Verify address was written correctly
    uint8_t read_addr[5] = {0};
    nrf24_read_register_multi(NRF_TX_ADDR, read_addr, 5);
    ESP_LOGI(TAG, "TX Addr set to: %02X:%02X:%02X:%02X:%02X", 
             read_addr[0], read_addr[1], read_addr[2], read_addr[3], read_addr[4]);
}

void nrf24_read_register_multi(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t *tx_buf = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    uint8_t *rx_buf = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    if (!tx_buf || !rx_buf) {
        ESP_LOGE(TAG, "Failed to allocate buffers");
        free(tx_buf);
        free(rx_buf);
        return;
    }
    
    memset(tx_buf, 0xFF, len + 1);
    tx_buf[0] = NRF_R_REGISTER | (reg & 0x1F);
    
    cs_select();
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    spi_device_transmit(nrf_spi, &t);
    cs_deselect();
    
    memcpy(data, &rx_buf[1], len);
    
    free(tx_buf);
    free(rx_buf);
}

void nrf24_send(uint8_t *data, uint8_t len)
{
    // Clear any pending interrupts (TX_DS, MAX_RT, RX_DR)
    nrf24_write_register(NRF_STATUS, 0x70);

    ce_disable();
    
    // FIX: Send command and payload in a single SPI transaction
    uint8_t *tx_buf = heap_caps_malloc(len + 1, MALLOC_CAP_DMA);
    if (!tx_buf) {
        ESP_LOGE(TAG, "Failed to allocate tx_buf for send");
        return;
    }
    
    tx_buf[0] = NRF_W_TX_PAYLOAD;
    memcpy(&tx_buf[1], data, len);
    
    cs_select();
    spi_transaction_t t = {
        .length = (len + 1) * 8,
        .tx_buffer = tx_buf,
    };
    spi_device_transmit(nrf_spi, &t);
    cs_deselect();
    
    free(tx_buf);

    ce_enable();
    esp_rom_delay_us(15); // Pulse CE for >10us
    ce_disable();
}

void nrf24_power_up_tx(void)
{
    uint8_t config = nrf24_read_register(NRF_CONFIG);
    config &= ~(NRF_CONFIG_PRIM_RX); // Clear PRIM_RX bit
    config |= NRF_CONFIG_PWR_UP;
    nrf24_write_register(NRF_CONFIG, config);
    esp_rom_delay_us(1500); // Start up delay
    
    ESP_LOGI(TAG, "Power Up TX - Config: 0x%02X", nrf24_read_register(NRF_CONFIG));
}

uint8_t nrf24_get_status(void)
{
    return nrf24_read_register(NRF_STATUS);
}
