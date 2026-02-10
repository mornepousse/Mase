#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"

// Configuration Pins
#define NRF_CE_GPIO     GPIO_NUM_3
#define NRF_CSN_GPIO    GPIO_NUM_11  // Selected safe pin, user can adjust

// NRF24L01 Registers
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_SETUP_RETR  0x04
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_OBSERVE_TX  0x08
#define NRF_CD          0x09
#define NRF_RX_ADDR_P0  0x0A
#define NRF_RX_ADDR_P1  0x0B
#define NRF_RX_ADDR_P2  0x0C
#define NRF_RX_ADDR_P3  0x0D
#define NRF_RX_ADDR_P4  0x0E
#define NRF_RX_ADDR_P5  0x0F
#define NRF_TX_ADDR     0x10
#define NRF_RX_PW_P0    0x11
#define NRF_RX_PW_P1    0x12
#define NRF_RX_PW_P2    0x13
#define NRF_RX_PW_P3    0x14
#define NRF_RX_PW_P4    0x15
#define NRF_RX_PW_P5    0x16
#define NRF_FIFO_STATUS 0x17

// Commands
#define NRF_R_REGISTER    0x00
#define NRF_W_REGISTER    0x20
#define NRF_R_RX_PAYLOAD  0x61
#define NRF_W_TX_PAYLOAD  0xA0
#define NRF_FLUSH_TX      0xE1
#define NRF_FLUSH_RX      0xE2
#define NRF_NOP           0xFF

// Bit definitions
#define NRF_CONFIG_PWR_UP   (1 << 1)
#define NRF_CONFIG_PRIM_RX  (1 << 0)
#define NRF_STATUS_RX_DR    (1 << 6)
#define NRF_STATUS_TX_DS    (1 << 5)
#define NRF_STATUS_MAX_RT   (1 << 4)

void nrf24_init(void);
void nrf24_write_register(uint8_t reg, uint8_t value);
uint8_t nrf24_read_register(uint8_t reg);
void nrf24_read_register_multi(uint8_t reg, uint8_t *data, uint8_t len);
void nrf24_write_register_multi(uint8_t reg, uint8_t *data, uint8_t len);
void nrf24_set_tx_addr(uint8_t *addr, uint8_t len);
void nrf24_set_rx_addr(uint8_t *addr, uint8_t len);
void nrf24_send(uint8_t *data, uint8_t len);
bool nrf24_is_sending(void);
uint8_t nrf24_get_status(void);
void nrf24_power_up_tx(void);
