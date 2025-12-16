#pragma once

#include <stdint.h>
#include "driver/spi_master.h"

// Pin definitions
#define PIN_NUM_MISO GPIO_NUM_7
#define PIN_NUM_MOSI GPIO_NUM_6
#define PIN_NUM_CLK GPIO_NUM_5
#define PIN_NUM_CS GPIO_NUM_15
#define PIN_NUM_MOTION GPIO_NUM_4

// Delay between address and data during read (tSRAD) — ~35 µs recommended
#define PMW_TSRAD_US 35

// Registers according to datasheet
#define REG_DELTA_X_L 0x03
#define REG_DELTA_X_H 0x04
#define REG_DELTA_Y_L 0x05
#define REG_DELTA_Y_H 0x06

// Global variables for sensor state
extern bool inBurst;
extern bool reportSQ;
extern int16_t dx, dy;
extern unsigned long lastTS;
extern unsigned long curTime;

// Function prototypes
void pmw3389_init(void);
void pmw3389_write(uint8_t reg, uint8_t val);
uint8_t pmw3389_read(uint8_t reg);
uint8_t pmw3389_read_burst(uint8_t *buffer, size_t length);
uint8_t pmw3389_read_old(uint8_t reg);
int16_t pmw3389_read_delta(uint8_t reg_low, uint8_t reg_high);
void pmw3389_upload_srom(void);
void pmw3389_selftest(void);
void pmw3389_set_dpi(uint32_t dpi);
void pmw3389_cycle_dpi(void);
void pmw3389_enable_burst_mode(void);
