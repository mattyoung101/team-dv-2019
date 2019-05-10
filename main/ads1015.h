// Source: https://github.com/Molorius/esp32-ADS1115
// License: none provided by the author

#ifdef __cplusplus
extern "C" {
#endif

#pragma once

#include <stdio.h>
#include "driver/i2c.h"
#include "driver/gpio.h"

typedef enum { // register address
  ADS1015_CONVERSION_REGISTER_ADDR = 0,
  ADS1015_CONFIG_REGISTER_ADDR,
  ADS1015_LO_THRESH_REGISTER_ADDR,
  ADS1015_HI_THRESH_REGISTER_ADDR,
  ADS1015_MAX_REGISTER_ADDR
} ADS1015_register_addresses_t;

typedef enum { // multiplex options
  ADS1015_MUX_0_1 = 0,
  ADS1015_MUX_0_3,
  ADS1015_MUX_1_3,
  ADS1015_MUX_2_3,
  ADS1015_MUX_0_GND,
  ADS1015_MUX_1_GND,
  ADS1015_MUX_2_GND,
  ADS1015_MUX_3_GND,
} ADS1015_mux_t;

typedef enum { // full-scale resolution options
  ADS1015_FSR_6_144 = 0,
  ADS1015_FSR_4_096,
  ADS1015_FSR_2_048,
  ADS1015_FSR_1_024,
  ADS1015_FSR_0_512,
  ADS1015_FSR_0_256,
} ADS1015_fsr_t;

typedef enum { // samples per second
  ADS1015_SPS_8 = 0,
  ADS1015_SPS_16,
  ADS1015_SPS_32,
  ADS1015_SPS_64,
  ADS1015_SPS_128,
  ADS1015_SPS_250,
  ADS1015_SPS_475,
  ADS1015_SPS_860
} ADS1015_sps_t;

typedef enum {
  ADS1015_MODE_CONTINUOUS = 0,
  ADS1015_MODE_SINGLE
} ADS1015_mode_t;

typedef union { // configuration register
  struct {
    uint16_t COMP_QUE:2;  // bits 0..  1  Comparator queue and disable
    uint16_t COMP_LAT:1;  // bit  2       Latching Comparator
    uint16_t COMP_POL:1;  // bit  3       Comparator Polarity
    uint16_t COMP_MODE:1; // bit  4       Comparator Mode
    uint16_t DR:3;        // bits 5..  7  Data rate
    uint16_t MODE:1;      // bit  8       Device operating mode
    uint16_t PGA:3;       // bits 9..  11 Programmable gain amplifier configuration
    uint16_t MUX:3;       // bits 12.. 14 Input multiplexer configuration
    uint16_t OS:1;        // bit  15      Operational status or single-shot conversion start
  } bit;
  uint16_t reg;
} ADS1015_CONFIG_REGISTER_Type;

typedef struct {
  bool in_use; // gpio is used
  gpio_num_t pin; // ready pin
  xQueueHandle gpio_evt_queue; // pin triggered queue
} ADS1015_rdy_pin_t;

typedef struct {
  ADS1015_CONFIG_REGISTER_Type config;
  i2c_port_t i2c_port;
  int address;
  ADS1015_rdy_pin_t rdy_pin;
  ADS1015_register_addresses_t last_reg; // save last accessed register
  bool changed; // save if a value was changed or not
  TickType_t max_ticks; // maximum wait ticks for i2c bus
} ADS1015_t;

// initialize device
ADS1015_t ADS1015_config(i2c_port_t i2c_port, uint8_t address); // set up configuration

// set configuration
void ADS1015_set_rdy_pin(ADS1015_t* ads, gpio_num_t gpio); // set up data-ready pin
void ADS1015_set_mux(ADS1015_t* ads, ADS1015_mux_t mux); // set multiplexer
void ADS1015_set_pga(ADS1015_t* ads, ADS1015_fsr_t fsr); // set fsr
void ADS1015_set_mode(ADS1015_t* ads, ADS1015_mode_t mode); // set read mode
void ADS1015_set_sps(ADS1015_t* ads, ADS1015_sps_t sps); // set sampling speed
void ADS1015_set_max_ticks(ADS1015_t* ads, TickType_t max_ticks); // maximum wait ticks for i2c bus

int16_t ADS1015_get_raw(ADS1015_t* ads); // get voltage in bits
double ADS1015_get_voltage(ADS1015_t* ads); // get voltage in volts

#ifdef __cplusplus
}
#endif