// Source: https://github.com/Molorius/esp32-ADS1015
// License: none provided by the author
#include "ads1015.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static void IRAM_ATTR gpio_isr_handler(void* arg) {
  const bool ret = 1; // dummy value to pass to queue
  xQueueHandle gpio_evt_queue = (xQueueHandle) arg; // find which queue to write
  xQueueSendFromISR(gpio_evt_queue, &ret, NULL);
}

static esp_err_t ADS1015_write_register(ADS1015_t* ads, ADS1015_register_addresses_t reg, uint16_t data) {
  i2c_cmd_handle_t cmd;
  esp_err_t ret;
  uint8_t out[2];

  out[0] = data >> 8; // get 8 greater bits
  out[1] = data & 0xFF; // get 8 lower bits
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd); // generate a start command
  i2c_master_write_byte(cmd,(ads->address<<1) | I2C_MASTER_WRITE,1); // specify address and write command
  i2c_master_write_byte(cmd,reg,1); // specify register
  i2c_master_write(cmd,out,2,1); // write it
  i2c_master_stop(cmd); // generate a stop command
  ret = i2c_master_cmd_begin(ads->i2c_port, cmd, ads->max_ticks); // send the i2c command
  i2c_cmd_link_delete(cmd);
  ads->last_reg = reg; // change the internally saved register
  return ret;
}

static esp_err_t ADS1015_read_register(ADS1015_t* ads, ADS1015_register_addresses_t reg, uint8_t* data, uint8_t len) {
  i2c_cmd_handle_t cmd;
  esp_err_t ret;

  if(ads->last_reg != reg) { // if we're not on the correct register, change it
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(ads->address<<1) | I2C_MASTER_WRITE,1);
    i2c_master_write_byte(cmd,reg,1);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(ads->i2c_port, cmd, ads->max_ticks);
    i2c_cmd_link_delete(cmd);
    ads->last_reg = reg;
  }
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd); // generate start command
  i2c_master_write_byte(cmd,(ads->address<<1) | I2C_MASTER_READ,1); // specify address and read command
  i2c_master_read(cmd, data, len, 0); // read all wanted data
  i2c_master_stop(cmd); // generate stop command
  ret = i2c_master_cmd_begin(ads->i2c_port, cmd, ads->max_ticks); // send the i2c command
  i2c_cmd_link_delete(cmd);
  return ret;
}

ADS1015_t ADS1015_config(i2c_port_t i2c_port, uint8_t address) {
  ADS1015_t ads; // setup configuration with default values
  ads.config.bit.OS = 1; // always start conversion
  ads.config.bit.MUX = ADS1015_MUX_0_GND;
  ads.config.bit.PGA = ADS1015_FSR_4_096;
  ads.config.bit.MODE = ADS1015_MODE_SINGLE;
  ads.config.bit.DR = ADS1015_SPS_64;
  ads.config.bit.COMP_MODE = 0;
  ads.config.bit.COMP_POL = 0;
  ads.config.bit.COMP_LAT = 0;
  ads.config.bit.COMP_QUE = 0b11;

  ads.i2c_port = i2c_port; // save i2c port
  ads.address = address; // save i2c address
  ads.rdy_pin.in_use = 0; // state that rdy_pin not used
  ads.last_reg = ADS1015_MAX_REGISTER_ADDR; // say that we accessed invalid register last
  ads.changed = 1; // say we changed the configuration
  ads.max_ticks = 10/portTICK_PERIOD_MS;
  return ads; // return the completed configuration
}

void ADS1015_set_mux(ADS1015_t* ads, ADS1015_mux_t mux) {
  ads->config.bit.MUX = mux;
  ads->changed = 1;
}

void ADS1015_set_rdy_pin(ADS1015_t* ads, gpio_num_t gpio) {
  const static char* TAG = "ADS1015_set_rdy_pin";
  gpio_config_t io_conf;
  esp_err_t err;

  io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE; // positive to negative (pulled down)
  io_conf.pin_bit_mask = 1<<gpio;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 1;
  io_conf.pull_down_en = 0;
  gpio_config(&io_conf); // set gpio configuration

  ads->rdy_pin.gpio_evt_queue = xQueueCreate(1, sizeof(bool));
  gpio_install_isr_service(0);

  ads->rdy_pin.in_use = 1;
  ads->rdy_pin.pin = gpio;
  ads->config.bit.COMP_QUE = 0b00; // assert after one conversion
  ads->changed = 1;

  err = ADS1015_write_register(ads, ADS1015_LO_THRESH_REGISTER_ADDR,0); // set lo threshold to minimum
  if(err) ESP_LOGE(TAG,"could not set low threshold: %s",esp_err_to_name(err));
  err = ADS1015_write_register(ads, ADS1015_HI_THRESH_REGISTER_ADDR,0xFFFF); // set hi threshold to maximum
  if(err) ESP_LOGE(TAG,"could not set high threshold: %s",esp_err_to_name(err));
}

void ADS1015_set_pga(ADS1015_t* ads, ADS1015_fsr_t fsr) {
  ads->config.bit.PGA = fsr;
  ads->changed = 1;
}

void ADS1015_set_mode(ADS1015_t* ads, ADS1015_mode_t mode) {
  ads->config.bit.MODE = mode;
  ads->changed = 1;
}

void ADS1015_set_sps(ADS1015_t* ads, ADS1015_sps_t sps) {
  ads->config.bit.DR = sps;
  ads->changed = 1;
}

void ADS1015_set_max_ticks(ADS1015_t* ads, TickType_t max_ticks) {
  ads->max_ticks = max_ticks;
}

int16_t ADS1015_get_raw(ADS1015_t* ads) {
  const static char* TAG = "ADS1015_get_raw";
  const static uint16_t sps[] = {8,16,32,64,128,250,475,860};
  const static uint8_t len = 2;
  uint8_t data[2];
  esp_err_t err;
  bool tmp; // temporary bool for reading from queue

  if(ads->rdy_pin.in_use) {
    gpio_isr_handler_add(ads->rdy_pin.pin, gpio_isr_handler, (void*)ads->rdy_pin.gpio_evt_queue);
    xQueueReset(ads->rdy_pin.gpio_evt_queue);
  }
  // see if we need to send configuration data
  if((ads->config.bit.MODE==ADS1015_MODE_SINGLE) || (ads->changed)) { // if it's single-ended or a setting changed
    err = ADS1015_write_register(ads, ADS1015_CONFIG_REGISTER_ADDR, ads->config.reg);
    if(err) {
      ESP_LOGE(TAG,"could not write to device: %s",esp_err_to_name(err));
      if(ads->rdy_pin.in_use) {
        gpio_isr_handler_remove(ads->rdy_pin.pin);
        xQueueReset(ads->rdy_pin.gpio_evt_queue);
      }
      return 0;
    }
    ads->changed = 0; // say that the data is unchanged now
  }

  if(ads->rdy_pin.in_use) {
    xQueueReceive(ads->rdy_pin.gpio_evt_queue, &tmp, portMAX_DELAY);
    gpio_isr_handler_remove(ads->rdy_pin.pin);
  }
  else {
    // wait for 1 ms longer than the sampling rate, plus a little bit for rounding
    vTaskDelay((((1000/sps[ads->config.bit.DR]) + 1) / portTICK_PERIOD_MS)+1);
  }

  err = ADS1015_read_register(ads, ADS1015_CONVERSION_REGISTER_ADDR, data, len);
  if(err) {
    ESP_LOGE(TAG,"could not read from device: %s",esp_err_to_name(err));
    return 0;
  }
  return ((uint16_t)data[0] << 8) | (uint16_t)data[1];
}

double ADS1015_get_voltage(ADS1015_t* ads) {
  const double fsr[] = {6.144, 4.096, 2.048, 1.024, 0.512, 0.256};
  const int16_t bits = (1L<<15)-1;
  int16_t raw;

  raw = ADS1015_get_raw(ads);
  return (double)raw * fsr[ads->config.bit.PGA] / (double)bits;
}