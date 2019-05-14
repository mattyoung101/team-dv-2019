#include "ads1015.h"

static uint8_t m_i2cAddress = ADS1015_ADDRESS;
static uint8_t m_conversionDelay = ADS1115_CONVERSIONDELAY / 2;
static uint8_t m_bitShift = 4;
static adsGain_t m_gain = GAIN_EIGHT;

static const char *TAG = "ADS1015";

static void ads1015_write_reg(uint8_t addr, uint8_t reg, uint16_t value) {
    uint8_t send[2] = {0};
    send[0] = value >> 8;
    send[1] = value & 0xFF;

    esp_i2c_write(addr, reg, 2, send);
}

static uint16_t ads1015_read_reg(uint8_t addr, uint8_t reg) {
    uint8_t recv[2] = {0};
    // it's always REG_POINTER_CONVERT for some idiot reason, meaning the reg param is never used - Adafruit wtf?
    esp_i2c_read(addr, ADS1015_REG_POINTER_CONVERT, 2, recv);
    return ((recv[0] << 8) | recv[1]);
}

int16_t ads1015_read(uint8_t channel){
    if (channel > 3){
        ESP_LOGE(TAG, "Illegal channel: %d", channel);
        return 0;
    }

    // Start with default values
    uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE  | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_490SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

    // Set PGA/voltage range
    config |= m_gain;

    // Set single-ended input channel
    switch (channel){
        case (0):
            config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
            break;
        case (1):
            config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
            break;
        case (2):
            config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
            break;
        case (3):
            config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
            break;
    }

    // Set 'start single-conversion' bit
    config |= ADS1015_REG_CONFIG_OS_SINGLE;

    ads1015_write_reg(m_i2cAddress, ADS1015_REG_POINTER_CONFIG, config);
    // vTaskDelay(pdMS_TO_TICKS(m_conversionDelay)); // it's 8ms, so we can use FreeRTOS funcs instead of ets_delay_us

    return ads1015_read_reg(m_i2cAddress, ADS1015_REG_POINTER_CONVERT) >> m_bitShift;
}