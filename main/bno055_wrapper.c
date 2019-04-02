#include "bno055_wrapper.h"

static const char *TAG = "BNO055";
#define BS_DELAY 500

s8 bno_i2c_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
	u8 idiot = *reg_data;
	ESP_LOGV(TAG, "Writing: dev_addr %d, reg_addr %d, reg_data %d, cnt %d", 
	(int) dev_addr, (int) idiot, (int) *reg_data, (int) cnt);

    u8 iError = BNO055_INIT_VALUE;
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);

	i2c_master_write_byte(cmd, reg_addr, 1);
	i2c_master_write(cmd, reg_data, cnt, 1);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
	if (espRc == ESP_OK) {
		ESP_LOGV(TAG, "Write successful");
		iError = BNO055_SUCCESS;
	} else {
		ESP_LOGV(TAG, "Write failed, error %s", esp_err_to_name(espRc));
		iError = BNO055_ERROR;
	}
	i2c_cmd_link_delete(cmd);

	return (s8) iError;
}

s8 bno_i2c_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	u8 idiot = *reg_data;
	ESP_LOGV(TAG, "Reading: dev_addr %d, reg_addr %d, reg_data %d, cnt %d", 
	(int) dev_addr, (int) idiot, (int) reg_data, (int) cnt);

	u8 iError = BNO055_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, reg_addr, 1);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, 1);

	if (cnt > 1) {
		i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
	if (espRc == ESP_OK) {
		ESP_LOGV(TAG, "Read successful");
		iError = BNO055_SUCCESS;
	} else {
		ESP_LOGV(TAG, "Read failed, error %s", esp_err_to_name(espRc));
		iError = BNO055_ERROR;
	}

	i2c_cmd_link_delete(cmd);

	return (s8) iError;
}

void bno_delay(u32 ms){
	ESP_LOGV(TAG, "Delaying for %d ms", ms);
	vTaskDelay(pdMS_TO_TICKS(ms));
}