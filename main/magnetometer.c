#include "magnetometer.h"

static const char *TAG = "MLX90393";
uint8_t gain = MLX90393_GAIN_1X;

bool transceive(uint8_t *txbuf, uint8_t txlen, uint8_t *rxbuf, uint8_t rxlen){
    uint8_t status = 0;

    // write stage
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (MLX90393_DEFAULT_ADDR << 1) | I2C_MASTER_WRITE, I2C_ACK_MODE));
        ESP_ERROR_CHECK(i2c_master_write(cmd, txbuf, txlen, I2C_ACK_MODE));
        ESP_ERROR_CHECK(i2c_master_stop(cmd));

        esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
        i2c_cmd_link_delete(cmd);

        I2C_ERR_CHECK(err);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // read stage
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (MLX90393_DEFAULT_ADDR << 1), I2C_ACK_MODE));
        ESP_ERROR_CHECK(i2c_master_start(cmd));
        ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (MLX90393_DEFAULT_ADDR << 1) | I2C_MASTER_READ, I2C_ACK_MODE));
        
        // read in status byte
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &status, I2C_ACK_MODE));
        // read in other bytes
        if (rxbuf != NULL){
            ESP_ERROR_CHECK(i2c_master_read(cmd, rxbuf, rxlen, I2C_ACK_MODE));
        }

        ESP_ERROR_CHECK(i2c_master_stop(cmd));
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(I2C_TIMEOUT));
        i2c_cmd_link_delete(cmd);

        I2C_ERR_CHECK(ret);
    }

    return (status >> 2) == 0;
}

bool mlx_set_gain(enum mlx90393_gain gain_){
    bool ok;
    gain = gain_;

    /* Set CONF1, including gain. */
    uint8_t tx[4] = { MLX90393_REG_WR,
                      0x00,
                      (uint8_t)(((gain & 0x7) << MLX90393_GAIN_SHIFT) | MLX90393_HALL_CONF),
                      (MLX90393_CONF1 & 0x3F) << 2};

    /* Perform the transaction. */
    ok = transceive(tx, sizeof tx, NULL, 0);

    /* Check status byte for errors. */
    return ok;
}

bool mlx_read_data(float *x, float *y, float *z){
    bool ok;
    uint8_t tx_mode[1] = { MLX90393_REG_SM | MLX90393_AXIS_ALL };
    uint8_t tx[1] = { MLX90393_REG_RM | MLX90393_AXIS_ALL };
    uint8_t rx[6] = { 0 };
    int16_t xi, yi, zi;

    /* Set the device to single measurement mode */
    ok = transceive(tx_mode, sizeof tx_mode, NULL, 0);
    if (!ok) {
        return false;
    }

    /* Read a single data sample. */
    ok = transceive(tx, sizeof tx, rx, sizeof rx);
    if (!ok) {
        return false;
    }

    /* Convert data to uT and float. */
    xi = (rx[0] << 8) | rx[1];
    yi = (rx[2] << 8) | rx[3];
    zi = (rx[4] << 8) | rx[5];

    *x = (float)xi * mlx90393_lsb_lookup[gain][0][0];
    *y = (float)yi * mlx90393_lsb_lookup[gain][0][0];
    *z = (float)zi * mlx90393_lsb_lookup[gain][0][1];

    return ok;
}

static void receive_calibration(){

}

// source: https://github.com/PaulStoffregen/NXPMotionSense/blob/master/examples/CalibrateSensors/CalibrateSensors.ino
void motion_cal_task(void *pvParameter){
    static const char *TAG = "MotionCalTask";
    float mX, mY, mZ;
    uint16_t loopcount = 0;

    // initialise IMU (gyro + accel) and mag
    mlx_set_gain(MLX90393_GAIN_1X);
    simu_init();
    simu_calibrate();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "Motion calibration task init OK");

    while (true){
        vec3d_t gyro = simu_read_gyro();
        mlx_read_data(&mX, &mY, &mZ);
        printfln("Raw:%f,%f,%f,%f,%f,%f,%f,%f,%f", gyro.x, gyro.y, gyro.z, 0.0f, 0.0f, 0.0f, mX, mY, mZ);
        loopcount++;

        receive_calibration();

        if (loopcount == 50 || loopcount > 100){
            // printfln("Cal1:");
        }
    }
}