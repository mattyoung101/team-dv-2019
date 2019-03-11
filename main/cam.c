#include "cam.h"

void cam_init(void){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));

    // Set UART pins(TX: IO16 (UART2 default), RX: IO17 (UART2 default), RTS: IO18, CTS: IO19)
    // TODO figure out pins
    //ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19));

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, SERIAL_BUF_LEN * 2, SERIAL_BUF_LEN * 2, 8, NULL, 0));
}

void cam_update(void){
    uint8_t byte;
    // wait only one tick for a byte to come through UART, cam_read() is called every tick so we'll get it eventually
    uart_read_bytes(UART_NUM_1, &byte, 1, 1);
    // we've read the byte we need, so start from scratch again
    uart_flush_input(UART_NUM_1);

    if (byte == CAM_BEGIN_BYTE){
        // begin, bfound, bx, by, yfound, yx, yy, end
        
        // temporary buffer on the stack, disposed after this method exits
        // shouldn't overflow stack as its only 6 bytes
        uint8_t *buffer = alloca((CAM_DATA_LEN - 2) * sizeof(uint8_t));

        // wait indefinitely for the camera buffer to fill up, 
        // shouldn't get stuck since if we read the start byte the camera must be sending to us
        // we've already read the first byte and we don't need the end byte, so only read 6 bytes
        uart_read_bytes(UART_NUM_1, buffer, CAM_DATA_LEN - 2, portMAX_DELAY);

        // now we can read straight from the alloca'd region because the "[]" operator is just syntactic sugar
        // for *(buffer + i)
        goalBlue.exists = buffer[0];
        goalBlue.x = buffer[1] - CAM_FRAME_WIDTH / 2 + CAM_OFFSET_X;
        goalBlue.y = buffer[2] - CAM_FRAME_HEIGHT / 2 + CAM_OFFSET_Y;

        goalYellow.exists = buffer[3];
        goalYellow.x = buffer[4] - CAM_FRAME_WIDTH / 2 + CAM_OFFSET_X;
        goalYellow.y = buffer[5] - CAM_FRAME_HEIGHT / 2 + CAM_OFFSET_Y;

        // discard the rest of the internal buffer
        uart_flush_input(UART_NUM_1);
        
        cam_calc();
    }
}

void cam_calc(void){
    goalBlue.angle = mod(450 - roundf(RAD_DEG * atan2f(goalBlue.y, goalBlue.x)), 360);
    goalBlue.length = sqrtf(powf(goalBlue.x, 2) + powf(goalBlue.y, 2));

    goalYellow.angle = mod(450 - roundf(RAD_DEG * atan2f(goalYellow.y, goalYellow.x)), 360);
    goalYellow.length = sqrtf(powf(goalYellow.x, 2) + powf(goalYellow.y, 2));

    if (!goalBlue.exists && !goalYellow.exists){
        robotX = CAM_NO_VALUE;
        robotY = CAM_NO_VALUE;
    } else {
        // Triangulate goal position
        // First calculate the position of the robot, according to both goals
        float blueY = goalBlue.length * sinf(goalBlue.angle) + (ENEMY_GOAL ? -HALFWAY_DISTANCE : HALFWAY_DISTANCE);
        float blueX = goalBlue.length * cosf(goalBlue.angle);

        float yellowY = goalYellow.length * sinf(goalYellow.angle) + (ENEMY_GOAL ? HALFWAY_DISTANCE : -HALFWAY_DISTANCE);
        float yellowX = goalYellow.length * cosf(goalYellow.angle);

        // Now find the average position of the robot based on both goals
        if (goalBlue.exists && !goalYellow.exists){
            // only blue
            robotX = blueX;
            robotY = blueY;
        } else if (goalYellow.exists && !goalBlue.exists){
            // only yellow
            robotX = yellowX;
            robotY = yellowY;
        } else {
            // both
            robotX = (blueX + yellowX) / 2.0f;
            robotY = (blueY + yellowY) / 2.0f;
        }
    }
}