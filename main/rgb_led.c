#include "rgb_led.h"

static rgb_led_mode_t mode = RGB_LED_MODE_RAINBOW;
static struct led_color_t led_strip_buf_1[LED_NUM];
static struct led_color_t led_strip_buf_2[LED_NUM];
static struct led_strip_t led_strip = {
    .rgb_led_type = RGB_LED_TYPE_WS2812,
    .rmt_channel = RMT_CHANNEL_1,
    .rmt_interrupt_num = 19,
    .gpio = GPIO_NUM_21,
    .led_strip_buf_1 = led_strip_buf_1,
    .led_strip_buf_2 = led_strip_buf_2,
    .led_strip_length = LED_NUM
};

// LED colours
static struct led_color_t red = {0xE7, 0x00, 0x00};
static struct led_color_t orange = {0xFF, 0x8C, 0x00};
static struct led_color_t yellow = {0xFF, 0xEF, 0x00};
static struct led_color_t green = {0x00, 0x81, 0x1F};
static struct led_color_t blue = {0x00, 0x44, 0xFF};
static struct led_color_t purple = {0x76, 0x00, 0x89};
static struct led_color_t white = {0xFF, 0xFF, 0xFF};

static uint8_t ledIndex = 0;

/** Task which controls RGB LEDs **/
static void rgb_led_task(void *pvParameters){
    static const char *TAG= "RGBLEDTask";
    led_strip.access_semaphore = xSemaphoreCreateBinary();
    float delta = esp_timer_get_time() / 1000000.0f; // delta time in seconds

    if (!led_strip_init(&led_strip)){
        ESP_LOGE(TAG, "RGB LED init error!");
        vTaskDelete(NULL);
        while (true);
    }

    esp_task_wdt_add(NULL);
    ESP_LOGI(TAG, "RGB LED init OK");

    while (true){
        switch (mode){
            case RGB_LED_MODE_RAINBOW: {
                break;
            }

            default: {
                ESP_LOGW(TAG, "Unknown RGB LED mode: %d", mode);
                break;
            }
        }

        delta = (esp_timer_get_time() - delta) / 1000000.0f;
        esp_task_wdt_reset();
    }
}

void rgb_led_init(){
    xTaskCreate(rgb_led_task, "RGBLEDTask", 4096, NULL, configMAX_PRIORITIES - 3, NULL);
}

void rgb_led_change_mode(rgb_led_mode_t mode_){
    mode = mode_;
}