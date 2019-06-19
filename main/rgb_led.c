#include "rgb_led.h"

static rgb_led_mode_t mode = RGB_LED_MODE_RAINBOW;
static struct led_color_t led_strip_buf_1[LED_NUM];
static struct led_color_t led_strip_buf_2[LED_NUM];
static struct led_strip_t led_strip = {
    .rgb_led_type = RGB_LED_TYPE_WS2812,
    .rmt_channel = RMT_CHANNEL_1,
    .rmt_interrupt_num = 19,
    .gpio = LED_PIN,
    .led_strip_buf_1 = led_strip_buf_1,
    .led_strip_buf_2 = led_strip_buf_2,
    .led_strip_length = LED_NUM
};

// LED colours
#define RED {0xE7, 0x00, 0x00}
#define ORANGE {0xFF, 0x8C, 0x00}
#define YELLOW {0xFF, 0xEF, 0x00}
#define GREEN {0x00, 0x81, 0x1F}
#define BLUE {0x00, 0x44, 0xFF}
#define PURPLE {0x76, 0x00, 0x89}
#define WHITE {0xFF, 0xFF, 0xFF}

static struct led_color_t rainbow[] = {RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE};
static float totalDelta = 0.0f;

// partial source: https://codeforwin.org/2017/03/c-program-to-right-rotate-array.html
// static void array_rotate(struct led_color_t *arr, size_t size){
//     struct led_color_t last = arr[size - 1];

//     for(int i = size - 1; i > 0; i--){
//         arr[i] = arr[i - 1];
//     }

//     arr[0] = last;
// }

// static struct led_color_t lerp_colour(struct led_color_t a, struct led_color_t b, float alpha){
//     struct led_color_t lerped = {
//         lerp(a.red, b.red, alpha), // red
//         lerp(a.green, b.green, alpha), // green
//         lerp(a.blue, b.blue, alpha), // blue
//     };
//     return lerped;
// }

/** Task which controls RGB LEDs **/
static void rgb_led_task(void *pvParameters){
    static const char *TAG = "RGBLEDTask";
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
                // if (totalDelta > RAINBOW_TRANSITION_TIME){
                //     ESP_LOGI(TAG, "Updating rainbow array");
                //     array_rotate(rainbow, 6);
                //     totalDelta = 0.0f;
                // }

                // // hop to every 2nd LED and set its colour. These LEDs are the "keyframes".
                // uint8_t rainbowIndex = 0;
                // for (int i = 0; i < LED_NUM; i += 2){
                //     led_strip_set_pixel_color(&led_strip, i, &rainbow[rainbowIndex++]);
                // }

                // // interpolate the other LEDs
                // for (int i = 1; i < LED_NUM; i += 2){
                //     struct led_color_t lerped = lerp_colour(rainbow[i - 1], rainbow[(i + 1) % 6], totalDelta / 5.0f);
                //     led_strip_set_pixel_color(&led_strip, i, &lerped);
                // }
                for (int i = 0; i < LED_NUM; i++){
                    led_strip_set_pixel_color(&led_strip, i, &rainbow[i % 6]);
                }
                break;
            }

            default: {
                ESP_LOGW(TAG, "Unknown RGB LED mode: %d", mode);
                break;
            }
        }

        // update LEDs
        led_strip_show(&led_strip);

        // update delta time
        delta = (esp_timer_get_time() - delta) / 1000000.0f;
        totalDelta += delta;
        esp_task_wdt_reset();
    }
}

void rgb_led_init(){
    xTaskCreate(rgb_led_task, "RGBLEDTask", 4096, NULL, configMAX_PRIORITIES - 3, NULL);
}

void rgb_led_change_mode(rgb_led_mode_t mode_){
    mode = mode_;
    totalDelta = 0.0f;
}