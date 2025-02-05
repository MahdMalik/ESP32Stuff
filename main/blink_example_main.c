#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"


#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_GPIO

#define BUFFER_SIZE 1024

#define UART_NUM UART_NUM_1

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    printf("Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

void app_main(void)
{
    typedef struct {float throttle; float yaw; float pitch; float roll;} MotorData;
    /* Configure the peripheral according to the LED type */
    configure_led();
    MotorData currentData = {0, 0, 0, 0};
    while (1) {
        ESP_LOGI("Test 1", "Value 1: %f, Value 2: %f, Value 3: %f, Value 4: %f", currentData.throttle, currentData.yaw, currentData.pitch, currentData.roll);
        ESP_LOGI("Test 1", "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
