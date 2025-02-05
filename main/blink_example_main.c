#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/uart.h"


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

static void initialize_uart()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUFFER_SIZE * 2, 0, 0, NULL, 0);
}

void app_main(void)
{
    typedef struct {float throttle; float yaw; float pitch; float roll;} MotorData;
    initialize_uart();
    /* Configure the peripheral according to the LED type */
    configure_led();
    MotorData currentData = {0, 0, 0, 0};
    uint8_t data[BUFFER_SIZE];

    while (1) {
        ESP_LOGI("Test 1", "Value 1: %f, Value 2: %f, Value 3: %f, Value 4: %f", currentData.throttle, currentData.yaw, currentData.pitch, currentData.roll);
        ESP_LOGI("Test 1", "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;

        int sentDataLength = uart_read_bytes(UART_NUM, data, BUFFER_SIZE, 20 / portTICK_PERIOD_MS);
        if(sentDataLength > 0)
        {
            ESP_LOGI("UART", "Received %d bytes", sentDataLength);
        }
        else if(sentDataLength == 0)
        {
            ESP_LOGI("UART", "No data sent yet.");
        }

        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
