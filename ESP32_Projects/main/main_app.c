/**
 * FreeRTOS LED Demo
 * 
 * One task flashes an LED at a rate specified by a value set in another task.
 * 
 * Date: December 4, 2020
 * Author: Shawn Hymel
 * License: 0BSD
 */
/*Project Overview
Objective:
Develop an embedded system using the ESP32-S3 microcontroller, utilizing FreeRTOS to run two concurrent tasks:

Blinking an LED on GPIO13.
Handling UART Communication on GPIO1 (TX) and GPIO2 (RX).
Data will be sent via a serial monitor (e.g., PuTTY) and received in the Terminal of VSCode.

Implementation Plan
1. LED Blinking (GPIO13):

Implement a FreeRTOS task to blink an LED.
2. UART Communication (GPIO1 & GPIO2):

Set up UART on GPIO1 (TX) and GPIO2 (RX).
Create a FreeRTOS task to handle UART data, displaying received data in the VSCode Terminal.
Integration:

Run both tasks concurrently using FreeRTOS.
Test with PuTTY to ensure proper UART communication and LED operation.
Expected Outcome
LED Blinking: LED on GPIO13 blinks continuously.
UART Communication: Data sent from PuTTY appears in VSCode Terminal.
*/
// Link for source: https://www.youtube.com/watch?v=LHCZPkoHGNc
// https://www.youtube.com/watch?v=C33J-1aEfok  UART Echo Task on ESP32 using ESP-IDF
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/uart.html


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "string.h"

// Needed for atoi()
#include <stdlib.h>

#define LOW 0
#define HIGH 1
#define led_pin GPIO_NUM_13

const uart_port_t uart_num = UART_NUM_2;
static const uint8_t buf_len = 20;

static int led_delay = 200;
static int val=0;


// CONFIG_FREERTOS_UNICORE
// Task: Blink LED at rate set by global variable
void blink(void)
{
    gpio_set_direction(GPIO_NUM_13,GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(led_pin, LOW);
        vTaskDelay(led_delay / portTICK_PERIOD_MS);
        gpio_set_level(led_pin, HIGH);
        vTaskDelay(led_delay / portTICK_PERIOD_MS);
    }
}
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-reference/peripherals/uart.html

/* Steps to impliment uart test:
Set Communication Parameters - Setting baud rate, data bits, stop bits, etc.

Set Communication Pins - Assigning pins for connection to a device

Install Drivers - Allocating ESP32-S3's resources for the UART driver

Run UART Communication - Sending/receiving data

Use Interrupts - Triggering interrupts on specific communication events

Deleting a Driver - Freeing allocated resources if a UART communication is no longer required
*/


//static const char *TAG = "UART TEST";
static void UartTest(void *arg)
{

    //const uart_port_t uart_num = UART_NUM_2;
    uart_config_t uart_config =
        {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = UART_SCLK_APB, // UART_SCLK_DEFAULT
        };
    
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 1, 2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size,
                                        uart_buffer_size, 10, &uart_queue, 0));

    // Write data to UART.
    // char* test_str = "This is a test string.\n\r";

    uint8_t data[1];

    ESP_LOGI("","Multi-task LED Demo+++++");
    ESP_LOGI("","Enter a number in milliseconds to change the LED delay******.");
    
    char buf[buf_len];
    uint8_t idx = 0;

    // Clear whole buffer
    memset(buf, 0, buf_len);
    //uint8_t data[128];
    int length = 0;
    vTaskDelay(500 / portTICK_PERIOD_MS);    // Configure serial and wait a second 
    // Loop forever

    ESP_LOGI("UART Test: ", "Begin..............................................................");
    while (1)
    {
        int len = uart_read_bytes(uart_num, data, 1, 10); // last number 10 means how fast you can send data. it is uart buffer clear timeout
        uart_write_bytes(uart_num, (const char *)data, len);
    // Read characters from serial
    if (len) 
    {       
      // Update delay variable and reset buffer if we get a newline character
      if (data[0] == 13) // 13 is ascii code for Enter on keyboard
      {         
        val = atoi(buf);
        printf("Integer value = %d\n", val);
        if(val >50) // small number for vTaskdelay will cause problem
          led_delay =val;
        ESP_LOGI("","int delay to: %d",led_delay);
        memset(buf, 0, buf_len);
        idx = 0;
      } 
      else 
      {    
        //ESP_LOGI("","Please... ");    
        // Only append if index is not over message limit
        if (idx < buf_len - 1) 
        {
          //ESP_LOGI("","num= %d",data[0]);
          ESP_LOGE("","num= %s",data);  
          //printf("%s", data);
          buf[idx] = data[0];
          idx++ ;
        }
      }
    } 
    }
}
void app_main(void)
{
   //xTaskCreate( Function to be called, Name of task, Stack size(bytes in ESP32, words in freertos) Parameter to pass to function, 
   //Task Priority 0 lowest priority (configMAX_PRIORITIES -1), Handle_t to task);
   xTaskCreate( blink, "LED blink", 2048, NULL , 1, 0 );
   xTaskCreate(UartTest, "UartTest_task", 4098, NULL, 10, 0);
  
}
