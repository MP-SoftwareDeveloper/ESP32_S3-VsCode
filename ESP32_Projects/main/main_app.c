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
//#define GPIO_NUM_40 GPIO_NUM_MAX
#define led_pin_green GPIO_NUM_43
#define led_pin_yellow GPIO_NUM_44

//const uart_port_t uart_num = UART_NUM_2;
//static const uint8_t buf_len = 20;

static int led_delay1 = 100;
static int led_delay2 = 300;
static int val=0;


// CONFIG_FREERTOS_UNICORE
// Task: Blink LED at rate set by global variable
void blink1(void)
{ 
    gpio_reset_pin(led_pin_green);
    gpio_set_direction(led_pin_green,GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(led_pin_green, LOW);
        vTaskDelay(led_delay1 / portTICK_PERIOD_MS);
        gpio_set_level(led_pin_green, HIGH);
        vTaskDelay(led_delay1 / portTICK_PERIOD_MS);
    }
}
void blink2(void)
{     
    gpio_reset_pin(led_pin_yellow);
    gpio_set_direction(led_pin_yellow,GPIO_MODE_OUTPUT);

    while (1)
    {        
        gpio_set_level(led_pin_yellow, HIGH);
        vTaskDelay(led_delay2 / portTICK_PERIOD_MS);        
        gpio_set_level(led_pin_yellow, LOW);
        vTaskDelay(led_delay2 / portTICK_PERIOD_MS);
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



void app_main(void)
{
   //xTaskCreate( Function to be called, Name of task, Stack size(bytes in ESP32, words in freertos) Parameter to pass to function, 
   //Task Priority 0 lowest priority (configMAX_PRIORITIES -1), Handle_t to task);
   xTaskCreate( blink1, "LED blink1", 2048, NULL , 1, 0 );
   xTaskCreate( blink2, "LED blink2", 2048, NULL , 1, 0 );
   //xTaskCreate(UartTest, "UartTest_task", 4098, NULL, 10, 0);
  
}
