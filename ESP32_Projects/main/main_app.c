// Link for source: https://www.youtube.com/watch?v=LHCZPkoHGNc
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 
#include "driver/gpio.h"

//CONFIG_FREERTOS_UNICORE

void blink(void)
{
    gpio_set_direction(GPIO_NUM_13,GPIO_MODE_OUTPUT);

    while(1)
    {
        gpio_set_level(GPIO_NUM_13,0);
        vTaskDelay(1500/portTICK_PERIOD_MS);
        gpio_set_level(GPIO_NUM_13,1 );
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}
void app_main(void)
{
    xTaskCreate(
        blink,   // Function to be called
     "LED blink",// Name of task
      2048,      // Stack size(bytes in ESP32, words in freertos)
     NULL ,      // Parameter to pass to function
        1,       // Task Priority 0 lowest priority (configMAX_PRIORITIES -1)
        0       // Handle_t to task
      );
  
}
