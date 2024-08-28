
#include <stdio.h>
//#include "esp_mac.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
void app_main(void)
{
  gpio_set_direction(GPIO_NUM_12,GPIO_MODE_DEF_INPUT);
  gpio_set_pull_mode(GPIO_NUM_12,GPIO_PULLUP_ONLY);
  gpio_set_direction(GPIO_NUM_13,GPIO_MODE_DEF_OUTPUT );

  while (true)
  {
    //gpio_set_level(13,1);
    if(gpio_get_level(GPIO_NUM_12))
        gpio_set_level(GPIO_NUM_13,0);
    else
        gpio_set_level(GPIO_NUM_13,1);
    /* code */

    vTaskDelay(10);
  }
    
}

