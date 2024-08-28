

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
 
#define GPIO_OUT_W1TS_REG 0x60004008
#define GPIO_OUT_W1TC_REG 0x6000400c
#define GPIO_ENABLE_REG   0x60004020
#define GPIO13             13
#define DELAY_MS          200
                       
void app_main(void)
{
   volatile __UINT32_TYPE__* gpio_out_w1ts_reg = (volatile __UINT32_TYPE__*) GPIO_OUT_W1TS_REG;
   volatile __UINT32_TYPE__* gpio_out_w1tc_reg = (volatile __UINT32_TYPE__*) GPIO_OUT_W1TC_REG;
   volatile __UINT32_TYPE__* gpio_enable_reg = (volatile __UINT32_TYPE__*) GPIO_ENABLE_REG;

   *gpio_enable_reg = (1 << GPIO13);

   while(1)
   {
      *gpio_out_w1ts_reg = (1 << GPIO13);
      vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
      *gpio_out_w1tc_reg = (1 << GPIO13);
      vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
   }

}
