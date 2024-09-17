/**
 * @file        main.c
 * @brief       USB test
 * @details     
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sdkconfig.h"
#include "usbd_core.h"


static const char *TAG = "main-task";
void cdc_acm_init1(uint8_t busid, uintptr_t reg_base);
void app_main(void)
{
    ESP_LOGI(TAG, "Hello world!");
   cdc_acm_init1(0, 0x60080000);
    
   while (1)
   {
        vTaskDelay(10 / portTICK_PERIOD_MS);
   }
}
