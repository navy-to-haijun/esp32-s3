/**
 * @file        main.c
 * @brief       USB test
 * @details     
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "main-task";



void app_main(void)
{
   
    
   while (1)
   {
        ESP_LOGI(TAG, "Hello world!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
   }
}
