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

#include "dap_main.h"

static const char *TAG = "main-task";
void winusbv2_init(uint8_t busid, uintptr_t reg_base);
void app_main(void)
{
     ESP_LOGI(TAG, "Hello world!");
     chry_dap_init(0, ESP_USBD_BASE);
  //   winusbv2_init(0, ESP_USBD_BASE);
     // cdc_acm_init1(0, 0x60080000);

     while (1)
     {
         chry_dap_handle();
          vTaskDelay(10 / portTICK_PERIOD_MS);
     }
}
