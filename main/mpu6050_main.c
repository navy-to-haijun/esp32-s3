#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "mpu6050.h"
#include "udp_client.h"

static const char *TAG = "main-task";
extern EventGroupHandle_t s_wifi_event_group;
QueueHandle_t mpu6050data_queue;

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    /*初始化wifi station*/
    wifi_init_sta();
    /*创建一个队列*/
    mpu6050data_queue= xQueueCreate(20, sizeof(float_to_u8)*3);
    if( mpu6050data_queue != NULL ) 
        ESP_LOGI(TAG, "Creat Queue successfully");
    else 
        ESP_LOGE(TAG, "Creat Queue Error");
    
    

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
     {
        ESP_LOGI(TAG, "Wifi connection successful");
        /*创建任务*/
        // xTaskCreate(mpu6050_task, "mpu6050_task", 1024*10, NULL, 5, NULL);
        xTaskCreate(udp_client_task, "udp_client_task", 1024*10, NULL, 7, NULL);
    } 
    else if (bits & WIFI_FAIL_BIT) 
    {
        ESP_LOGI(TAG, "Wifi connection failed");
    } else 
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    } 
}
