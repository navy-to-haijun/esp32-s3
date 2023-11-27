
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "mpu6050.h"
#include "udp_client.h"


#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

static const char *TAG = "main-task";
QueueHandle_t mpu6050data_queue;

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /*创建一个队列*/
    mpu6050data_queue= xQueueCreate(20, sizeof(int16_t)*3);
    if( mpu6050data_queue != NULL ) 
        ESP_LOGI(TAG, "Creat Queue successfully");
    else 
        ESP_LOGE(TAG, "Creat Queue Error");

    /*创建任务*/
    xTaskCreate(mpu6050_task, "mpu6050_task", 1024*10, NULL, 5, NULL);
    xTaskCreate(udp_client_task, "udp_client_task", 1024*10, NULL, 7, NULL);
    
    
    
}
