// /**
//  * @file        main.c
//  * @brief       USB test
//  * @details
//  */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "dap_main.h"

#include "esp_vfs.h"
#include "esp_vfs_fat.h"

static const char *TAG = "main-task";

/*分区路径 */
const char *base_path = "/spiflash";
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

void msc_storage_init(uint8_t busid, uintptr_t reg_base);
void msc_ram_init(uint8_t busid, uintptr_t reg_base);

void app_main(void)
{
     ESP_LOGI(TAG, "Hello world!");

     msc_storage_init(0, ESP_USBD_BASE);



//      /*配置信息 */
     const esp_vfs_fat_mount_config_t mount_config = {
          .max_files = 5,                    // 一次性最大打开的文件数
         .format_if_mount_failed = false,     // 如果挂载失败，就格式化
         .allocation_unit_size = CONFIG_WL_SECTOR_SIZE, // 分配单元大小
         };
     /*挂载*/
     esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
     // msc_ram_init(0, ESP_USBD_BASE);

   
 //    chry_dap_init(0, ESP_USBD_BASE);
  //   winusbv2_init(0, ESP_USBD_BASE);
     // cdc_acm_init1(0, 0x60080000);


    // const char *filename = "/spiflash/example.txt";

    // FILE *f = fopen(filename, "wb");
    // if (f == NULL) {
    //     perror("fopen"); // Print reason why fopen failed
    //     ESP_LOGE(TAG, "Failed to open file for writing");
    //     return;
    // }

    // fprintf(f, "Hello World!\n");
    // fclose(f);

//     ESP_LOGI(TAG, "File written");

     /*尝试读取文件 */
    FILE * f = fopen("/spiflash/pc.txt", "r");
     if (f == NULL) {
         ESP_LOGE(TAG, "该文件不存在");
         return;
     }

     char line[128] = {0};
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    else
    {
        ESP_LOGW(TAG, "文件格式不正确");
    }

    ESP_LOGI(TAG, "文件内容: '%s'", line);

     
    
     while (1)
     {
         //chry_dap_handle();
          vTaskDelay(10 / portTICK_PERIOD_MS);
     }
}
