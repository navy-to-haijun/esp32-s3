# include "udp_client.h"
# include "mpu6050.h"

static const char *TAG = "UDP client task";
#define HOST_IP_ADDR  "192.168.7.7"
#define PORT 8888

#define ESP_WIFI_SSID      "CU_7daysinn301"
#define ESP_WIFI_PASS      "77777777"
#define ESP_MAXIMUM_RETRY  5

/* FreeRTOS event group to signal when we are connected*/
EventGroupHandle_t s_wifi_event_group;

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    /*sta状态*/
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    /*wifi 断开*/
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    /*获取到IP*/
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
/**
 * @brief wifi station 初始化
*/
void wifi_init_sta(void)
{

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

void udp_client_task(void *pvParameters)
{
    int addr_family = 0;
    int ip_protocol = 0;
    BaseType_t xStatus;

    float_to_u8 senddata[3];

    /*超时时间:20ms*/
    const TickType_t xTicksToWait = pdMS_TO_TICKS(20);

    while(1)
    {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        }

         // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
        vTaskDelay(50 / portTICK_PERIOD_MS);

        float_to_u8 euler_mpu[3];
        /*初值*/
        euler_mpu[0].fdata = 0.0001;
        euler_mpu[1].fdata = 1.1234;
        euler_mpu[2].fdata = 123.123;


        while(1)
        {
            /*发送*/
            sendto(sock, euler_mpu, sizeof(euler_mpu), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            /*修改值*/
            euler_mpu[0].fdata += 0.0001;
            euler_mpu[1].fdata += 0.1;
            euler_mpu[2].fdata += 100;
        //     xStatus = xQueueReceive(mpu6050data_queue, senddata, xTicksToWait);
        //     if(xStatus == pdPASS)
        //     {
        //         /*发送*/
        //         int err = sendto(sock, senddata, sizeof(senddata), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        //         if (err < 0)
        //         {
        //             ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        //             break;
        //         }
        //     }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}