# include "udp_client.h"
# include "mpu6050.h"

static const char *TAG = "UDP client task";
#define HOST_IP_ADDR  "192.168.6.2"
#define PORT 8888

/**
 * 发送给匿名上位机（欧拉角）
*/
void tranportdata(int16_t *esp23data, uint8_t *pcdata)
{
    pcdata[11] = 0;
    pcdata[12] = 0;
    pcdata[0] = 0XAA;
    pcdata[1] = 0XFF;
    pcdata[2] = 0X03;
    pcdata[3] = 7;
    for (uint8_t i = 0; i < 3; i++)
    {
        pcdata[4 + 2*i] = esp23data[i] & 0x00ff;
        pcdata[4 + 2*i + 1] = esp23data[i] >> 8;
    }
    /*融合状态*/
    pcdata[10] = 1;  
    /*校验位*/
    for (uint8_t i = 0; i < pcdata[3]+4; i++)
    {
        pcdata[11] += pcdata[i];
        pcdata[12] +=  pcdata[11];
    }
}


void udp_client_task(void *pvParameters)
{
    int addr_family = 0;
    int ip_protocol = 0;
    BaseType_t xStatus;
    int16_t mpu6050_data[4];
    uint8_t euler_esp32_to_pc[13];   

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

        while(1)
        {
            xStatus = xQueueReceive(mpu6050data_queue, mpu6050_data, xTicksToWait);
            if(xStatus == pdPASS)
            {
                /*封装数据*/
                tranportdata(mpu6050_data, euler_esp32_to_pc);
                // printf("%d\n", mpu6050_data[1]);
                /*发送*/
                int err = sendto(sock, euler_esp32_to_pc, 13, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
            // printf("Error \n");

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    

   
    

}