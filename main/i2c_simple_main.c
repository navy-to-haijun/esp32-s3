/* i2c - Simple example

   Simple I2C example that shows how to initialize I2C
   as well as reading and writing from and to registers for a sensor connected over I2C.

   The sensor used in this example is a MPU9250 inertial measurement unit.

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "mpu6050.h"

static const char *TAG = "mpu6050-example";

void app_main(void)
{
    uint8_t data[2];
    float temp = 0;
    float gxyz[3];
    float axyz[3];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    /*读取设备ID，判断I2C通信是否正常*/
    ESP_ERROR_CHECK(mpu6050_register_read(MPU_DEVICE_ID_REG, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
    /*重置设备*/
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU_PWR_MGMT1_REG, 0x80));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    /*禁用睡眠模式，选择X轴陀螺仪为时钟源*/
     ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU_PWR_MGMT1_REG, 0x01));
    /*配置数字低通滤波器，带宽为5，陀螺仪输出频率为1kHz*/
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU_CFG_REG, 0x06));
    /*配置采样频率:50Hz*/
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU_SAMPLE_RATE_REG, 19));
    /*配置陀螺仪量程:± 2000 °/s, 不自检*/
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU_GYRO_CFG_REG, 0x18));
    /*配置加速度计量程: ± 16g， 不自检*/
    ESP_ERROR_CHECK(mpu6050_register_write_byte(MPU_ACCEL_CFG_REG, 0x18));
    while (1)
    {
        /*读取温度传感器*/
        ESP_ERROR_CHECK(mpu6050_get_temperature(&temp));
        ESP_LOGI(TAG, "temp = %.2f", temp);
        ESP_ERROR_CHECK(mpu6050_get_gyroscope(gxyz));
        ESP_LOGI(TAG, "gx = %.2f\t gy = %.2f\t gz = %.2f", gxyz[0], gxyz[1], gxyz[2]);
        ESP_ERROR_CHECK(mpu6050_get_accelerometer(axyz));
         ESP_LOGI(TAG, "ax = %.2f\t ay = %.2f\t az = %.2f", axyz[0], axyz[1], axyz[2]);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    

}
