
#include <stdio.h>
#include "esp_log.h"
#include "mpu6050.h"
#include "math.h"

/* q30 系数*/
#define q30  1073741824.0f
static const char *TAG = "mpu6050-example";

void app_main(void)
{
    unsigned long sensor_timestamp=0;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
    float pitch, roll, yaw;
    /*初始化I2C*/
    ESP_ERROR_CHECK(i2c_master_init());
    /*初始化MPU和MDP*/
    if(MDP_init() == ESP_OK)
        ESP_LOGI(TAG, "I2C and mpu6050 initialized successfully");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while (1)
    {
        /* 读取温度传感器
         * gyro：陀螺仪；accel：加速度：sensor_timestamp：时间戳
         * sensors：判断有那些数据被读出；more：FIFO中是否还有数据
        */
        if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more) == ESP_OK)
        {
            // if(sensors & INV_XYZ_GYRO)
            // {
            //     ESP_LOGI(TAG, "gx = %d\t gy = %d\t gz = %d", gyro[0], gyro[1], gyro[2]);
            // }
            // if (sensors & INV_XYZ_ACCEL)
            // {
            //     ESP_LOGI(TAG, "ax = %d\t ay = %d\t az = %d", accel[0], accel[1], accel[2]);
            // }
            if(sensors & INV_WXYZ_QUAT)
            {
                /*转化为欧拉角*/
                q0 = quat[0] / q30;	
                q1 = quat[1] / q30;
                q2 = quat[2] / q30;
                q3 = quat[3] / q30;
                /*计算得到俯仰角/横滚角/航向角*/
                pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
                roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
                yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw

                ESP_LOGI(TAG, "mputimerstamp = %ld", sensor_timestamp / 1000);
                ESP_LOGI(TAG, "%.2f, %.2f, %.2f,", pitch, roll, yaw);
            }

        }
        // short gxyz[3];
        // mpu_get_gyro_reg(gxyz, &sensor_timestamp);
        // ESP_LOGI(TAG, "%ld", sensor_timestamp);
        
         

        // ESP_ERROR_CHECK(mpu6050_get_temperature(&temp));
        // ESP_LOGI(TAG, "temp = %.2f", temp);
        // ESP_ERROR_CHECK(mpu6050_get_gyroscope(gxyz));
        // ESP_LOGI(TAG, "gx = %d\t gy = %d\t gz = %d", gxyz[0], gxyz[1], gxyz[2]);
        // float axyz[3];
        // ESP_ERROR_CHECK(mpu6050_get_accelerometer(axyz));
        // ESP_LOGI(TAG, "ax = %.2f\t ay = %.2f\t az = %.2f", axyz[0], axyz[1], axyz[2]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    

}
