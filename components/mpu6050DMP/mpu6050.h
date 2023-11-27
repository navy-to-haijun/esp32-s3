#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define I2C_MASTER_SCL_IO           8       // 时钟线
#define I2C_MASTER_SDA_IO           18      // 数据线
#define I2C_MASTER_NUM              0       // I2C master i2c port number
#define I2C_MASTER_FREQ_HZ          400000  // I2C频率                 
#define I2C_MASTER_TX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000    // 超时时间

extern QueueHandle_t mpu6050data_queue;

extern esp_err_t i2c_master_init(void);
/*适配MDP*/
extern esp_err_t esp32s3_i2c_write_bytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
extern esp_err_t esp32s3_i2c_read_bytes(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data);
extern int esp32s3_delay_ms(unsigned long num_ms);
extern int esp32s3_get_clock_ms(unsigned long *count);
extern esp_err_t MDP_init(void);

extern void mpu6050_task(void *pvParameters);
#endif
