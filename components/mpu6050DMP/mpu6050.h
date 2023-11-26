#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "eMPL/inv_mpu.h"
#include "eMPL/inv_mpu_dmp_motion_driver.h"

#define I2C_MASTER_SCL_IO           8       // 时钟线
#define I2C_MASTER_SDA_IO           18      // 数据线
#define I2C_MASTER_NUM              0       // I2C master i2c port number
#define I2C_MASTER_FREQ_HZ          400000  // I2C频率                 
#define I2C_MASTER_TX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0       /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000    // 超时时间

// 相关寄存器
#define MPU_ADDRESS_AD0_LOW     0x68 // address pin low (GND)
#define MPU_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU_DEFAULT_ADDRESS     MPU_ADDRESS_AD0_LOW

#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器

#define PI 3.141592

extern esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
extern esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data);
extern esp_err_t i2c_master_init(void);
extern esp_err_t mpu6050_get_temperature(float *temp);
extern esp_err_t mpu6050_get_gyroscope(float *gxyz);
extern esp_err_t mpu6050_get_accelerometer(float *axyz);

/*适配MDP*/
extern esp_err_t esp32s3_i2c_write_bytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);
extern esp_err_t esp32s3_i2c_read_bytes(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data);
extern int esp32s3_delay_ms(unsigned long num_ms);
extern int esp32s3_get_clock_ms(unsigned long *count);
extern esp_err_t MDP_init(void);
#endif
