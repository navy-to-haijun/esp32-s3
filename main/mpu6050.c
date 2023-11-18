# include "mpu6050.h"
/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU_DEFAULT_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU_DEFAULT_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
/**
* @brief 读取温度值
*/
esp_err_t mpu6050_get_temperature(float *temp)
{
    int ret;
    uint8_t data[2];
    int16_t raw;
    ret = mpu6050_register_read(MPU_TEMP_OUTH_REG, data, 2);
    raw = (int16_t)((data[0] << 8) + data[1]);
    *temp = 36.53 + raw/340.0;
    return ret;
}
/**
 * @brief 读取陀螺仪值
*/
esp_err_t mpu6050_get_gyroscope(float *gxyz)
{
    int ret;
    uint8_t data[6];
    int16_t raw_gxyz[3];
    float LSB[4] = {131.0, 65.5, 32.8, 16.4};
    /*读取原始数据*/
    ret = mpu6050_register_read(MPU_GYRO_XOUTH_REG, data, 6);
    if (ret == ESP_OK)
    {
        /*转化原始数据*/
        raw_gxyz[0] = (int16_t)((data[0] << 8) + data[1]);
        raw_gxyz[1] = (int16_t)((data[2] << 8) + data[3]);
        raw_gxyz[2] = (int16_t)((data[4] << 8) + data[5]);
    }
    /*读取量程*/
    ret = mpu6050_register_read(MPU_GYRO_CFG_REG, data, 1);
        
    if (ret == ESP_OK)
    {
        uint8_t a = (data[0] & 0X18) >> 3;
        // 实际值
        for (uint8_t i = 0; i < 3; i++)
        {
            gxyz[i] =raw_gxyz[i] / LSB[a];
        }
    }
    
    return ret; 
}
/**
 * @brief 读取加速度值
*/
esp_err_t mpu6050_get_accelerometer(float *axyz)
{
    int ret;
    uint8_t data[6];
    int16_t raw_axyz[3];
    float LSB[4] = {16384.0, 8192.0, 4096.0, 2048.0};
    /*原始值*/
    ret = mpu6050_register_read(MPU_ACCEL_XOUTH_REG, data, 6);
    if (ret == ESP_OK)
    {
        raw_axyz[0] = (int16_t)((data[0] << 8) + data[1]);
        raw_axyz[1] = (int16_t)((data[2] << 8) + data[3]);
        raw_axyz[2] = (int16_t)((data[4] << 8) + data[5]);
    }
    /*读取量程*/
    ret = mpu6050_register_read(MPU_ACCEL_CFG_REG, data, 1);
    if (ret == ESP_OK)
    {
        uint8_t a = (data[0] & 0X18) >> 3 ;
        /*转化为实际值*/
        for (uint8_t i = 0; i < 3; i++)
        {
            axyz[i] =raw_axyz[i] / LSB[a];
        }
        
    }
    return ret; 
}