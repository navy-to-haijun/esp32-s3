# include "mpu6050.h"

/*陀螺仪方向设定*/
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU_DEFAULT_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief 写数据(单字节)
 */
esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU_DEFAULT_ADDRESS, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

/**
 * @brief 写数据(多字节)
 */
esp_err_t esp32s3_i2c_write_bytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, 1);
    if (ret != ESP_OK)
        return ESP_FAIL;

    ret = i2c_master_write_byte(cmd, reg_addr, 1);
    if (ret != ESP_OK)
        return ESP_FAIL;

    ret = i2c_master_write(cmd, data, length, 1);
    if (ret != ESP_OK)
        return ESP_FAIL;

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
/**
 * @brief 读数据(多字节)
 *      
*/
esp_err_t esp32s3_i2c_read_bytes(uint8_t slave_addr, uint8_t reg_addr,uint8_t length, uint8_t *data)
{
   int ret;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, 1);
	if(ret != ESP_OK)
		return ESP_FAIL;

	ret = i2c_master_write_byte(cmd, reg_addr, 1);
	if(ret != ESP_OK)
		return ESP_FAIL;

	i2c_master_start(cmd);
	ret = i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, 1);
	if(ret != ESP_OK)
		return ESP_FAIL;

	ret = i2c_master_read(cmd, data, length, I2C_MASTER_LAST_NACK);
	if(ret != ESP_OK)
		return ESP_FAIL;

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

    return ret;
}
/**
 *  @brief 延迟函数，适配MDP
*/
int esp32s3_delay_ms(unsigned long num_ms)
{
    vTaskDelay(num_ms / portTICK_PERIOD_MS);
    return 0;
}
/**
 *  @brief 获取时间，适配MDP（空函数，没用）
*/
int esp32s3_get_clock_ms(unsigned long *count)
{
    *count = (unsigned long)xTaskGetTickCount() * 10;
    return 0;
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
/**
 * @brief MDP初始化
*/
void mpu6050_init(void)
{

    // uint8_t data;
    //  /*读取设备ID，判断I2C通信是否正常*/
    // ESP_ERROR_CHECK(esp32s3_i2c_read_bytes(MPU_DEFAULT_ADDRESS, MPU_DEVICE_ID_REG, 1, &data));
    // ESP_LOGI("mpu6050 init", "WHO_AM_I = %X", data);
    // /*重置设备*/
    // data = 0x80;
    // ESP_ERROR_CHECK(esp32s3_i2c_write_bytes(MPU_DEFAULT_ADDRESS, MPU_PWR_MGMT1_REG, 1, &data));
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    // /*禁用睡眠模式，选择X轴陀螺仪为时钟源*/
    // data = 0x01;
    //  ESP_ERROR_CHECK(esp32s3_i2c_write_bytes(MPU_DEFAULT_ADDRESS, MPU_PWR_MGMT1_REG, 1, &data));
    // /*配置数字低通滤波器，带宽为5，陀螺仪输出频率为1kHz*/
    // data = 0x06;
    // ESP_ERROR_CHECK(esp32s3_i2c_write_bytes(MPU_DEFAULT_ADDRESS, MPU_CFG_REG, 1, &data));
    // /*配置采样频率:100Hz*/
    // data = 9;
    // ESP_ERROR_CHECK(esp32s3_i2c_write_bytes(MPU_DEFAULT_ADDRESS, MPU_SAMPLE_RATE_REG, 1, &data));
    /*配置陀螺仪量程:± 2000 °/s, 不自检*/
    // data = 0x18;
    // ESP_ERROR_CHECK(esp32s3_i2c_write_bytes(MPU_DEFAULT_ADDRESS, MPU_GYRO_CFG_REG, 1, &data));
    /*配置加速度计量程: ± 16g， 不自检*/
    // ESP_ERROR_CHECK(esp32s3_i2c_write_bytes(MPU_DEFAULT_ADDRESS, MPU_ACCEL_CFG_REG, 1, &data));
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static inline void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    
    if (result == 0x03) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        ESP_LOGI("DMP init", "self test successfully");
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
    else
    {
        ESP_LOGE("DMP init", "self test error");
    }
}


/**
 * @brief MDP初始化
*/
esp_err_t MDP_init(void)
{
    int  ret;
    /*MPU6050初始化*/
    // mpu6050_init();
    ret = mpu_init(NULL);


    if (ret != 0)
        printf("0, %d\n", ret);

    /*唤醒*/
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (ret != 0)
        printf("1, %d\n", ret);

    /*将加速度和陀螺仪数据放入FIFO中*/
    ret=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (ret != 0)
        printf("2, %d\n", ret);
    /*设置采样频率*/
    mpu_set_sample_rate(100);
    if (ret != 0)
        printf("3, %d\n", ret);
    /*加载内存*/
    dmp_load_motion_driver_firmware();
    if (ret != 0)
        printf("3, %d\n", ret);
    /*设置方向*/
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    if (ret != 0)
        printf("4, %d\n", ret);
    /*启用功能
    * DMP_FEATURE_6X_LP_QUAT:产生四元数
    * DMP_FEATURE_TAP：检测敲击事件
    * DMP_FEATURE_ANDROID_ORIENT：实现了与Google Motion_driver设备兼容的显示方向
    * DMP_FEATURE_SEND_RAW_ACCEL：计步器功能
    * DMP_FEATURE_SEND_CAL_GYRO：设备处于无运动状态超过8秒，就会校准陀螺仪偏置
    * DMP_FEATURE_GYRO_CAL：将原始加速度计数据添加到FIFO
    */
    unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(dmp_features);
    if (ret != 0)
        printf("5, %d\n", ret);
    dmp_set_fifo_rate(100);
    if (ret != 0)
        printf("6, %d\n", ret);
    run_self_test();
    /*启用DMP*/
    ret = mpu_set_dmp_state(1);
    if (ret != 0)
        printf("7, %d\n", ret);

    return ret;

}
