# include "mpu6050.h"

/*陀螺仪方向设定*/
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};


/**
 * @brief 写数据
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
 * @brief 读数据
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
 *  @brief 获取时间，适配MDP
*/
int esp32s3_get_clock_ms(unsigned long *count)
{
    *count = xTaskGetTickCount() * 10;
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
    ret = mpu_init(NULL);

    if (ret != 0)
        printf("0, %d\n", ret);
    /*唤醒*/
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /*将加速度和陀螺仪数据放入FIFO中*/
    ret=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    /*设置采样频率*/
    mpu_set_sample_rate(100);
    /*加载内存*/
    dmp_load_motion_driver_firmware();
    /*设置方向*/
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
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

/* q30 系数*/
#define q30  1073741824.0f
static const char *TAG = "mpu6050-task";


/**
 *@brief mpu6050 task
 * 将数据放在队列中
*/
void mpu6050_task(void *pvParameters)
{
    unsigned long sensor_timestamp=0;
    short gyro[3], accel[3], sensors;
    unsigned char more;
    long quat[4];
    float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
    float pitch, roll, yaw;
    int16_t mpu6050_data[3];
    BaseType_t xStatus;
    /*初始化I2C*/
    ESP_ERROR_CHECK(i2c_master_init());
    /*初始化MPU和MDP*/
    if(MDP_init() == ESP_OK)
        ESP_LOGI(TAG, "I2C and mpu6050 initialized successfully");
    vTaskDelay(50 / portTICK_PERIOD_MS);
    
    while (1)
    {
        /*
         * gyro：陀螺仪；accel：加速度：sensor_timestamp：时间戳
         * sensors：判断有那些数据被读出；more：FIFO中是否还有数据
        */
        if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more) == ESP_OK)
        {
            if(sensors & INV_WXYZ_QUAT)
            {
                /*转化为欧拉角*/
                q0 = quat[0] / q30;	
                q1 = quat[1] / q30;
                q2 = quat[2] / q30;
                q3 = quat[3] / q30;

                pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
                roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
                yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
                /*发送数据*/
                mpu6050_data[0] = (int16_t)(roll * 100);
                mpu6050_data[1] = (int16_t)(pitch * 100);
                mpu6050_data[2] = (int16_t)(yaw * 100);
                xStatus = xQueueSend(mpu6050data_queue, mpu6050_data, 0);
                if (xStatus != pdPASS)
                {
                    ESP_LOGE(TAG, "Could not send to the queue.");
                }
            
                // ESP_LOGI(TAG, "mputimerstamp = %ld", sensor_timestamp);
                // ESP_LOGI(TAG, "%.2f, %.2f, %.2f,", pitch, roll, yaw);
            }

        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
