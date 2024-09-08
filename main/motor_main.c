/**
 * @file        main.c
 * @brief       步进电机控制
 * @details     控制方法：使用1个定时器实现周期事件，在事件中翻转电平实现可控脉冲
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
    
static const char *TAG = "main-task";

#define PLUSE_GPIO  GPIO_NUM_4          // 步进电机脉冲
#define DIR_GPIO    GPIO_NUM_5          // 步进电机方向
#define ENABLE_GPIO  GPIO_NUM_48         // 步进电机使能
/**
 * @brief       定时器周期事件回调函数
 * @details     使用GPIO4产生脉冲信号
 * @param       timer: 定时器句柄
 * @param       edata: 事件数据
 * @param       user_data: 用户数据
 * @return      无
 */
static bool generate_pluse_motor_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    uint16_t exp_pluse = *(uint32_t *)(user_data);
    static uint16_t count;
    count++;
    if(count > 2*exp_pluse)
    {
        count = 0;
        gptimer_stop(timer);
    }
    else{
        gpio_set_level(PLUSE_GPIO, count % 2);
    }
    
    return true;
}
/**
 * @brief       设置脉冲数和转动方向
 */
bool set_pluse_count(gptimer_handle_t timer, uint32_t *exp_pluse, uint32_t set_pluse, int8_t dir)
{
    esp_err_t st;
    st = gptimer_start(timer);
    if(st == ESP_OK)
    {
        *exp_pluse = set_pluse;
        gpio_set_level(DIR_GPIO, dir);
        ESP_LOGI(TAG, "pluse=%ld, dir=%d", set_pluse, dir);
    }
    else{
        ESP_LOGE(TAG, "gptimer_start failed with %s", esp_err_to_name(st));
    }
    return st;
}

void app_main(void)
{
    uint32_t expected_value = 0;                        // 期望脉冲数
    /*配置GPIO: PLUSE_GPIO配置为输出*/
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,               // 初始电平
        .mode = GPIO_MODE_OUTPUT,                    // 输出模式
        .pin_bit_mask = (1ULL << PLUSE_GPIO | 1ULL << DIR_GPIO | 1ULL << ENABLE_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,       // 禁止下拉
        .pull_up_en = GPIO_PULLUP_DISABLE           // 禁止上拉
    };
   gpio_config(&io_conf);

    /*创建定时器*/
    gptimer_handle_t gptimer = NULL;
    /*定时器计数周期1us*/
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,         // APB时钟源
        .direction = GPTIMER_COUNT_UP,              // 向上计数
        .resolution_hz = 1 * 1000 * 1000,           // 计数分辨率（1MHz）
        .intr_priority = 0,                         // 中断优先级:默认
    };
    gptimer_new_timer(&timer_config, &gptimer);

    /*设置周期性动作*/
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 50,                         // 事件周期50us
        .reload_count = 0,                         //  重新加载的计数值为0
        .flags.auto_reload_on_alarm = true         //  自动重载
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);

    /*设置周期性回调函数*/
    gptimer_event_callbacks_t cbs = {
        .on_alarm = generate_pluse_motor_cb
    };
    gptimer_register_event_callbacks(gptimer, &cbs, &expected_value);
    /*使能定时器*/
    gptimer_enable(gptimer);
    /*启动定时器*/
    // gptimer_start(gptimer);

    gpio_set_level(ENABLE_GPIO, 1);
    
   while (1)
   {
        set_pluse_count(gptimer, &expected_value, 1600, 1);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        set_pluse_count(gptimer, &expected_value, 1600, 0);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
   }
}
