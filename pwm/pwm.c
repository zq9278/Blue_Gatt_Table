
#include "pwm.h"

// uint16_t pwm_duty = 0;
//  初始化PWM
void ntc_pwm_init()
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // 分辨率
        .freq_hz = 5000,                      // 频率
        .speed_mode = LEDC_HS_MODE,           // 速度模式
        .timer_num = LEDC_HS_TIMER            // 定时器
    };
    // 设置定时器配置
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_HS_CH0_CHANNEL,
        .duty = 0,
        .gpio_num = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_HS_TIMER};
    // 设置通道配置
    ledc_channel_config(&ledc_channel);
}

// 设置PWM占空比的函数
void set_pwm_duty(uint8_t duty_value)
{
    // 将百分比转换为占空比
    // LEDC_TEST_DUTY通常是根据PWM分辨率来定义的最大值
    uint32_t duty = (LEDC_TEST_DUTY * duty_value) / 100;

    // 设置PWM占空比
    // LEDC_HS_MODE表示高速模式，LEDC_HS_CH0_CHANNEL是要控制的通道
    ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, duty);

    // 更新PWM占空比以应用新的设置
    ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);
}

void pwm_control_task(void *pvParameter)
{
    ntc_pwm_init();

    // 使用 len ...
    // 任务完成后释放内存

    while (1)
    {

        printf("pwm_duty = %d\n", pwm_duty);
        set_pwm_duty(pwm_duty);
        vTaskDelay(10);
    }

    free(pvParameter);
    // 任务完成后，删除任务
    vTaskDelete(NULL);
}
