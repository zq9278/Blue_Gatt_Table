#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_log.h"
extern uint8_t pwm_duty;



#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (1) // 示例GPIO
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TEST_DUTY         (8192) // 占空比
#define LEDC_TEST_FADE_TIME    (3000)
void pwm_control_task(void *pvParameter);
void set_pwm_duty(uint8_t duty_value);
void ntc_pwm_init();                                        