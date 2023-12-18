#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "pwm.h"


extern double temperatures;//实际温度
extern uint8_t pwm_duty;
extern double setpoint; // 目标温度
extern SemaphoreHandle_t temperatureset;


// my_struct.h
#ifndef PID_Con
#define PID_Con

typedef struct 
{
    double Kp;        // 比例系数
    double Ki;        // 积分系数
    double Kd;        // 微分系数
    double integral;  // 积分项
    double pre_error; // 上一次误差
    float maxIntegral;
    float maxOutput;
} PID_Controller;
#endif // MY_STRUCT_H



void PID_Init(PID_Controller *pid, double Kp, double Ki, double Kd);
void pid_control_task(void *pvParameter);

#define Limit(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))