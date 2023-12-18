
#include "pid.h"
// PID 控制器结构
 PID_Controller pid;
// 初始化PID控制器
void PID_Init(PID_Controller *pid, double Kp, double Ki, double Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0;
    pid->pre_error = 0.0;
    pid->maxIntegral = 100.0;
    pid->maxOutput = 100.0;
}

// 计算PID控制器的输出
double PID_Compute(PID_Controller *pid, double setpoint, double measured)
{
    double error = setpoint - measured;                                               // 计算误差
    pid->integral += error;                                                           // 积分项
    Limit(pid->integral,-pid->maxIntegral,pid->maxIntegral);
    double derivative = error - pid->pre_error;                                       // 微分项
    pid->pre_error = error;                                                           // 更新误差
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative; // 计算输出
    Limit(output,-pid->maxOutput,pid->maxOutput);
    return output;
}

void pid_control_task(void *pvParameter)
{
   
    PID_Init(&pid, 100.0, 0.05, 0.0); // 初始化PID参数
    ntc_pwm_init();
    // 主控制循环
   while (1)
    {
        // 测量当前温度 (假设有一个函数来测量温度)
        // 计算PID输出
        xSemaphoreTake(temperatureset, portMAX_DELAY);
        pwm_duty = PID_Compute(&pid, setpoint, temperatures);
       // printf("pwm_duty = %d\n", pwm_duty);
        set_pwm_duty(pwm_duty);
        xSemaphoreGive(temperatureset);
        
        // 等待一段时间
        vTaskDelay(pdMS_TO_TICKS(10)); // 假设的延时函数
    }
}
