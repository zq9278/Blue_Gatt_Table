#include <stdio.h>                  // 包含标准输入输出头文件
#include "freertos/FreeRTOS.h"      // 包含FreeRTOS的头文件
#include "freertos/task.h"          // 包含FreeRTOS任务管理的头文件
#include "driver/uart.h"            // 包含ESP32 UART驱动的头文件
#include "pid.h"
#include "math.h"
#include <math.h>
#include "esp_log.h"
#define UART_NUM UART_NUM_1         // 定义使用的UART编号
#define BUF_SIZE (1024)             // 定义缓冲区大小

// 数据帧的起始和结束标识
#define START_BYTE 0xAA
#define END_BYTE 0xBB


extern double temperatures;
extern PID_Controller pid;
void uart_init();
void uart_task(void *pvParameters);
uint8_t checksum(uint8_t *data, int len);
