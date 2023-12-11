
#include "usart.h" // 包含ESP32 UART驱动的头文件

void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_NUM, &uart_config);                             // 初始化UART参数配置
    uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0); // 安装UART驱动
    uart_set_pin(
        UART_NUM, // UART 编号
        6,        // TX GPIO
        5,        // RX GPIO
        -1,       // RTS GPIO
        -1        // CTS GPIO
    );
}

uint8_t checksum(uint8_t *data, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; ++i)
    {
        sum += data[i];
    }
    return sum;
}

void uart_task(void *pvParameters)
{

    //                  // double received_vars[3]; // 定义接收的三个变量
    //                  // uint8_t data[BUF_SIZE]; // 定义数据缓冲区

    //     //uint8_t to_send[5] = {START_BYTE, 0x01, 0x02, 0x03, END_BYTE}; // 发送数据帧
    //     char to_send = 'A';
    //     //to_send[4] = checksum(to_send, 4);                             // 计算校验和

    //     // uint8_t data[BUF_SIZE];
    // printf("test1");
    //     while (1)
    //     {
    //         // 发送数据帧
    //         printf("test2");
    //         uart_write_bytes(UART_NUM,&to_send, sizeof(to_send));

    //         // // 接收数据
    //         // int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, 100 / portTICK_RATE_MS);
    //         // if (len > 0) {
    //         //     // 处理接收到的数据帧
    //         //     if (data[0] == START_BYTE && data[len - 1] == END_BYTE) {
    //         //         // 验证校验和
    //         //         if (checksum(data, len - 1) == data[len - 1]) {
    //         //             // 数据校验成功，处理数据
    //         //             // ...
    //         //              PID_Init(&pid, 100.0, 0.5, 0.0); // 初始化PID参数
    //         //         }
    //         //     }
    //         // }
    //         printf("test3");
    //         vTaskDelay(10); // 任务延时
    //     }

    // vofa的数据帧格式
    //     float t = 0;
    //  while(1) {
    //      t += 0.1;
    //      // 发送数据
    //      float ch[4];
    //      ch[0] = sin(t);
    //      ch[1] = sin(2*t);
    //      ch[2] = sin(3*t);
    //      ch[3] = sin(4*t);
    //       //uart_write_bytes(UART_NUM,ch, sizeof(ch));
    //       uart_write_bytes(UART_NUM,(const char *)ch, 4*sizeof(float));
    //      // 发送帧尾
    //      char tail[4] = {0x00, 0x00, 0x80, 0x7f};
    //      //printf(tail, 4);
    //       uart_write_bytes(UART_NUM,tail, sizeof(tail));
    //     vTaskDelay(100); // 任务延时
    //  }
    //  }
    uart_init(); // 初始化UART

    while (1)
    {
        // 发送数据
        float ch[1];
        ch[0] = temperatures;

        //char received_str[] = "";
        double val1, val2, val3;

        char data[200]; // 定义数据缓冲区
        // 转换成字节流进行传输，传输的是0x00形式，需要接收方再转换成相应的格式。
        uart_write_bytes(UART_NUM, (const char *)ch, sizeof(float));
        char tail[4] = {0x00, 0x00, 0x80, 0x7f}; // 发送帧尾
        uart_write_bytes(UART_NUM, tail, sizeof(tail));

        // 接收数据
        int len = uart_read_bytes(UART_NUM, data, 200, 100 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            int result = sscanf(data, "%lf, %lf, %lf", &val1, &val2, &val3);
            ESP_LOGI("TAG", "p=%f i=%f D= %f len=%d",val1, val2, val3,len); // 输出ADC1通道4的原始数据
            
                // 成功提取了三个 double 值
                // val1, val2, val3 现在包含提取的数值
                PID_Init(&pid, val1, val2, val3); // 初始化PID参数

           
        }

        vTaskDelay(1 / portTICK_PERIOD_MS); // 任务延时
    }
}