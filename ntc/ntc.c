
#include "ntc.h"
extern char latestTemperature[12];
extern double temperatures;
double R0 = 100000.0; // 100kΩ at 25°C
double T0 = 298.15;   // 25°C in Kelvin
double B = 4250.0;    // B constant for 25/50℃ range
extern SemaphoreHandle_t temperatureMutex;
void ntc_read_task(void *pvParameter) // 主函数
{
    adc_oneshot_unit_handle_t adc1_handle; // 定义一个ADC单次模式单元句柄
    adc_oneshot_unit_init_cfg_t init_config1 = {
        // 定义一个ADC单次模式单元初始化配置结构体
        .unit_id = ADC_UNIT_1, // 设置ADC单元为ADC_UNIT_1
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle)); // 创建一个新的ADC单次模式单元

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        // 定义一个ADC单次模式通道配置结构体
        .bitwidth = ADC_BITWIDTH_DEFAULT, // 设置位宽为默认值
        .atten = EXAMPLE_ADC_ATTEN,       // 设置衰减为示例值
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, EXAMPLE_ADC1_CHAN4, &config)); // 配置ADC1的通道4

    while (1)
    {                                                                                 // 无限循环
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN4, &adc_raw)); // 读取ADC1通道4的数据
                                                                                      // 获取互斥量并更新温度
        xSemaphoreTake(temperatureMutex, portMAX_DELAY);
        double resistance = (adc_raw / 4095.0) * 3.3;
        printf("adc_raw:%d",adc_raw);
        resistance = (resistance * 100000) / (3.3 - resistance);
        printf("resistance:%f",resistance);
        temperatures = (1.0 / (1.0 / T0 + log(resistance / R0) / B)) - 273.15;
        printf("temperatures:%f\n",temperatures);
        sprintf(latestTemperature, "%f", temperatures);

        xSemaphoreGive(temperatureMutex);

        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %f", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN4, temperatures); // 输出ADC1通道4的原始数据
        vTaskDelay(pdMS_TO_TICKS(100));                                                              // 延时1000毫秒
    }
}
