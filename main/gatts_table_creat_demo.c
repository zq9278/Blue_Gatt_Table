/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
 *
 * This demo showcases creating a GATT database using a predefined attribute table.
 * It acts as a GATT server and can send adv data, be connected by client.
 * Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
 * Client demo will enable GATT server's notify after connection. The two devices will then exchange
 * data.
 *
 ****************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"
#include "ntc.h"
#include "pwm.h"
#include "pid.h"
#include "usart.h"

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define SAMPLE_DEVICE_NAME "zhangqi"
#define SVC_INST_ID 0  // 服务实例
#define SVC_INST_ID1 1 // 服务实例

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
 *  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
 */
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;
static TaskHandle_t pwmhandle;
static TaskHandle_t ntchandle;

uint16_t heart_rate_handle_table[HRS_IDX_NB];
// 存储心率服务在 GATT 服务器中的每个属性的句柄（handles）
uint16_t heart_rate_handle_table2[HRS_IDX_NB2];

// 全局变量来存储温度
char latestTemperature[12];
double setpoint = 50; // 目标温度
double temperatures;  // 实际温度
// 互斥量用于同步对全局变量的访问
SemaphoreHandle_t temperatureMutex;
SemaphoreHandle_t temperatureset;
uint8_t pwm_duty;

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power*/
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00,
    /* device name */
    0x0f, 0x09, 'Z', 'H', 'A', 'N', 'G', 'Q', 'I', 'B', 'L', 'U', 'D', 'E', 'M', 'O'};
static uint8_t raw_scan_rsp_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power */
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, // test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TEST_A = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TEST_B = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_TEST_C = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_TEST_D = 0xFF04;

static const uint16_t GATTS_SERVICE_UUID_TEST2 = 0x00EE;
static const uint16_t GATTS_CHAR_UUID_TEST_A2 = 0xEE01;
static const uint16_t GATTS_CHAR_UUID_TEST_B2 = 0xEE02;
static const uint16_t GATTS_CHAR_UUID_TEST_C2 = 0xEE03;
static const uint16_t GATTS_CHAR_UUID_TEST_D2 = 0xEE04;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2] = {0x00, 0x00};
static const uint8_t char_value[4] = {0x11, 0x22, 0x33, 0x44};

/* Full Database Description - Used to add attributes into the database
ESP_GATT_AUTO_RSP: 指示框架应自动响应GATT请求。
ESP_UUID_LEN_16: 指示使用16位的UUID。
(uint8_t *)&primary_service_uuid: 指向服务的UUID。
ESP_GATT_PERM_READ: 读权限。
ESP_GATT_PERM_WRITE: 写权限。
sizeof(uint16_t): 属性的长度。
sizeof(GATTS_SERVICE_UUID_TEST): UUID的长度。
(uint8_t *)&GATTS_SERVICE_UUID_TEST: 指向UUID数据的指针。
CHAR_DECLARATION_SIZE: 特性声明的大小。
(uint8_t *)&char_prop_read_write_notify: 指示该特性的属性（如可读写，支持通知等）。
GATTS_DEMO_CHAR_VAL_LEN_MAX: 特性值的最大长度。
(uint8_t *)char_value: 特性的默认值。
(uint8_t *)&character_client_config_uuid: 指向客户端配置特性描述符的UUID。
sizeof(heart_measurement_ccc): 客户端配置描述符的长度。
(uint8_t *)heart_measurement_ccc: 客户端配置描述符的默认值。
 */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
    {
        // 服务声明
        [IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},
        /* 特性声明 A */
        [IDX_CHAR_A] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
        /* 特性值 A */
        [IDX_CHAR_VAL_A] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
        /* 客户端特性配置描述符 A */
        [IDX_CHAR_CFG_A] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},
        /* 特性声明 B */
        [IDX_CHAR_B] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}}, // 最后一个参数才是设置读写权限的
        /* 特性值 B */
        [IDX_CHAR_VAL_B] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
        /* 特性声明 C */
        [IDX_CHAR_C] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},
        /* 特性值 C */
        [IDX_CHAR_VAL_C] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
        /* 特性声明 D */
        [IDX_CHAR_D] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},
        /* 特性值 D */
        [IDX_CHAR_VAL_D] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_D, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
};

static const esp_gatts_attr_db_t gatt_db2[HRS_IDX_NB2] =
    {
        // 服务声明
        [IDX_SVC2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST2), (uint8_t *)&GATTS_SERVICE_UUID_TEST2}},
        /* 特性声明 A */
        [IDX_CHAR_A2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},
        /* 特性值 A */
        [IDX_CHAR_VAL_A2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
        /* 客户端特性配置描述符 A */
        [IDX_CHAR_CFG_A2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},
        /* 特性声明 B */
        [IDX_CHAR_B2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},
        /* 特性值 B */
        [IDX_CHAR_VAL_B2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
        /* 特性声明 C */
        [IDX_CHAR_C2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},
        /* 特性值 C */
        [IDX_CHAR_VAL_C2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
        /* 特性声明 D */
        [IDX_CHAR_D2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},
        /* 特性值 D */
        [IDX_CHAR_VAL_D2] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_D2, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},
};
/*
这段代码主要负责处理与广告有关的事件，如广告数据设置完成、广告开始/停止等，
以及连接参数更新的事件。根据事件类型的不同，代码会执行不同的操作，
例如在广告数据设置完成后开始广告，或者在广告开始/停止事件发生时打印相关日志
*/
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    // 根据传入的事件类型执行相应的操作
    switch (event)
    {
#ifdef CONFIG_SET_RAW_ADV_DATA
    // 如果定义了CONFIG_SET_RAW_ADV_DATA，则处理原始广播数据设置完成事件
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        // 清除广告配置完成标志
        adv_config_done &= (~ADV_CONFIG_FLAG);
        // 如果所有的配置都完成了，开始广告
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    // 处理原始扫描响应数据设置完成事件
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        // 清除扫描响应配置完成标志
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        // 如果所有的配置都完成了，开始广告
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    // 如果没有定义CONFIG_SET_RAW_ADV_DATA，则处理广告数据设置完成事件
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        // 清除广告配置完成标志
        adv_config_done &= (~ADV_CONFIG_FLAG);
        // 如果所有的配置都完成了，开始广告
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    // 处理扫描响应数据设置完成事件
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        // 清除扫描响应配置完成标志
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        // 如果所有的配置都完成了，开始广告
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    // 处理广告开始完成事件，以指示广告是成功开始还是失败
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            // 如果广告开始失败，打印错误日志
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
        }
        else
        {
            // 如果广告开始成功，打印信息日志
            ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
        }
        break;
    // 处理广告停止完成事件
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            // 如果广告停止失败，打印错误日志
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
        }
        else
        {
            // 如果广告停止成功，打印信息日志
            ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
        }
        break;
    // 处理连接参数更新事件
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        // 打印连接参数更新的状态和参数
        ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        // 对于其他事件不做处理
        break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL)
    {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    else
    {
        if (param->write.offset > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_OFFSET;
        }
        else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp)
    {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL)
        {
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK)
    {
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param)
{
    // 根据事件类型使用switch语句来处理不同事件
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    { // 当注册事件发生时
        // 设置设备名称
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        if (set_dev_name_ret)
        {
            // 如果设置设备名称失败，打印错误代码
            ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        // 配置原始广播数据
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret)
        {
            // 如果配置失败，打印错误代码
            ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        // 设置广播配置完成标志
        adv_config_done |= ADV_CONFIG_FLAG;
        // 配置原始扫描响应数据
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret)
        {
            // 如果配置失败，打印错误代码
            ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        // 设置扫描响应配置完成标志
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
        // 配置广播数据
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            // 如果配置失败，打印错误代码
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        // 设置广播配置完成标志
        adv_config_done |= ADV_CONFIG_FLAG;
        // 配置扫描响应数据
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            // 如果配置失败，打印错误代码
            ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
        }
        // 设置扫描响应配置完成标志
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
        // 创建属性表
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
        esp_err_t create_attr_ret2 = esp_ble_gatts_create_attr_tab(gatt_db2, gatts_if, HRS_IDX_NB2, SVC_INST_ID1);
        // esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, 3);
        if (create_attr_ret)
        {
            // 如果创建属性表失败，打印错误代码
            ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
        }
        if (create_attr_ret2)
        {
            // 如果创建属性表失败，打印错误代码
            ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret2);
        }
    }
    break;
    case ESP_GATTS_READ_EVT: // 当读取事件发生时
        // 打印信息，表示读取事件发生
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");

        if (heart_rate_handle_table[IDX_CHAR_VAL_B] == param->read.handle)
        {
            xSemaphoreTake(temperatureMutex, portMAX_DELAY);
            esp_ble_gatts_set_attr_value(heart_rate_handle_table[IDX_CHAR_VAL_B], sizeof(latestTemperature),
                                         (uint8_t *)latestTemperature);
            xSemaphoreGive(temperatureMutex);
        }

        break;
    case ESP_GATTS_WRITE_EVT: // 当写入事件发生时
        if (!param->write.is_prep)
        {
            // 如果不是准备写入操作
            // 打印写入的句柄和值的长度和内容
            ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
            // 创建内存空间并存储数据指针
            if (param->write.handle == 49)
            {
                // uint8_t *len_ptr = malloc(sizeof(uint8_t))*2;
                // if (len_ptr != NULL)
                // {
                //     *len_ptr = *(param->write.value); // 复制值，而不是更改指针
                //     // len_ptr = param->write.value;// 复制值，而不是更改指针
                //     ESP_LOGI(GATTS_TABLE_TAG, "double:%c",*(param->write.value));
                //     ESP_LOGI(GATTS_TABLE_TAG, "double:%c",*((param->write.value)+1));
                // }
                // //xSemaphoreTake(temperatureset, portMAX_DELAY);
                // double setpoint = *(double *)(len_ptr);
                // //memcpy(&setpoint,*(uint8_t*)param->write.value, sizeof(double));
                // xSemaphoreGive(temperatureset);
                char combinedStr[]="1";
                char *endPtr;
                for (int i = 0; i < param->write.len; i++)
                {
                    combinedStr[i] = *(param->write.value);
                    param->write.value++;
                }
                ESP_LOGI(GATTS_TABLE_TAG, "double:%s",combinedStr);
                xSemaphoreTake(temperatureset, portMAX_DELAY);
                setpoint = strtod(combinedStr, &endPtr);
                xSemaphoreGive(temperatureset);
                ESP_LOGI(GATTS_TABLE_TAG, "double:%f",setpoint);
            }

            // esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
            // ESP_LOGI(GATTS_TABLE_TAG, "double:%f",setpoint);
            //   如果是特定的描述符被写入，根据写入的值启用通知或指示
            if (heart_rate_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001)
                {
                    // 如果是启用通知
                    ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                    // 设置通知数据
                    uint8_t notify_data[15];
                    for (int i = 0; i < sizeof(notify_data); ++i)
                    {
                        notify_data[i] = i % 0xff;
                    }
                    // 发送通知
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(notify_data), notify_data, false);
                }
                else if (descr_value == 0x0002)
                {
                    // 如果是启用指示
                    ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                    // 设置指示数据
                    uint8_t indicate_data[15];
                    for (int i = 0; i < sizeof(indicate_data); ++i)
                    {
                        indicate_data[i] = i % 0xff;
                    }
                    // 发送指示
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(indicate_data), indicate_data, true);
                }
                else if (descr_value == 0x0000)
                {
                    // 如果是禁用通知/指示
                    ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                }
                else
                {
                    // 如果是未知的描述符值
                    ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                    // 打印未知的值
                    esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                }
            }

            if (heart_rate_handle_table2[IDX_CHAR_CFG_A2] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001)
                {
                    // 如果是启用通知
                    ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                    // 设置通知数据
                    uint8_t notify_data[15];
                    for (int i = 0; i < sizeof(notify_data); ++i)
                    {
                        notify_data[i] = i % 0xAA;
                    }
                    // 发送通知
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table2[IDX_CHAR_VAL_A2],
                                                sizeof(notify_data), notify_data, false);
                }
                else if (descr_value == 0x0002)
                {
                    // 如果是启用指示
                    ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                    // 设置指示数据
                    uint8_t indicate_data[15];
                    for (int i = 0; i < sizeof(indicate_data); ++i)
                    {
                        indicate_data[i] = i % 0xAA;
                    }
                    // 发送指示
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table2[IDX_CHAR_VAL_A2],
                                                sizeof(indicate_data), indicate_data, true);
                }
                else if (descr_value == 0x0000)
                {
                    // 如果是禁用通知/指示
                    ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                }
                else
                {
                    // 如果是未知的描述符值
                    ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                    // 打印未知的值
                    esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                }
            }

            // 如果需要响应，发送响应
            if (param->write.need_rsp)
            {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        else
        {
            // 如果是准备写入操作，处理预写环境
            example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
        }
        break;
    case ESP_GATTS_EXEC_WRITE_EVT: // 当执行写入操作事件发生时
        // 执行预写操作
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        example_exec_write_event_env(&prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT: // 当MTU事件发生时
        // 打印MTU大小
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_CONF_EVT: // 当确认事件发生时
        // 打印状态和属性句柄
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    case ESP_GATTS_START_EVT: // 当启动事件发生时
        // 打印状态和服务句柄
        ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;

    case ESP_GATTS_CONNECT_EVT: // 当连接事件发生时
        // 打印连接ID和远程设备地址
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
        // 设置连接参数
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        // 设置连接间隔和超时
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        // 更新连接参数
        esp_ble_gap_update_conn_params(&conn_params);

        break;
    case ESP_GATTS_DISCONNECT_EVT: // 当断开连接事件发生时
        // 打印断开原因
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        // 开始广告
        esp_ble_gap_start_advertising(&adv_params);
        // 关闭pwm任务
        vTaskDelete(pwmhandle);

        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT: // 创建表事件发生时
    {

        // 判断创建的属性表是哪个服务的
        if (param->add_attr_tab.svc_uuid.uuid.uuid16 == GATTS_SERVICE_UUID_TEST)
        {
            // 检查状态是否为ESP_GATT_OK
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                // 创建属性表失败
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB)
            {
                // 创建属性表异常，句柄数目不匹配
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, HRS_IDX_NB);
            }
            else
            {
                // 创建属性表成功
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
                // 复制属性句柄到全局变量
                memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
                // 启动服务
                esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
            }
        }
        else if (param->add_attr_tab.svc_uuid.uuid.uuid16 == GATTS_SERVICE_UUID_TEST2)
        {
            // 检查状态是否为ESP_GATT_OK
            if (param->add_attr_tab.status != ESP_GATT_OK)
            {
                // 创建属性表失败
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != HRS_IDX_NB2) // 新添加的
            {
                // 创建属性表异常，句柄数目不匹配
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
                         param->add_attr_tab.num_handle, HRS_IDX_NB2); // 新添加的
            }
            else
            {
                // 创建属性表成功
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
                // 复制属性句柄到全局变量
                memcpy(heart_rate_handle_table2, param->add_attr_tab.handles, sizeof(heart_rate_handle_table2)); // 新添加的
                // 启动服务
                esp_ble_gatts_start_service(heart_rate_handle_table2[IDX_SVC2]); // 新添加的
            }
        }

        break;
    }
    // 当GATT服务停止事件发生
    case ESP_GATTS_STOP_EVT:
        // 服务停止事件
        break;
    // 当GATT服务打开事件发生
    case ESP_GATTS_OPEN_EVT:
        // 服务打开事件
        break;
    // 当取消GATT服务打开事件发生
    case ESP_GATTS_CANCEL_OPEN_EVT:
        // 取消服务打开事件
        break;
    // 当GATT服务关闭事件发生
    case ESP_GATTS_CLOSE_EVT:
        // 服务关闭事件
        break;
    // 当GATT服务监听事件发生
    case ESP_GATTS_LISTEN_EVT:
        // 服务监听事件
        break;
    // 当GATT服务拥塞事件发生
    case ESP_GATTS_CONGEST_EVT:
        // 服务拥塞事件
        break;
    // 当GATT服务注销事件发生
    case ESP_GATTS_UNREG_EVT:
        // 服务注销事件
        break;
    // 当删除GATT服务事件发生
    case ESP_GATTS_DELETE_EVT:
        // 服务删除事件
        break;
    // 其他默认事件
    default:
        break;
    }
}

/*
这个函数是用来处理不同的蓝牙GATT服务器事件的，比如注册事件、连接事件等。
这里只展示了对注册事件的处理，如果注册成功，则为对应的服务设置GATT接口，
否则打印错误信息。然后，循环调用每个配置文件注册的回调函数来处理事件
*/

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* 如果事件是注册事件，则为每个配置文件存储gatts_if（GATT接口） */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            /* 如果注册状态为OK，则为相应的配置文件索引设置GATT接口 */
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            /* 如果应用程序注册失败，则打印错误信息 */
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }



    /* 通过循环来处理所有的GATT事件 */
    do
    {
        int idx;
        /* 遍历所有的配置文件 */
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            /* 如果gatts_if是ESP_GATT_IF_NONE，表示不指定某个特定的gatts_if，需要调用每个配置文件的回调函数 */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if)
            {
                /* 如果配置文件有设置回调函数，则调用之 */
                if (heart_rate_profile_tab[idx].gatts_cb)
                {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0); /* 使用do...while(0)在宏定义中安全地包装多条语句，这里只执行一次 */
}

void app_main(void)
{
    esp_err_t ret;
    /* Initialize NVS.初始化非易失性存储系统(NVS)。*/
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        /* 如果没有足够的空闲页面，或者发现了NVS的新版本，则擦除NVS以重新初始化。 */
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    /* 检查NVS初始化的返回状态。 */
    ESP_ERROR_CHECK(ret);

    /* 释放蓝牙控制器内存，如果不需要经典蓝牙的话。 */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    /* 蓝牙控制器默认配置 */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        /* 如果蓝牙控制器初始化失败，则打印错误日志。 */
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* 启用蓝牙控制器，设置为BLE模式。 */
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        /* 如果蓝牙控制器启用失败，则打印错误日志。 */
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* 初始化蓝牙协议栈。 */
    ret = esp_bluedroid_init();
    if (ret)
    {
        /* 如果蓝牙协议栈初始化失败，则打印错误日志。 */
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* 启用蓝牙协议栈。 */
    ret = esp_bluedroid_enable();
    if (ret)
    {
        /* 如果蓝牙协议栈启用失败，则打印错误日志。 */
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    /* 注册GATT服务事件处理回调函数。当GATT（通用属性）相关的事件发生时，使用这个函数来处理事件。这些事件可能包括服务注册、连接状态改变、接收到数据等。 */
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    /*
    当您在 BLE 应用程序中调用 esp_ble_gatts_register_callback 函数注册 gatts_event_handler 作为事件回调时，
    您告诉 BLE 栈在发生 GATT 相关事件时调用这个函数，并传递适当的参数。这是 BLE 栈的内部机制自动处理的，
    开发者只需关注如何根据这些参数在回调函数中编写相应的逻辑。
    */
    if (ret)
    {
        /* 如果GATT服务事件注册失败，则打印错误日志。 */
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    /* 注册GAP事件处理回调函数。 */
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        /* 如果GAP事件注册失败，则打印错误日志。 */
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    /* 注册GATT应用程序ID。*/
    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    /*
    BLE（Bluetooth Low Energy）中的 GATT（Generic Attribute Profile）结构可以通过以下层次结构图表来描述清楚它们之间的包含关系：
    GATT 应用程序
    +-- GATT 服务 1
    |   |
    |   +-- 特性 1
    |   |   |
    |   |   +-- 特性值
    |   |   |
    |   |   +-- 描述符 1
    |   |   |
    |   |   +-- 描述符 2
    |   |
    |   +-- 特性 2
    |       |
    |       +-- 特性值
    +-- GATT 服务 2
        |
        +-- 特性 1
            |
            +-- 特性值
            |
            +-- 描述符 1
    - **GATT 应用程序**：这是最顶层，代表整个 BLE 应用程序。它可以包含多个 GATT 服务。
    - **GATT 服务**：每个服务代表一组相关的功能，如心率监测或电池状态。服务包含一个或多个特性。
    - **特性**：特性是服务的核心，包含实际的数据（特性值）和描述这些数据的元数据（特性声明）。一个服务可以有多个特性。
    - **特性值**：这是特性的实际数据内容，例如心率数值或电池电量。
    - **描述符**：描述符是可选的，为特性提供额外的信息，例如数据格式或用户描述。一个特性可以有零个或多个描述符。
    这个层次结构表明了 GATT 应用程序如何通过一系列嵌套的服务和特性来组织数据，以及如何通过描述符为这些特性提供附加信息。
    */
    if (ret)
    {
        /* 如果GATT应用程序注册失败，则打印错误日志。 */
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    /* 设置蓝牙最大传输单元(MTU)大小。 */
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        /* 如果设置MTU大小失败，则打印错误日志。 */
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    temperatureMutex = xSemaphoreCreateMutex();
    temperatureset = xSemaphoreCreateMutex();

    xTaskCreate(ntc_read_task, "ntc_read_task", 2048, NULL, 5, &ntchandle);
    xTaskCreate(pwm_control_task, "pwm_control_task", 2048, NULL, 5, &pwmhandle);
    esp_ble_gatts_set_attr_value(heart_rate_handle_table[IDX_CHAR_VAL_B], sizeof(latestTemperature),
                                 (uint8_t *)latestTemperature);
    xTaskCreate(pid_control_task, "pid_control_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 5, NULL); // 创建UART任务
}
