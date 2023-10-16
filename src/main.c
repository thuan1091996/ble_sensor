/*******************************************************************************
* Title                 :    
* Filename              :   main.c
* Author                :   ItachiVN
* Origin Date           :   2023/10/04
* Version               :   0.0.0
* Compiler              :   nRF connect SDK v2.4
* Target                :   nRF52840 SoC
* Notes                 :   None
*******************************************************************************/


/******************************************************************************
* Includes
*******************************************************************************/
#include <zephyr/kernel.h>

#include "ble\ble_gatt.h"
#include "ble\bluetoothle.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define FW_MAJOR_VERSION                (0)
#define FW_MINOR_VERSION                (0)
#define FW_BUIILD_VERSION               (1)

#define SAMPLING_RATE_MS                (50)
#define SENSOR_PACKET_LEN               (24) // 4B for each axis * 6 axis
#define SENSOR_DATA_HEADER              (0xAA)

/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
#include "zephyr/logging/log.h"
#define MODULE_NAME			        main
#define MODULE_LOG_LEVEL	        LOG_LEVEL_DBG
LOG_MODULE_REGISTER(MODULE_NAME, MODULE_LOG_LEVEL);

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
volatile static bool is_ble_connect=false;

void on_ble_connect(void)
{
    is_ble_connect = true;
    LOG_INF("BLE connected");

}

void on_ble_disconnect(void)
{
    is_ble_connect = false;
    LOG_INF("BLE disconnected");
}
ble_callback_t ble_callback = 
{
    .ble_connected_cb = &on_ble_connect,
    .ble_disconnected_cb = &on_ble_disconnect,
};
/******************************************************************************
* Function Prototypes
*******************************************************************************/
int ble_app_init(void);
/******************************************************************************
* Function Definitions
*******************************************************************************/

/*
 * @brief BLE application init
 * @param None
 * @retval 0: success, -1: failed
 */
int ble_app_init(void)
{
    if(ble_init(&ble_callback) != 0)
    {
        LOG_ERR("BLE init failed");
        return -1;
    }

    ble_custom_service_init(NULL);

    if (ble_adv_start() != 0)
    {
        LOG_ERR("BLE adv start failed");
        return -1;
    }
    return 0;
}


/*
 * @brief: Send sensor data via BLE
 * 
 * @param p_data: pointer to the data to send 
 * @param p_length: pointer to the length of the data to send
 * @param frame_cnt: frame count
 * @return int: 0 on success or negative code otherwise
 */
int sensor_data_send_ble(uint8_t* p_data, uint16_t* p_length, uint32_t frame_cnt)
{
    __ASSERT_NO_MSG(p_data != NULL);
    __ASSERT_NO_MSG(p_length != NULL);
    uint16_t input_len = *p_length;
    uint8_t send_payload[1 + sizeof(frame_cnt) + input_len]; // 1B frame heaader + 4B frame count + 12B sensor data
    memset(send_payload, 0, sizeof(send_payload));
    send_payload[0] = SENSOR_DATA_HEADER;
    memcpy(send_payload + 1, &frame_cnt, sizeof(frame_cnt));
    memcpy(send_payload + 1 +sizeof(frame_cnt), p_data, input_len);
#if (SENSOR_DATA_SEND_BOARDCAST != 0)
    return ble_set_custom_adv_payload(send_payload, 1 + sizeof(frame_cnt) + input_len);
#else /* !(SENSOR_DATA_SEND_BOARDCAST != 0) */
    int send_status = char3_send_indication(send_payload, 1 + sizeof(frame_cnt) + input_len);
    if(send_status != 0)
    {
        LOG_ERR("BLE send failed with status %d", send_status);
        return send_status;
    }
    return 0;
#endif /* End of (SENSOR_DATA_SEND_BOARDCAST != 0) */
    
}

int main(void)
{
    LOG_INF("Firmware version: %d.%d.%d", FW_MAJOR_VERSION, FW_MINOR_VERSION, FW_BUIILD_VERSION);
    k_sleep(K_MSEC(2000));
	if (ble_app_init() != 0)
    {
        LOG_ERR("App BLE init failed");
        return -1;
    }

#if (CONFIG_BOARD_XIAO_BLE == 1) 
    if (sensor_init() != 0)
    {
        LOG_ERR("IMU sensor init failed");
        return -1;
    }
#endif /* End of (CONFIG_BOARD_XIAO_BLE == 1)  */
    while(1)
    {
        if(is_ble_connect == false)
        {
            k_sleep(K_MSEC(1000));
            LOG_WRN("Waiting for BLE connection");
            continue;
        }
        static uint32_t frame_cnt = 0;
        frame_cnt++;
        uint8_t sensor_data[50] = {0};
        uint16_t sensor_data_len = 0;
#if (CONFIG_BOARD_XIAO_BLE == 1) 

        if (sensor_sampling(sensor_data, &sensor_data_len) != 0)
        {
            LOG_ERR("Sensor sampling failed");
        }
        if(sensor_data_len == SENSOR_PACKET_LEN)
        {
            if (sensor_data_send_ble(sensor_data, &sensor_data_len, frame_cnt) != 0)
            {
                LOG_ERR("Sensor data send BLE failed");
            }
            else
            {
                LOG_INF("Frame %d sent", frame_cnt);
                LOG_DBG("Accel (m/s^2): %.02f, %.02f, %.02f \r\n", *(float*)sensor_data, *(float*)(sensor_data + 4), *(float*)(sensor_data + 8));
                LOG_DBG("Gyro (radians/s): %.02f, %.02f, %.02f \r\n", *(float*)(sensor_data + 12), *(float*)(sensor_data + 16), *(float*)(sensor_data + 20));
            }
        }
        else
        {
            LOG_WRN("Sensor data length %d is invalid", sensor_data_len);
        }
#else
        // Simulate sending sensor data
        for(uint8_t count=0; count< sizeof(sensor_data); count++)
        {
            sensor_data[count] = count;
        }
        sensor_data_len = sizeof(sensor_data);
        if (sensor_data_send_ble(sensor_data, &sensor_data_len, frame_cnt) != 0)
        {
            LOG_ERR("Sensor data send BLE failed");
        }
        else
        {
            LOG_INF("Frame %d sent", frame_cnt);
        }
#endif /* End of (CONFIG_BOARD_XIAO_BLE == 1)  */
        k_sleep(K_MSEC(SAMPLING_RATE_MS));
    }
	return 0;
}