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
    if(ble_init(NULL) != 0)
    {
        LOG_ERR("BLE init failed");
        return -1;
    }
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
    uint8_t send_payload[sizeof(frame_cnt) + SENSOR_PACKET_LEN + 1] = {0}; // 1B frame heaader + 4B frame count + 12B sensor data
    send_payload[0] = SENSOR_DATA_HEADER;
    memcpy(send_payload + 1, &frame_cnt, sizeof(frame_cnt));
    memcpy(send_payload + 1 +sizeof(frame_cnt), p_data, *p_length);
    return ble_set_custom_adv_payload(send_payload, 1 + sizeof(frame_cnt) + *p_length);
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
#if (CONFIG_BOARD_XIAO_BLE == 1) 
        static uint32_t frame_cnt = 0;
        frame_cnt++;
        uint8_t sensor_data[50] = {0};
        uint16_t sensor_data_len = 0;
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
                printk("Accel (m/s^2): %.02f, %.02f, %.02f \r\n", *(float*)sensor_data, *(float*)(sensor_data + 4), *(float*)(sensor_data + 8));
                printk("Gyro (radians/s): %.02f, %.02f, %.02f \r\n", *(float*)(sensor_data + 12), *(float*)(sensor_data + 16), *(float*)(sensor_data + 20));
            }
        }
        else
        {
            LOG_WRN("Sensor data length %d is invalid", sensor_data_len);
        }
#endif /* End of (CONFIG_BOARD_XIAO_BLE == 1)  */
        k_sleep(K_MSEC(SAMPLING_RATE_MS));
    }
	return 0;
}