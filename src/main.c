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
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>

#include "ble\ble_gatt.h"
#include "ble\bluetoothle.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#define SAMPLING_RATE_MS        (50)

#define CONF_TEST_COUNTER       (1) // 1: test counter by toggle IO in counter callback, 0: not test counter
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
#include "zephyr/logging/log.h"
#define MODULE_NAME			        main
#define MODULE_LOG_LEVEL	        LOG_LEVEL_DBG
LOG_MODULE_REGISTER(MODULE_NAME, MODULE_LOG_LEVEL);

#define SAMPLE_COUNTER_DEVICE    DT_NODELABEL(timer2)
/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static const struct device *gpio0_device = DEVICE_DT_GET(DT_NODELABEL(gpio0));

/******************************************************************************
* Function Prototypes
*******************************************************************************/
int ble_app_init(void);
int sampling_counter_init(void);
/******************************************************************************
* Function Definitions
*******************************************************************************/
int main(void)
{
    if(sampling_counter_init() != 0)
    {
        LOG_ERR("Sampling counter init failed");
        return -1;
    }
	if (ble_app_init() != 0)
    {
        LOG_ERR("App BLE init failed");
        return -1;
    }

    // Setup an output IO for testing
    if (!device_is_ready(gpio0_device))
    {
        LOG_ERR("GPIO0 device is not ready");
        return -1;
    }
    gpio_pin_configure(gpio0_device, 13, GPIO_OUTPUT); 
    int status = gpio_pin_set_raw(gpio0_device, 13, 1);
    if (status != 0)
    {
        LOG_ERR("GPIO0 set pin failed");
        return -1;
    }
    k_sleep(K_MSEC(100));
    status = gpio_pin_set_raw(gpio0_device, 13, 0);
	return 0;
}

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
// Sensor stuff
/*
 * @brief: Sensor sampling
 * 
 * @param (out) p_data: pointer to store new data
 * @param (out) p_length: pointer to store length of new data
 * @return int: 0 on success or negative code otherwise
 */
int sensor_sampling(uint8_t* p_data, uint16_t* p_length)
{
    int status = 0;
    __ASSERT_NO_MSG(p_data != NULL);
    __ASSERT_NO_MSG(p_length != NULL);
    //TODO: Fill in the sensor data & update the length
    *p_length = 12;
    return status;
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
    uint8_t send_payload[16] = {0}; // 4B frame count + 12B sensor data
    memcpy(send_payload, &frame_cnt, sizeof(frame_cnt));
    memcpy(send_payload + sizeof(frame_cnt), p_data, *p_length);
    return ble_set_custom_adv_payload(send_payload, sizeof(frame_cnt) + *p_length);
}

// COUNTER STUFF
const struct device *sampling_counter = DEVICE_DT_GET(SAMPLE_COUNTER_DEVICE);

void sampling_counter_cb(const struct device *dev, void *user_data)
{
#if (CONF_TEST_COUNTER != 0)
    int status = gpio_pin_set_raw(gpio0_device, 13, 1);
    if (status != 0)
    {
        LOG_ERR("GPIO0 set pin failed");
        return;
    }
    LOG_WRN("Counter callback %u ms", k_uptime_get_32());
    status = gpio_pin_set_raw(gpio0_device, 13, 0);
#else /* !(CONF_TEST_COUNTER != 0) */
    static uint32_t frame_cnt = 0;
    frame_cnt++;
    uint8_t sensor_data[20] = {0};
    uint16_t sensor_data_len = 0;
    if (sensor_sampling(sensor_data, &sensor_data_len) != 0)
    {
        LOG_ERR("Sensor sampling failed");
        return;
    }
    if (sensor_data_send_ble(sensor_data, &sensor_data_len, frame_cnt) != 0)
    {
        LOG_ERR("Sensor data send BLE failed");
        return;
    }
#endif /* End of (CONF_TEST_COUNTER != 0) */
}
/*
 * @brief Sampling counter init
 * @param None
 * @retval 0: success, -1: failed
*/
int sampling_counter_init(void)
{
    if (!device_is_ready(sampling_counter))
    {
        LOG_ERR("Sampling counter device is not ready");
        return -1;
    }
    struct counter_top_cfg sampling_counter_cfg = {
        .callback = &sampling_counter_cb,
        .flags = COUNTER_TOP_CFG_RESET_WHEN_LATE  | COUNTER_CONFIG_INFO_COUNT_UP ,
        .user_data = NULL,
        .ticks = counter_us_to_ticks(sampling_counter, SAMPLING_RATE_MS * 1000),
    };
    if (counter_set_top_value(sampling_counter, &sampling_counter_cfg) != 0)
    {
        LOG_ERR("Sampling counter set top value failed");
        return -1;
    }
    if (counter_start(sampling_counter) != 0)
    {
        LOG_ERR("Sampling counter start failed");
        return -1;
    }
    return 0;
}