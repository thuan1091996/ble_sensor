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

// COUNTER STUFF
const struct device *sampling_counter = DEVICE_DT_GET(SAMPLE_COUNTER_DEVICE);

void sampling_counter_cb(const struct device *dev, void *user_data)
{
    int status = gpio_pin_set_raw(gpio0_device, 13, 1);
    if (status != 0)
    {
        LOG_ERR("GPIO0 set pin failed");
        return;
    }
    LOG_WRN("Counter callback %u ms", k_uptime_get_32());
    status = gpio_pin_set_raw(gpio0_device, 13, 0);
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