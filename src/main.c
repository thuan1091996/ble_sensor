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
#define SAMPLING_RATE_MS        (50)

#define CONF_TEST_COUNTER       (1) // 1: test counter by toggle IO in counter callback, 0: not test counter
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
// Sensor stuff

#if (CONFIG_DT_HAS_ST_LSM6DSL_ENABLED != 0)
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#else /* !(CONFIG_DT_HAS_ST_LSM6DSL_ENABLED != 0) */
#error "No enabled LSM6DSL sensor node"
#endif /* End of (CONFIG_DT_HAS_ST_LSM6DSL_ENABLED != 0) */

const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
// Possible ODR of LSM6DSL
typedef enum
{
    LSM6DSL_ODR_12_5HZ  = 12,
    LSM6DSL_ODR_26HZ    = 26,
    LSM6DSL_ODR_52HZ    = 52,
    LSM6DSL_ODR_104HZ   = 104,
    LSM6DSL_ODR_208HZ   = 208,
    LSM6DSL_ODR_416HZ   = 416,
    LSM6DSL_ODR_833HZ   = 833,
    LSM6DSL_ODR_1660HZ  = 1660,
    LSM6DSL_ODR_3330HZ  = 3330,
    LSM6DSL_ODR_6660HZ  = 6660,
    LSM6DSL_ODR_MAX
} lsm6dsl_odr_t;

int sensor_init()
{
    if (!device_is_ready(lsm6dsl_dev)) 
    {
		LOG_ERR("LSM6DSL device not ready");
        return -1;
	}
    struct sensor_value odr_attr;
    /* set accel/gyro sampling frequency to 104 Hz */
    odr_attr.val1 = LSM6DSL_ODR_52HZ;
    odr_attr.val2 = 0;
    if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) 
    {
        LOG_ERR("Cannot set sampling frequency for accel");
        return -1;
	}

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) 
    {
        LOG_ERR("Cannot set sampling frequency for gyro");
        return -1;
	}

    if (sensor_sample_fetch(lsm6dsl_dev) < 0) 
    {
        LOG_ERR("Cannot fetch sample");
        return -1;
	}
    return 0;
}


/*
 * @brief: Sensor sampling
 * 
 * @param (out) p_data: pointer to store new data
 * @param (out) p_length: pointer to store length of new data
 * @return int: 0 on success or negative code otherwise
 */
int sensor_sampling(const struct device *dev, uint8_t* p_data, uint16_t* p_length)
{
    int status = 0;
    __ASSERT_NO_MSG(p_data != NULL);
    __ASSERT_NO_MSG(p_length != NULL);

    static struct sensor_value accel_x, accel_y, accel_z;
    static struct sensor_value gyro_x, gyro_y, gyro_z;
    
    sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	/* lsm6dsl gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

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

int main(void)
{
	if (ble_app_init() != 0)
    {
        LOG_ERR("App BLE init failed");
        return -1;
    }

    if (sensor_init() != 0)
    {
        LOG_ERR("IMU sensor init failed");
        return -1;
    }

    while(1)
    {
        static uint32_t frame_cnt = 0;
        frame_cnt++;
        uint8_t sensor_data[20] = {0};
        uint16_t sensor_data_len = 0;
        if (sensor_sampling(lsm6dsl_dev ,sensor_data, &sensor_data_len) != 0)
        {
            LOG_ERR("Sensor sampling failed");
        }
        if(sensor_data_len == 12)
        {
            if (sensor_data_send_ble(sensor_data, &sensor_data_len, frame_cnt) != 0)
            {
                LOG_ERR("Sensor data send BLE failed");
            }
            LOG_INF("Frame %d sent", frame_cnt);
        }
        else
        {
            LOG_WRN("Sensor data length %d is invalid", sensor_data_len);
        }
        k_sleep(K_MSEC(SAMPLING_RATE_MS));
    }
	return 0;
}