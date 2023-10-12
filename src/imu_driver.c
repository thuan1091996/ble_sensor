/*******************************************************************************
* Title                 :   Driver for LSM6DSL IMU
* Filename              :   imu_driver.c
* Origin Date           :   2023/10/09
* Version               :   0.0.0
* Compiler              :   nRF connect SDK v2.4
* Target                :   nRF52840 SoC
* Notes                 :   None
*******************************************************************************/


/** \file imu_driver.c
 *  \brief This module contains the
 */
/******************************************************************************
* Includes
*******************************************************************************/

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/
#include "zephyr/logging/log.h"
#define MODULE_NAME			        imu_sensor
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

/******************************************************************************
* Function Definitions
*******************************************************************************/
#if (CONFIG_BOARD_XIAO_BLE == 1) 

#if (CONFIG_DT_HAS_ST_LSM6DSL_ENABLED != 0)
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#else /* !(CONFIG_DT_HAS_ST_LSM6DSL_ENABLED != 0) */
#error "No enabled LSM6DSL sensor node"
#endif /* End of (CONFIG_DT_HAS_ST_LSM6DSL_ENABLED != 0) */

const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);
// Possible ODR of LSM6DSL

int sensor_init()
{
    if (!device_is_ready(lsm6dsl_dev)) 
    {
		LOG_ERR("LSM6DSL device not ready");
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
 * @brief: Convert sensor data to float
 * 
 * @param val: input sensor data
 * @return float 
 */
float sensor_data_convert(struct sensor_value *val)
{
    __ASSERT_NO_MSG(val != NULL);
    return (float)(val->val1 + val->val2 / 1000000.0f);
}

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

    static struct sensor_value accel_x, accel_y, accel_z;
    static struct sensor_value gyro_x, gyro_y, gyro_z;

    static float accel_x_f, accel_y_f, accel_z_f;
    static float gyro_x_f, gyro_y_f, gyro_z_f;
    
    if (sensor_sample_fetch_chan(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ) < 0) {
        LOG_ERR("Cannot fetch accel sample");
        return -1;
    }
    if (sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_X, &accel_x) < 0) {
        LOG_ERR("Cannot get accel_x data");
        return -1;
    }
    if (sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Y, &accel_y) < 0) {
        LOG_ERR("Cannot get accel_y data");
        return -1;
    }
    if (sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_Z, &accel_z) < 0) {
        LOG_ERR("Cannot get accel_z data");
        return -1;
    }

    /* lsm6dsl gyro */
    if (sensor_sample_fetch_chan(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ) < 0) {
        LOG_ERR("Cannot fetch gyro sample");
        return -1;
    }
    if (sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_X, &gyro_x) < 0) {
        LOG_ERR("Cannot get gyro_x data");
        return -1;
    }
    if (sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Y, &gyro_y) < 0) {
        LOG_ERR("Cannot get gyro_y data");
        return -1;
    }
    if (sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_Z, &gyro_z) < 0) {
        LOG_ERR("Cannot get gyro_z data");
        return -1;
    }

    accel_x_f = sensor_data_convert(&accel_x);
    accel_y_f = sensor_data_convert(&accel_y);
    accel_z_f = sensor_data_convert(&accel_z);

    gyro_x_f = sensor_data_convert(&gyro_x);
    gyro_y_f = sensor_data_convert(&gyro_y);
    gyro_z_f = sensor_data_convert(&gyro_z);

    uint16_t payload_idx = 0;
    memcpy(p_data + payload_idx, &accel_x_f, sizeof(accel_x_f));
    payload_idx += sizeof(accel_x_f);
    memcpy(p_data + payload_idx, &accel_y_f, sizeof(accel_y_f));
    payload_idx += sizeof(accel_y_f);
    memcpy(p_data + payload_idx, &accel_z_f, sizeof(accel_z_f));
    payload_idx += sizeof(accel_z_f);

    memcpy(p_data + payload_idx, &gyro_x_f, sizeof(gyro_x_f));
    payload_idx += sizeof(gyro_x_f);
    memcpy(p_data + payload_idx, &gyro_y_f, sizeof(gyro_y_f));
    payload_idx += sizeof(gyro_y_f);
    memcpy(p_data + payload_idx, &gyro_z_f, sizeof(gyro_z_f));
    payload_idx += sizeof(gyro_z_f);

    *p_length = payload_idx;
    return status;
}
#endif /* End of (CONFIG_BOARD_XIAO_BLE == 1)  */
