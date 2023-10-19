/****************************************************************************
* Title                 :    header file
* Filename              :   gatt_sensor.h
* Author                :   thuantm5
* Origin Date           :   2023/03/17
* Version               :   v0.0.0
* Compiler              :   nRF connect SDK 2.4
* Target                :   nrf52
* Notes                 :   None
*****************************************************************************/

/*************** INTERFACE CHANGE LIST **************************************
*
*    Date    	Software Version    Initials   	Description
*  2023/03/17    v0.0.0         	thuantm5      Interface Created.
*
*****************************************************************************/

/** \file gatt_sensor.h
 *  \brief This module contains .
 *
 *  This is the header file for 
 */
#ifndef BLE_GATT_SENSOR_H_
#define BLE_GATT_SENSOR_H_

/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/******************************************************************************
* Preprocessor Constants
*******************************************************************************/
/** @brief UUID of the SERVICE1/ CHAR1 **/
#define SENSOR_SERVICE_UUID         (0xFF00)
#define SENSOR_CHAR_UUID            (0xFF01)
#define COMMAND_CHAR_UUID           (0xFF02)
/******************************************************************************
* Configuration Constants
*******************************************************************************/


/******************************************************************************
* Macros
*******************************************************************************/

/******************************************************************************
* Typedefs
*******************************************************************************/
/* 
 * @brief: GATT read callback
 * @param [out] p_param: (reserved)
 * @para [out] p_len: updated length of p_data
 * @return: pointer to data of the attribute
*/
typedef void* (*ble_gatt_read_cb)(void* p_param, void* p_len); /* Custom GATT READ callbacks */

/* 
 * @brief: custom GATT write callback
 * @param [in] p_data: pointer to data from GATT client
 * @param [in] p_len: pointer that contain receive length from GATT client
 * @return: (reserved)
*/
typedef int (*ble_gatt_write_cb)(void* p_data, void* p_len); /* BLE GATT callbacks custom service*/


typedef struct
{
    ble_gatt_read_cb custom_char_read_cb;    // Application callback for custom char 1 read evt
    ble_gatt_write_cb custom_char_write_cb;    // Application callback for custom char 2 read evt
}ble_custom_gatt_cb_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
void ble_custom_service_init(ble_custom_gatt_cb_t* ble_gatt_cb);
int sensor_char_send_indication(uint8_t* p_data, uint16_t len);



#endif // BLE_GATT_SENSOR_H_

/*** End of File **************************************************************/