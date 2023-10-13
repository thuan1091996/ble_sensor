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
#define CUSTOM_SERVICE1_NAME    CUSTOM_SERVICE1
#define BT_CUSTOM_SERV1_UUID    BT_UUID_128_ENCODE(0xe9ea0001, 0xe19b, 0x482d, 0x9293, 0xc7907585fc11)
#define BT_CUSTOM_CHAR1_UUID    BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x482d, 0x9293, 0xc7907585fc11)
#define BT_CUSTOM_CHAR2_UUID    BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x482d, 0x9293, 0xc7907585fc22)
#define BT_CUSTOM_CHAR3_UUID    BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x482d, 0x9293, 0xc7907585fc33)
#define BT_CUSTOM_CHAR4_UUID    BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x482d, 0x9293, 0xc7907585fc44)
#define BT_CUSTOM_CHAR5_UUID    BT_UUID_128_ENCODE(0xe9ea0002, 0xe19b, 0x482d, 0x9293, 0xc7907585fc55)

#define BT_UUID_CUSTOM_SERV1    BT_UUID_DECLARE_128(BT_CUSTOM_SERV1_UUID)
#define BT_UUID_CUSTOM_CHAR1    BT_UUID_DECLARE_128(BT_CUSTOM_CHAR1_UUID)
#define BT_UUID_CUSTOM_CHAR2    BT_UUID_DECLARE_128(BT_CUSTOM_CHAR2_UUID)
#define BT_UUID_CUSTOM_CHAR3    BT_UUID_DECLARE_128(BT_CUSTOM_CHAR3_UUID)
#define BT_UUID_CUSTOM_CHAR4    BT_UUID_DECLARE_128(BT_CUSTOM_CHAR4_UUID)
#define BT_UUID_CUSTOM_CHAR5    BT_UUID_DECLARE_128(BT_CUSTOM_CHAR5_UUID)


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
    ble_gatt_read_cb custom_char1_read_cb;    // Application callback for custom char 1 read evt
    ble_gatt_read_cb custom_char2_read_cb;    // Application callback for custom char 2 read evt
    ble_gatt_read_cb custom_char3_read_cb;    // Application callback for custom char 3 read evt
    ble_gatt_read_cb custom_char4_read_cb;    // Application callback for custom char 4 read evt
    ble_gatt_write_cb custom_char5_write_cb;   // Application callback for custom char 5 write evt
}ble_custom_gatt_cb_t;

/******************************************************************************
* Variables
*******************************************************************************/


/******************************************************************************
* Function Prototypes
*******************************************************************************/
void ble_custom_service_init(ble_custom_gatt_cb_t* ble_gatt_cb);
int char3_send_indication(uint8_t* p_data, uint16_t len);
int char4_send_notify(uint8_t* p_data, uint16_t len);



#endif // BLE_GATT_SENSOR_H_

/*** End of File **************************************************************/