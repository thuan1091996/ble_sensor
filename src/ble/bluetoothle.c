/*******************************************************************************
* Title                 :   Bluetooth LE 
* Filename              :   bluetoothle.c
* Author                :   Itachi
* Origin Date           :   2022/09/04
* Version               :   0.0.0
* Compiler              :   nRF connect SDK v2.4
* Target                :   nRF52840 SoC
* Notes                 :   None
*******************************************************************************/

/*************** MODULE REVISION LOG ******************************************
*
*    Date       Software Version	Initials	Description
*  2022/09/04       0.0.0	         Itachi      Module Created.
*
*******************************************************************************/

/** \file bluetoothle.c
 * \brief This module contains the
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "bluetoothle.h"
#include "ble_gatt.h"

/******************************************************************************
* Module configs
*******************************************************************************/
/* Logging relative */
#include <zephyr/logging/log.h>
#define MODULE_NAME			        bluetoothle
#define MODULE_LOG_LEVEL	        LOG_LEVEL_INF
LOG_MODULE_REGISTER(MODULE_NAME, MODULE_LOG_LEVEL);

#define ADV_DEFAULT_DEVICE_NAME     CONFIG_BT_DEVICE_NAME
#define ADV_CUSTOM_DATA_TYPE        BT_DATA_MANUFACTURER_DATA
#define ADV_PACKET_MAX_LEN          (31)
#define ADV_CUSTOM_PAYLOAD_LEN      (0) // Max length - 1B type - 1B length
#define ADV_NAME_MAX_LEN            (ADV_PACKET_MAX_LEN - (ADV_CUSTOM_PAYLOAD_LEN + 2) - 2)


#define CONF_ADV_NAME_APPEND_MAC_ADDR   (1) // 1: include mac address in adv name, 0: not include

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
volatile static bool is_advertising=false;
static struct bt_conn *current_conn;

#if (ADV_NAME_MAX_LEN < 0)
#warning "ADV_NAME_MAX_LEN should be greater than 0"
#else /* !(ADV_NAME_MAX_LEN < 0) */
static char ADV_NAME[ADV_NAME_MAX_LEN] = ADV_DEFAULT_DEVICE_NAME;
#endif /* End of (ADV_NAME_MAX_LEN < 0) */

uint8_t ADV_CUSTOM_PAYLOAD[ADV_CUSTOM_PAYLOAD_LEN] = {0};

static struct bt_data ADV_DATA[] = 
{
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),                   /* General discoverable mode */
#if (ADV_NAME_MAX_LEN < 0)
#warning "ADV_NAME_MAX_LEN is negative"
#else
    BT_DATA(BT_DATA_NAME_COMPLETE, ADV_NAME, sizeof(ADV_NAME)-1), /* Device name */
#endif

#if !(ADV_CUSTOM_PAYLOAD_LEN <= 0)
    BT_DATA(ADV_CUSTOM_DATA_TYPE, ADV_CUSTOM_PAYLOAD, ADV_CUSTOM_PAYLOAD_LEN)  /* Custom payload */
#endif
};

/* Bluetooth applicatiton callbacks */
static ble_callback_t ble_cb_app = {
    .ble_connected_cb = NULL,
    .ble_disconnected_cb = NULL,
    .ble_adv_started_cb = NULL,
    .ble_adv_stopped_cb = NULL,
};

/******************************************************************************
* Static Function Definitions
*******************************************************************************/
static void MTU_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) 
    {
		LOG_INF("MTU exchange done. "); 
        LOG_INF("MTU: %d", bt_gatt_get_mtu(current_conn) - 3);
	} else 
    {
		LOG_ERR("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

static void request_mtu_exchange(void)
{	int err;
	static struct bt_gatt_exchange_params exchange_params;
	exchange_params.func = MTU_exchange_cb;

	err = bt_gatt_exchange_mtu(current_conn, &exchange_params);
	if (err)
    {
		LOG_ERR("MTU exchange failed (err %d)", err);
	}  else 
    {
		LOG_INF("MTU exchange pending");
	}
}

static void request_data_len_update(void)
{
	int err;
	err = bt_conn_le_data_len_update(current_conn, BT_LE_DATA_LEN_PARAM_MAX);
    if (err) 
    {
        LOG_ERR("LE data length update request failed: %d",  err);
    }
}

static bool ble_is_advertising(void)
{
    return is_advertising;
}

static void ble_set_advertising(bool is_adv)
{
    is_advertising = is_adv;
}

static void on_ble_connect(struct bt_conn *conn, uint8_t err)
{
	if(err) 
    {
		LOG_ERR("BLE connection err: %d, re-advertising \n", err);
		return;
	}
    current_conn= bt_conn_ref(conn); 
    LOG_INF("BLE Connected");
    LOG_INF("Initializing data length update and MTU exchange");
	request_mtu_exchange();
	request_data_len_update();

    if(ble_cb_app.ble_connected_cb != NULL)
    {
        ble_cb_app.ble_connected_cb();
    }
}

static void on_ble_disconnect(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE Disconnected (reason: %d)", reason);
    if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
    if(ble_cb_app.ble_disconnected_cb != NULL)
    {
        ble_cb_app.ble_disconnected_cb();
    }
}

void on_ble_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
    LOG_INF("BLE param updated");
    // Dump all params
    LOG_INF("BLE interval: %.02f ms", (float)interval * 1.25);
    LOG_INF("BLE timeout: %.02f ms", (float)timeout * 10);
    LOG_INF("BLE latency: %d", latency);
}

void on_ble_data_len_updated(struct bt_conn *conn, struct bt_conn_le_data_len_info *info)
{
    LOG_INF("BLE data len updated");
    // Dump all params
    LOG_INF("BLE tx_max_len: %dB", info->tx_max_len);
    LOG_INF("BLE tx_max_time: %d (us)", info->tx_max_time);
    LOG_INF("BLE rx_max_len: %dB", info->rx_max_len);
    LOG_INF("BLE rx_max_time: %d (us)", info->rx_max_time);
}

/******************************************************************************
* Function Definitions
*******************************************************************************/
int ble_adv_start(void)
{
    if(ble_is_advertising())
    {
        LOG_WRN("Advertising already started\n");
        return 0;
    }
	int errorcode = bt_le_adv_start(BT_LE_ADV_CONN, ADV_DATA, ARRAY_SIZE(ADV_DATA), NULL, 0);
    if (errorcode) {
        LOG_ERR("Couldn't start advertising (err = %d)", errorcode);
        return errorcode;
    }
    ble_set_advertising(true);
    LOG_INF("Advertising successfully started\n");
    if(ble_cb_app.ble_adv_started_cb != NULL)
    {
        ble_cb_app.ble_adv_started_cb();
    }
    return 0;
}

int ble_adv_stop(void)
{
    if(!ble_is_advertising())
    {
        LOG_WRN("Advertising already stopped\n");
        return 0;
    }
    int errorcode = bt_le_adv_stop();
    if (errorcode) {
        LOG_ERR("Couldn't stop advertising (err = %d)", errorcode);
        return errorcode;
    }
    ble_set_advertising(false);
    LOG_INF("Advertising successfully stopped\n");
    if(ble_cb_app.ble_adv_stopped_cb != NULL)
    {
        ble_cb_app.ble_adv_stopped_cb();
    }
    return 0;
}

int ble_init(ble_callback_t* p_app_cb)
{
    int errorcode= 0;

    /* Bluetooth zephyr internal callbacks */
    static struct bt_conn_cb ble_cb = {
        .connected 		= &on_ble_connect,
        .disconnected 	= &on_ble_disconnect,
        .le_param_updated = &on_ble_param_updated,
        .le_data_len_updated = &on_ble_data_len_updated,
    };

    LOG_INF("BLE initializing \n\r");

    /* Initialize the application callbacks */
    if(p_app_cb != NULL)
    {
        ble_cb_app = *p_app_cb;
    }
    /* Assign callbacks for connection events */
    bt_conn_cb_register(&ble_cb);

    /* BLE initialization */
    errorcode = bt_enable(NULL);
    if(errorcode)
    {
        LOG_ERR("bt_enable return err %d \r\n", errorcode);
        return errorcode;
    }
    LOG_INF("BLE init succesfully");

#if (CONF_ADV_NAME_APPEND_MAC_ADDR != 0)
    // Get the BLE address base on NRF_FICR->DEVICEADDR
    char adv_device_name[ADV_NAME_MAX_LEN] = {0};
    // Append 4 last digits of mac address to device name 
    sprintf(adv_device_name, "%s%02X%02X%02X%02X%02X%02X", ADV_DEFAULT_DEVICE_NAME,
                    (uint8_t)( (NRF_FICR->DEVICEADDR[1] >> 8) & 0xFF), (uint8_t)(NRF_FICR->DEVICEADDR[1] & 0xFF),
                    (uint8_t)( (NRF_FICR->DEVICEADDR[0] >> 24) & 0xFF), (uint8_t)( (NRF_FICR->DEVICEADDR[0] >> 16) & 0xFF),
                    (uint8_t)( (NRF_FICR->DEVICEADDR[0] >> 8) & 0xFF), (uint8_t)(NRF_FICR->DEVICEADDR[0] & 0xFF));
    ble_set_adv_name(adv_device_name);
#endif // End of CONF_ADV_NAME_APPEND_MAC_ADDR

    return 0;
}

int ble_set_adv_name(char* p_name)
{
    __ASSERT_NO_MSG(p_name != NULL);
    if(strlen(p_name) > ADV_NAME_MAX_LEN)
    {
        LOG_ERR("BLE name too long, %d/%d", strlen(p_name), ADV_NAME_MAX_LEN);
    }
    // Find index of BT_DATA_NAME_COMPLETE in ADV_DATA[]
    for(int index = 0; index < ARRAY_SIZE(ADV_DATA); index++)
    {
        if(ADV_DATA[index].type == BT_DATA_NAME_COMPLETE)
        {
            memset((void *)ADV_DATA[index].data, 0, ADV_DATA[index].data_len);
            ADV_DATA[index].data_len = strlen(p_name);
            memcpy((void *)ADV_DATA[index].data, p_name, strlen(p_name));
            LOG_INF("BLE name set to %s", p_name);
            if(ble_is_advertising())
            {
                return bt_le_adv_update_data(ADV_DATA, ARRAY_SIZE(ADV_DATA), NULL, 0);
            }
            else
            {
                return 0;
            }
        }
    }
    LOG_ERR("Couldn't find BT_DATA_NAME_COMPLETE in ADV_DATA");
    return -1;
}

/* 
 * @brief: Set the custom payload of BLE advertising packet with given ADV type is ADV_CUSTOM_DATA_TYPE
 * 
 * @param p_data: pointer to the data
 * @param len: length of the data
 * @return int: 0 on success or negative code otherwise
 */
int ble_set_custom_adv_payload(uint8_t* p_data, uint8_t len)
{   
    __ASSERT_NO_MSG(p_data != NULL);
    if(len > ADV_CUSTOM_PAYLOAD_LEN)
    {
        LOG_ERR("BLE custom payload too long, max length is %d data will be truncated", ADV_CUSTOM_PAYLOAD_LEN);
    }
    // Find index of ADV_CUSTOM_DATA_TYPE in ADV_DATA[]
    for(int index = 0; index < ARRAY_SIZE(ADV_DATA); index++)
    {
        if(ADV_DATA[index].type == ADV_CUSTOM_DATA_TYPE)
        {
            memset((void *)ADV_DATA[index].data, 0, ADV_DATA[index].data_len);
            ADV_DATA[index].data_len = len;
            memcpy((void *)ADV_DATA[index].data, p_data, len);
            if(ble_is_advertising())
            {
                return bt_le_adv_update_data(ADV_DATA, ARRAY_SIZE(ADV_DATA), NULL, 0);
            }
            else
            {
                return 0;
            }
        }
    }
    LOG_ERR("Couldn't find ADV_CUSTOM_DATA_TYPE in ADV_DATA");
    return -1;
}