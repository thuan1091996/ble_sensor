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

#define ADV_DEFAULT_DEVICE_NAME     "nRF52-BLE"
#define ADV_PACKET_MAX_LEN          (29)
#define ADV_NAME_MAX_LEN            (29)
#define BLE_CONFIG_ADV_NAME         (1)     /* 1 -> Include ADV name in the ADC packet*/


/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static char ADV_NAME[ADV_NAME_MAX_LEN] = ADV_DEFAULT_DEVICE_NAME;
static struct bt_data ADV_DATA[] = 
{
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,  BT_CUSTOM_SERV1_UUID),   /* Custom service UUID */
    BT_DATA(BT_DATA_NAME_COMPLETE, ADV_NAME, sizeof(ADV_DEFAULT_DEVICE_NAME))  /* Device name */
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
static void on_ble_connect(struct bt_conn *conn, uint8_t err)
{
	if(err) 
    {
		LOG_ERR("BLE connection err: %d, re-advertising \n", err);
		return;
	}
    LOG_INF("BLE Connected.");

    if(ble_cb_app.ble_connected_cb != NULL)
    {
        ble_cb_app.ble_connected_cb();
    }
}

static void on_ble_disconnect(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE Disconnected (reason: %d)", reason);
    if(ble_cb_app.ble_disconnected_cb != NULL)
    {
        ble_cb_app.ble_disconnected_cb();
    }
}

/******************************************************************************
* Function Definitions
*******************************************************************************/
int ble_adv_start(void)
{
    struct bt_le_adv_param adv_param =
    {
        .id = 0,
        .sid = 0,
        .options = BT_LE_ADV_OPT_NONE | BT_LE_ADV_OPT_USE_NAME,
        .interval_min = 0x0020, // 20ms
        .interval_max = 0x0030, // 30ms
    };
	int errorcode = bt_le_adv_start(&adv_param, ADV_DATA, ARRAY_SIZE(ADV_DATA), NULL, 0);
    if (errorcode) {
        LOG_ERR("Couldn't start advertising (err = %d)", errorcode);
        return errorcode;
    }
    LOG_INF("Advertising fully started\n");
    if(ble_cb_app.ble_adv_started_cb != NULL)
    {
        ble_cb_app.ble_adv_started_cb();
    }
    return 0;
}

int ble_adv_stop(void)
{
    int errorcode = bt_le_adv_stop();
    if (errorcode) {
        LOG_ERR("Couldn't stop advertising (err = %d)", errorcode);
        return errorcode;
    }
    LOG_INF("Advertising 0fully stopped\n");
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
    return 0;
}

int ble_set_adv_name(char* p_name)
{
    __ASSERT_NO_MSG(p_name != NULL);
    if(strlen(p_name) > ADV_NAME_MAX_LEN)
    {
        LOG_ERR("BLE name too long, max length is %d", ADV_NAME_MAX_LEN);
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
            return bt_le_adv_update_data(ADV_DATA, ARRAY_SIZE(ADV_DATA), NULL, 0);
        }
    }
    LOG_ERR("Couldn't find BT_DATA_NAME_COMPLETE in ADV_DATA");
    return -1;
}