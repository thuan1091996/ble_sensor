/*******************************************************************************
* Title                 :    
* Filename              :   gatt_sensor.c
* Author                :   thuantm5
* Origin Date           :   2023/09/12
* Version               :   0.0.0
* Compiler              :   nRF connect SDK 2.4
* Target                :   nrf52
* Notes                 :   None
*******************************************************************************/

/*************** MODULE REVISION LOG ******************************************
*
*    Date       Software Version	Initials	Description
*  2023/03/17       0.0.0	         thuantm5      Module Created.
*
*******************************************************************************/

/** \file gatt_sensor.c
 *  \brief This module contains the
 */
/******************************************************************************
* Includes
*******************************************************************************/

#include "bluetoothle.h"
#include "ble_gatt.h"
/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/
#include <zephyr/logging/log.h>
#define MODULE_NAME			        gatt_sensor
#define MODULE_LOG_LEVEL	        LOG_LEVEL_INF
LOG_MODULE_REGISTER(MODULE_NAME, MODULE_LOG_LEVEL);
/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/



/******************************************************************************
* Module Typedefs
*******************************************************************************/


/******************************************************************************
* Module Variable Definitions
*******************************************************************************/
static bool g_sensor_char_indicate_en=false;
static const struct bt_gatt_attr *p_sensor_char_attr = NULL; 
static uint8_t indicating;
static struct bt_gatt_indicate_params ind_params;

static ble_custom_gatt_cb_t ble_custom_gatt_cb = 
{
    .custom_char_read_cb = NULL,
    .custom_char_write_cb = NULL,
};

const struct bt_uuid_16 sensor_service_uuid = BT_UUID_INIT_16(SENSOR_SERVICE_UUID);
const struct bt_uuid_16 sensor_char_uuid = BT_UUID_INIT_16(SENSOR_CHAR_UUID);
const struct bt_uuid_16 command_char_uuid = BT_UUID_INIT_16(COMMAND_CHAR_UUID);

/******************************************************************************
* Static Function Prototypes
*******************************************************************************/
static ssize_t cmd_char_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t cmd_char_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static ssize_t sensor_char_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static void sensor_ccc_attr_changed(const struct bt_gatt_attr *attr, uint16_t value);

/******************************************************************************
* Static Function Definitions
*******************************************************************************/
static ssize_t sensor_char_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_INF("SENSOR_CHAR - ReadCB");
    return bt_gatt_attr_read(conn, attr, buf, len, offset, "WARN: INVALID DATA", strlen("WARN: INVALID DATA"));
}

static ssize_t cmd_char_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_INF("CMD_CHAR - ReadCB");
    void* p_cmd_char_data = NULL;      // Pointer to char1 data    
    uint16_t cmd_char_data_len = 0;
    do
    {
        if(ble_custom_gatt_cb.custom_char_read_cb == NULL)
            break;
        // Call application callback to update p_cmd_char_data point to the new data
        p_cmd_char_data = ble_custom_gatt_cb.custom_char_read_cb(NULL, &cmd_char_data_len);
        
        if(p_cmd_char_data == NULL)
        {
            LOG_ERR("Invalid data pointer");
            break;
        }

        if(cmd_char_data_len > len)
        {
            LOG_ERR("%d is an invalid data length", cmd_char_data_len);
            break;
        }

        return bt_gatt_attr_read(conn, attr, buf, len, offset, p_cmd_char_data, cmd_char_data_len);
    }while(0);

    LOG_ERR("Invalid char1 attribute payload");
    return bt_gatt_attr_read(conn, attr, buf, len, offset, "WARN: INVALID DATA", strlen("WARN: INVALID DATA"));
}

static ssize_t cmd_char_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    LOG_INF("CMDCHAR - WriteCB: Received %dB ", len);
    LOG_HEXDUMP_DBG(buf, len, "CMDCHAR - WriteCB: Received data");
    if(ble_custom_gatt_cb.custom_char_write_cb != NULL)
    {
        int recv_len = len;
        ble_custom_gatt_cb.custom_char_write_cb((void*)buf, &recv_len);
    }
    return len;
}

void sensor_ccc_attr_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    switch(value)
    {
        case BT_GATT_CCC_INDICATE:

            g_sensor_char_indicate_en = true;
            break;

        case 0: 
            g_sensor_char_indicate_en = false;
            break;
        
        default: 
            LOG_ERR("Error, CCCD has been set to an invalid value");     
    } 
    LOG_INF("SENSOR_CHAR indication %s.", g_sensor_char_indicate_en ? "enabled" : "disabled");
}


/******************************************************************************
* Function Definitions
*******************************************************************************/

/**
 * @brief Assigned application callbacks for custom char 1 - > custom char 5
 * @param ble_gatt_cb Application callbacks 
 */
void ble_custom_service_init(ble_custom_gatt_cb_t* ble_gatt_cb)
{
    if(ble_gatt_cb != NULL)
    {
        ble_custom_gatt_cb.custom_char_read_cb = ble_gatt_cb->custom_char_read_cb;
        ble_custom_gatt_cb.custom_char_write_cb = ble_gatt_cb->custom_char_write_cb;
    }
    // Get attribute pointer of char3 and char4 to support notification
    p_sensor_char_attr = bt_gatt_find_by_uuid(NULL, 0 , (const struct bt_uuid *)&sensor_char_uuid);
    LOG_INF("Attribute handle %p", p_sensor_char_attr);
}

static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	LOG_DBG("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	LOG_DBG("Indication complete\n");
	indicating = 0U;
}

int sensor_char_send_indication(uint8_t* p_data, uint16_t len)
{
    __ASSERT_NO_MSG(p_data != NULL);
    if(p_sensor_char_attr == NULL)
    {
        LOG_ERR("Can't find SENSOR_CHAR attribute handle");
        return -1;
    }
    int ret_val = -1;
    if(!g_sensor_char_indicate_en)
    {
        LOG_INF("SENSOR_CHAR indication disable");
    }
    else 
    {
        LOG_DBG("SENSOR_CHAR sending %dB indication", len);
        // ret_val = bt_gatt_notify(NULL, p_sensor_char_attr, p_data, len);
        ind_params.attr = p_sensor_char_attr;
		ind_params.func = indicate_cb;
		ind_params.destroy = indicate_destroy;
		ind_params.data = p_data;
		ind_params.len = len;
        ret_val = bt_gatt_indicate(NULL, &ind_params);
    }
    return ret_val;
}


BT_GATT_SERVICE_DEFINE(CUSTOM_SERVICE1_NAME,
BT_GATT_PRIMARY_SERVICE((void*)&sensor_service_uuid),
    BT_GATT_CHARACTERISTIC((const struct bt_uuid *) &sensor_char_uuid,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
                    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, /* No security enable */
                    sensor_char_read_cb, NULL, NULL),          
    BT_GATT_CCC(sensor_ccc_attr_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC((const struct bt_uuid *)&command_char_uuid,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, /* No security enable */
                    cmd_char_read_cb, cmd_char_write_cb, NULL),
);