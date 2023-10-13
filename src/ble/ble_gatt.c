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
#define MODULE_LOG_LEVEL	        LOG_LEVEL_DBG
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
static bool g_char3_indicate_en=false, g_char4_notify_en=false;
static const struct bt_gatt_attr *p_char3_attr = NULL, *p_char4_attr = NULL; 

static ble_custom_gatt_cb_t ble_custom_gatt_cb = 
{
    .custom_char1_read_cb = NULL,
    .custom_char2_read_cb = NULL,
    .custom_char3_read_cb = NULL,
    .custom_char4_read_cb = NULL,
    .custom_char5_write_cb = NULL,
};


/******************************************************************************
* Static Function Prototypes
*******************************************************************************/
static ssize_t custom_char1_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t custom_char2_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t custom_char3_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t custom_char4_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset);
static ssize_t custom_char5_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags);

static void custom_char3_notify_changed(const struct bt_gatt_attr *attr, uint16_t value);
static void custom_char4_notify_changed(const struct bt_gatt_attr *attr, uint16_t value);

/******************************************************************************
* Static Function Definitions
*******************************************************************************/

static ssize_t custom_char1_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_INF("CUSTOM_CHAR1 - ReadCB");
    void* p_char1_data = NULL;      // Pointer to char1 data    
    uint16_t char1_data_len = 0;
    do
    {
        if(ble_custom_gatt_cb.custom_char1_read_cb == NULL)
            break;
        // Call application callback to update p_char1_data point to the new data
        p_char1_data = ble_custom_gatt_cb.custom_char1_read_cb(NULL, &char1_data_len);
        
        if(p_char1_data == NULL)
        {
            LOG_ERR("Invalid data pointer");
            break;
        }

        if(char1_data_len > len)
        {
            LOG_ERR("%d is an invalid data length", char1_data_len);
            break;
        }

        return bt_gatt_attr_read(conn, attr, buf, len, offset, p_char1_data, char1_data_len);
    }while(0);

    LOG_ERR("Invalid char1 attribute payload");
    return bt_gatt_attr_read(conn, attr, buf, len, offset, "WARN: INVALID DATA", strlen("WARN: INVALID DATA"));
}

static ssize_t custom_char2_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_INF("CUSTOM_CHAR2 - ReadCB");
    void* p_char2_data = NULL;      // Pointer to char2 data    
    uint16_t char2_data_len = 0;
    do
    {
        if(ble_custom_gatt_cb.custom_char2_read_cb == NULL)
            break;
        // Call application callback to update p_char2_data point to the new data
        p_char2_data = ble_custom_gatt_cb.custom_char2_read_cb(NULL, &char2_data_len);
        
        if(p_char2_data == NULL)
        {
            LOG_ERR("Invalid data pointer");
            break;
        }

        if(char2_data_len > len)
        {
            LOG_ERR("%d is an invalid data length", char2_data_len);
            break;
        }

        return bt_gatt_attr_read(conn, attr, buf, len, offset, p_char2_data, char2_data_len);
    }while(0);

    LOG_ERR("Invalid char2 attribute payload");
    return bt_gatt_attr_read(conn, attr, buf, len, offset, "WARN: INVALID DATA", strlen("WARN: INVALID DATA"));
}

static ssize_t custom_char3_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_INF("CUSTOM_CHAR3 - ReadCB");
    void* p_char3_data = NULL;      // Pointer to char3 data    
    uint16_t char3_data_len = 0;
    do
    {
        if(ble_custom_gatt_cb.custom_char3_read_cb == NULL)
            break;
        // Call application callback to update p_char3_data point to the new data
        p_char3_data = ble_custom_gatt_cb.custom_char3_read_cb(NULL, &char3_data_len);
        
        if(p_char3_data == NULL)
        {
            LOG_ERR("Invalid data pointer");
            break;
        }

        if(char3_data_len > len)
        {
            LOG_ERR("%d is an invalid data length", char3_data_len);
            break;
        }

        return bt_gatt_attr_read(conn, attr, buf, len, offset, p_char3_data, char3_data_len);
    }while(0);

    LOG_ERR("Invalid char3 attribute payload");
    return bt_gatt_attr_read(conn, attr, buf, len, offset, "WARN: INVALID DATA", strlen("WARN: INVALID DATA"));
}

static ssize_t custom_char4_read_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
    LOG_INF("CUSTOM_CHAR4 - ReadCB");
    void* p_char4_data = NULL;      // Pointer to char4 data    
    uint16_t char4_data_len = 0;
    do
    {
        if(ble_custom_gatt_cb.custom_char4_read_cb == NULL)
            break;
        // Call application callback to update p_char4_data point to the new data
        p_char4_data = ble_custom_gatt_cb.custom_char4_read_cb(NULL, &char4_data_len);
        
        if(p_char4_data == NULL)
        {
            LOG_ERR("Invalid data pointer");
            break;
        }

        if(char4_data_len > len)
        {
            LOG_ERR("%d is an invalid data length", char4_data_len);
            break;
        }

        return bt_gatt_attr_read(conn, attr, buf, len, offset, p_char4_data, char4_data_len);
    }while(0);

    LOG_ERR("Invalid char4 attribute payload");
    return bt_gatt_attr_read(conn, attr, buf, len, offset, "WARN: INVALID DATA", strlen("WARN: INVALID DATA"));
}


static ssize_t custom_char5_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(offset);
    ARG_UNUSED(flags);
    LOG_INF("CUSTOM_CHAR5 - WriteCB");
    if(ble_custom_gatt_cb.custom_char5_write_cb != NULL)
    {
        int recv_len = len;
        ble_custom_gatt_cb.custom_char5_write_cb((void*)buf, &recv_len);
    }
    return len;
}

void custom_char3_notify_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    switch(value)
    {
        case BT_GATT_CCC_INDICATE:

            g_char3_indicate_en = true;
            break;

        case 0: 
            g_char3_indicate_en = false;
            break;
        
        default: 
            LOG_ERR("Error, CCCD has been set to an invalid value");     
    } 
    LOG_INF("CUSTOM_CHAR3 notification %s.", g_char3_indicate_en ? "enabled" : "disabled");
}

void custom_char4_notify_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    ARG_UNUSED(attr);
    switch(value)
    {
        case BT_GATT_CCC_NOTIFY:

            g_char4_notify_en = true;
            break;

        case 0: 
            g_char4_notify_en = false;
            break;
        
        default: 
            LOG_ERR("Error, CCCD has been set to an invalid value");     
    } 
    LOG_INF("CUSTOM_CHAR4 notification %s.", g_char4_notify_en ? "enabled" : "disabled");
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
        ble_custom_gatt_cb.custom_char1_read_cb = ble_gatt_cb->custom_char1_read_cb;
        ble_custom_gatt_cb.custom_char2_read_cb = ble_gatt_cb->custom_char2_read_cb;
        ble_custom_gatt_cb.custom_char3_read_cb = ble_gatt_cb->custom_char3_read_cb;
        ble_custom_gatt_cb.custom_char4_read_cb = ble_gatt_cb->custom_char4_read_cb;
        ble_custom_gatt_cb.custom_char5_write_cb = ble_gatt_cb->custom_char5_write_cb;
    }
    // Get attribute pointer of char3 and char4 to support notification
    p_char3_attr = bt_gatt_find_by_uuid(NULL, 0 , BT_UUID_CUSTOM_CHAR3);
    p_char4_attr = bt_gatt_find_by_uuid(NULL, 0 , BT_UUID_CUSTOM_CHAR4);
    LOG_INF("Attribute handle %p, %p", p_char3_attr, p_char4_attr);
}

static uint8_t indicating;
static struct bt_gatt_indicate_params ind_params;
static void indicate_cb(struct bt_conn *conn, struct bt_gatt_indicate_params *params, uint8_t err)
{
	LOG_WRN("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	LOG_WRN("Indication complete\n");
	indicating = 0U;
}
int char3_send_indication(uint8_t* p_data, uint16_t len)
{
    __ASSERT_NO_MSG(p_data != NULL);
    if(p_char3_attr == NULL)
    {
        LOG_ERR("Can't find CUSTOM_CHAR3 attribute handle");
        return -1;
    }
    int ret_val = -1;
    if(!g_char3_indicate_en)
    {
        LOG_WRN("CUSTOM_CHAR3 notification disable");
    }
    else 
    {
        LOG_INF("CUSTOM_CHAR3 sending %dB notification", len);
        // ret_val = bt_gatt_notify(NULL, p_char3_attr, p_data, len);
        ind_params.attr = p_char3_attr;
		ind_params.func = indicate_cb;
		ind_params.destroy = indicate_destroy;
		ind_params.data = p_data;
		ind_params.len = len;
        ret_val = bt_gatt_indicate(NULL, &ind_params);
    }
    return ret_val;
}

int char4_send_notify(uint8_t* p_data, uint16_t len)
{
    __ASSERT_NO_MSG(p_data != NULL);
    if(p_char4_attr == NULL)
    {
        LOG_ERR("Can't find CUSTOM_CHAR4 attribute handle");
        return -1;
    }
    int ret_val = 0;
    if(!g_char4_notify_en)
    {
        LOG_WRN("CUSTOM_CHAR4 notification disable");
    }
    else 
    {
        LOG_INF("CUSTOM_CHAR4 sending %dB notification", len);
        ret_val = bt_gatt_notify(NULL, p_char4_attr, p_data, len);
    }
    return ret_val;
}


/*

*/
BT_GATT_SERVICE_DEFINE(CUSTOM_SERVICE1_NAME,
BT_GATT_PRIMARY_SERVICE(BT_UUID_CUSTOM_SERV1),
    // Custom char1
    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_CHAR1,
                    BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, /* No security enable */
                    custom_char1_read_cb, NULL, NULL),
    // Custom char2
    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_CHAR2,
                    BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, /* No security enable */
                    custom_char2_read_cb, NULL, NULL),
    // Custom char3
    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_CHAR3,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
                    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, /* No security enable */
                    custom_char3_read_cb, NULL, NULL),          
    BT_GATT_CCC(custom_char3_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    // Custom char4
    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_CHAR4,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, /* No security enable */
                    custom_char4_read_cb, NULL, NULL),
    BT_GATT_CCC(custom_char4_notify_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    // Custom char5
    BT_GATT_CHARACTERISTIC(BT_UUID_CUSTOM_CHAR5,
                    BT_GATT_CHRC_WRITE,
                    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, /* No security enable */
                    NULL, custom_char5_write_cb, NULL),
);