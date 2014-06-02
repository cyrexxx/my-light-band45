/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_luxsync.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "cy_io.h"
#include "m24m02.h"
#include "nrf_delay.h"

#define MEMORY_LED_PIN_NO               LED_2




/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_luxsync       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_luxsync_t * p_luxsync, ble_evt_t * p_ble_evt)
{
    p_luxsync->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_luxsync       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_luxsync_t * p_luxsync, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_luxsync->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_luxsync       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_luxsync_t * p_luxsync, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_luxsync->LuxSync_ACK_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_luxsync->Luxsync_ack_write_handler != NULL))
    {
        p_luxsync->Luxsync_ack_write_handler(p_luxsync, p_evt_write->data[0]);
    }
}


void ble_luxsync_on_ble_evt(ble_luxsync_t * p_luxsync, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_luxsync, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_luxsync, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_luxsync, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_luxsync        Battery Service structure.
 * @param[in]   p_luxsync_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t LuxSync_char_add(ble_luxsync_t * p_luxsync, const ble_luxsync_init_t * p_luxsync_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
        
    
    // Add LuxSync characteristic
    memset(&cccd_md, 0, sizeof(cccd_md));
                
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc = BLE_GATTS_VLOC_STACK;
       
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;
    
    //Add UUID for the CHAR
		ble_uuid.type = p_luxsync->uuid_type;
    ble_uuid.uuid =LuxSync_UUID_SYNC_CHAR;
		
		memset(&attr_md, 0, sizeof(attr_md));
    
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
		attr_char_value.p_value      = NULL;
		
				
    return sd_ble_gatts_characteristic_add(p_luxsync->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_luxsync->LuxSync_handles);
        
     
}

static uint32_t LuxSync_ACK_add(ble_luxsync_t * p_lbs, const ble_luxsync_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write = 1;
	  char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = LuxSync_UUID_SYNC_ACK_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;
    
    return sd_ble_gatts_characteristic_add(p_lbs->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbs->LuxSync_ACK_handles);
}


uint32_t ble_luxsync_init(ble_luxsync_t * p_luxsync, const ble_luxsync_init_t * p_luxsync_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_luxsync->evt_handler               = p_luxsync_init->evt_handler;
	  p_luxsync->Luxsync_ack_write_handler    = p_luxsync_init->Luxsync_ack_write_handler;
    p_luxsync->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_luxsync->is_notification_supported = p_luxsync_init->support_notification;
    
    
    // Add service
    ble_uuid128_t base_uuid = LuxSync_UUID_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_luxsync->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		ble_uuid.type = p_luxsync->uuid_type;
    ble_uuid.uuid = LuxSync_UUID_SERVICE;
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_luxsync->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    // Add Lux Sync characteristic
	  err_code = LuxSync_char_add(p_luxsync, p_luxsync_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		// Add Lux Sync ACK characteristic
	  err_code = LuxSync_ACK_add(p_luxsync, p_luxsync_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
      return NRF_SUCCESS;
}


uint32_t ble_luxsync_write_update(ble_luxsync_t * p_luxsync, uint8_t mem_data)
{
 
	uint32_t err_code = NRF_SUCCESS;  	    		        
        // Send value if connected and notifying
 if ((p_luxsync->conn_handle != BLE_CONN_HANDLE_INVALID) && p_luxsync->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            uint16_t               len;
						len = sizeof(mem_data);
						
            memset(&hvx_params, 0, sizeof(hvx_params));
                
            hvx_params.handle   = p_luxsync->LuxSync_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &len;
            hvx_params.p_data   = &mem_data;
					  err_code = sd_ble_gatts_hvx(p_luxsync->conn_handle, &hvx_params);
         }   
     return err_code;
								

}

uint32_t ble_luxsync_ACK_update(ble_luxsync_t * p_luxsync, uint8_t Lux_Ack)
{
    uint32_t err_code = NRF_SUCCESS;
	  ble_gatts_hvx_params_t hvx_params;
    uint16_t len = sizeof(Lux_Ack);
    
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.handle = p_luxsync->LuxSync_ACK_handles.value_handle;
    hvx_params.p_data = &Lux_Ack;
    hvx_params.p_len = &len;
    err_code = sd_ble_gatts_hvx(p_luxsync->conn_handle, &hvx_params);
	  err_code=NRF_SUCCESS;
    return err_code;
			
		
}
void send_data_stream(ble_luxsync_t * p_luxsync)
{
  ble_luxsync_ACK_update(p_luxsync,0x02);    // •	The device acknowledges by changing it to 0x02 and starts sending data 
	nrf_gpio_pin_clear(MEMORY_LED_PIN_NO);     // to indicate Memory is busy
	//code
	nrf_delay_ms(50000);
	nrf_gpio_pin_set(MEMORY_LED_PIN_NO);
	ble_luxsync_ACK_update(p_luxsync,0x04);   //•	Once all the data is sent it changes sync ACk to 0x04 indicating end of data stream
}

void upload_done(ble_luxsync_t * p_luxsync)
{ 
  ble_luxsync_ACK_update(p_luxsync,0x08);
	//lux_timers_stop();
	nrf_gpio_pin_clear(MEMORY_LED_PIN_NO);     // to indicate Memory is busy
	//code to erase memory
	i2c_eeprom_erase();
	//timers_start();
	nrf_gpio_pin_set(MEMORY_LED_PIN_NO);
	NVIC_SystemReset(); // reset device as the connection usually fails during erase. 
}
	
