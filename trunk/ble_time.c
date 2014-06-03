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

#include "ble_time.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ble_date_time.h"
#include "ble_l2cap.h"


#define INVALID_time  255

#define INVALID_VALUE  		0

#define INVALID_YEAR  		2012
#define INVALID_MONTH  		0
#define INVALID_DAY  		  0
#define INVALID_HOURS  		0
#define INVALID_MINUTES 	0
#define INVALID_SECONDS 	0
#define INVALID_DAY_OF_WEEK 0


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_time       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_time_t * p_time, ble_evt_t * p_ble_evt)
{
    p_time->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_time       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_time_t * p_time, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_time->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_time       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_time_t * p_time, ble_evt_t * p_ble_evt)
{   
   ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	 
	if ((p_evt_write->handle == p_time->time_byte2_char_handles.value_handle ) &&
      (p_evt_write->len == 1) && 
      (p_time->time_byte2_write_handler != NULL))
					{
						p_time->time_byte2_write_handler(p_time,p_evt_write->data[0]);	
						
					}
  else if ((p_evt_write->handle == p_time->time_byte1_char_handles.value_handle ) &&
      (p_evt_write->len == 1) && 
      (p_time->time_byte1_write_handler != NULL))
					{
						p_time->time_byte1_write_handler(p_time,p_evt_write->data[0]);	
					}	
   else if ((p_evt_write->handle == p_time->time_byte0_char_handles.value_handle ) &&
      (p_evt_write->len == 1) && 
      (p_time->time_byte0_write_handler != NULL))
					{
						p_time->time_byte0_write_handler(p_time,p_evt_write->data[0]);	
					}						
	  
}


void ble_time_on_ble_evt(ble_time_t * p_time, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_time, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_time, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_time, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Current Time characteristic.
 *
 * @param[in]   p_time        Time Service structure.
 * @param[in]   p_time_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t current_time_byte2_char_add(ble_time_t * p_time, const ble_time_init_t * p_time_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    
    
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_time->uuid_type;
    ble_uuid.uuid = TIME_UUID_CURRENT_TIME_BYTE2_CHAR;
    
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
    
    return sd_ble_gatts_characteristic_add(p_time->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_time->time_byte2_char_handles);
    
}

/**@brief Function for adding the Current Time characteristic.
 *
 * @param[in]   p_time        Time Service structure.
 * @param[in]   p_time_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t current_time_byte1_char_add(ble_time_t * p_time, const ble_time_init_t * p_time_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
 

  	memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_time->uuid_type;
    ble_uuid.uuid = TIME_UUID_CURRENT_TIME_BYTE1_CHAR;
    
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
    
    return sd_ble_gatts_characteristic_add(p_time->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_time->time_byte1_char_handles);
    
}
/**@brief Function for adding the Current Time characteristic.
 *
 * @param[in]   p_time        Time Service structure.
 * @param[in]   p_time_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t current_time_byte0_char_add(ble_time_t * p_time, const ble_time_init_t * p_time_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_time->uuid_type;
    ble_uuid.uuid = TIME_UUID_CURRENT_TIME_BYTE0_CHAR;
    
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
    
    return sd_ble_gatts_characteristic_add(p_time->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_time->time_byte0_char_handles);
    
}



uint32_t ble_time_init(ble_time_t * p_time, const ble_time_init_t * p_time_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_time->evt_handler               = p_time_init->evt_handler;
	  p_time->time_byte2_write_handler   = p_time_init->time_byte2_write_handler;
	  p_time->time_byte1_write_handler    = p_time_init->time_byte1_write_handler;
	  p_time->time_byte0_write_handler  = p_time_init->time_byte0_write_handler;
    p_time->conn_handle               = BLE_CONN_HANDLE_INVALID;
   
    p_time->time_last                 = p_time_init->initial_time_on_start;
    // Add service
    ble_uuid128_t base_uuid = TIME_UUID_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_time->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_time->uuid_type;
    ble_uuid.uuid = TIME_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_time->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add current time characteristic
	  err_code= current_time_byte2_char_add(p_time, p_time_init);
		if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		 err_code= current_time_byte1_char_add(p_time, p_time_init);
		if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		 err_code= current_time_byte0_char_add(p_time, p_time_init);
		if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		return NRF_SUCCESS;
}

//Update char values 
uint32_t ble_time_time_byte2_update(ble_time_t * p_time, uint8_t time)
{
    
    uint16_t len = sizeof(uint8_t);
                
        // Update database
  return sd_ble_gatts_value_set(p_time->time_byte2_char_handles.value_handle,
                                          0,
                                          &len,
                                          &time);
}

uint32_t ble_time_time_byte1_update(ble_time_t * p_time, uint8_t time)
{
    

    uint16_t len = sizeof(uint8_t);
        
       
        // Update database
   return sd_ble_gatts_value_set(p_time->time_byte1_char_handles.value_handle,
                                          0,
                                          &len,
                                          &time);
}
uint32_t ble_time_time_byte0_update(ble_time_t * p_time, uint8_t time)
{
    

    uint16_t len = sizeof(uint8_t);
        
       
        // Update database
   return sd_ble_gatts_value_set(p_time->time_byte0_char_handles.value_handle,
                                          0,
                                          &len,
                                          &time);
}

static char not_leap(uint16_t year)
{
if (!(year%100))
	{
		return (char)(year%400);
	}
else
	{
		return (char)(year%4);
	}
		
}

uint32_t update_current_time(ble_time_t * p_time,ble_date_time_t * p_current_date_time,uint8_t * p_date_timebuffer)
{ uint32_t err_code = NRF_SUCCESS;
	uint8_t  len;
	uint8_t date_timebuffer_temp[3];
	p_current_date_time->seconds+=10;
	if(p_current_date_time->seconds>=60)
	{
		p_current_date_time->seconds=0;
		p_current_date_time->minutes+=1;
		if(p_current_date_time->minutes==60)
		{
			p_current_date_time->minutes=0;
			p_current_date_time->hours+=1;
			if(p_current_date_time->hours==24)
			{
				p_current_date_time->hours=0;
				p_current_date_time->day+=1;
				if(p_current_date_time->day==32)
				{
					p_current_date_time->day=1;
					p_current_date_time->month+=1;
				}
				else if(p_current_date_time->day==31)
				{
					if((p_current_date_time->month==4)||(p_current_date_time->month==6)||(p_current_date_time->month==9)||(p_current_date_time->month==11))
					{
						p_current_date_time->day=1;
						p_current_date_time->month+=1;
					}
				}
				else if(p_current_date_time->day==30)
				{
					if(p_current_date_time->month==2)
					{
						p_current_date_time->day=1;
						p_current_date_time->month+=1;
					}
				}
				else if(p_current_date_time->day==29)
				{
					if((p_current_date_time->month==2)&&(not_leap(p_current_date_time->year)))
					{
						p_current_date_time->day=1;
						p_current_date_time->month+=1;
					}	
				}
				if(p_current_date_time->month==13)
				{
					p_current_date_time->month=1;
					p_current_date_time->year+=1;
				}
			}
		}
	}
  len=cy_date_time_encode(p_current_date_time,date_timebuffer_temp);
	*p_date_timebuffer=date_timebuffer_temp[0];
  *(p_date_timebuffer+1)=date_timebuffer_temp[1];
	*(p_date_timebuffer+2)=date_timebuffer_temp[2];
	
	
	ble_time_time_byte2_update( p_time,  date_timebuffer_temp[0]);
	 if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	ble_time_time_byte1_update( p_time,  date_timebuffer_temp[1]);
		 if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	ble_time_time_byte0_update( p_time,  date_timebuffer_temp[2]);
		 if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

		return err_code;
}



