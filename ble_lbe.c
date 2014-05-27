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

#include "ble_lbe.h"
#include <string.h>                                                             	
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ble_l2cap.h"
#include "math.h"

#define OPCODE_LENGTH  1                                                    /**< Length of opcode inside  packet. */
#define HANDLE_LENGTH  2                                                    /**< Length of handle inside  packet. */
#define MAX_HTM_LEN   (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)   /**< Maximum size of a transmitted  Measurement. */

#define INVALID_LIGHT_LEVEL  999999


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_lbe       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_lbe_t * p_lbe, ble_evt_t * p_ble_evt)
{
    p_lbe->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_lbe       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_lbe_t * p_lbe, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbe->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_lbe       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_lbe_t * p_lbe, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if ((p_evt_write->handle == p_lbe->LUX_sample_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_lbe->s_rate_write_handler != NULL))
    {
        p_lbe->s_rate_write_handler(p_lbe, p_evt_write->data[0]);
    }
}


void ble_lbe_on_ble_evt(ble_lbe_t * p_lbe, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_lbe, p_ble_evt);
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_lbe, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_lbe, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_lbe        Battery Service structure.
 * @param[in]   p_lbe_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t lux_1_level_char_add(ble_lbe_t * p_lbe, const ble_lbe_init_t * p_lbe_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
        
    
    // Add Battery Level characteristic
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
    
    ble_uuid.type = p_lbe->uuid_type;
    ble_uuid.uuid = LBE_UUID_LUX_SENS_1_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    //attr_md.read_perm  = p_lbe_init->light_level_char_attr_md.read_perm;
    //attr_md.write_perm = p_lbe_init->light_level_char_attr_md.write_perm;
		
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
           
    return sd_ble_gatts_characteristic_add(p_lbe->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbe->LUX_1_handles);
}


static uint32_t lux_2_level_char_add(ble_lbe_t * p_lbe, const ble_lbe_init_t * p_lbe_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
        
    
    // Add Battery Level characteristic
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
    
    ble_uuid.type = p_lbe->uuid_type;
    ble_uuid.uuid = LBE_UUID_LUX_SENS_2_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    //attr_md.read_perm  = p_lbe_init->light_level_char_attr_md.read_perm;
    //attr_md.write_perm = p_lbe_init->light_level_char_attr_md.write_perm;
		
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
           
    return sd_ble_gatts_characteristic_add(p_lbe->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbe->LUX_2_handles);
}

static uint32_t lux_3_level_char_add(ble_lbe_t * p_lbe, const ble_lbe_init_t * p_lbe_init)
{
    
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
        
    
    // Add Battery Level characteristic
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
    
    ble_uuid.type = p_lbe->uuid_type;
    ble_uuid.uuid = LBE_UUID_LUX_SENS_3_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    //attr_md.read_perm  = p_lbe_init->light_level_char_attr_md.read_perm;
    //attr_md.write_perm = p_lbe_init->light_level_char_attr_md.write_perm;
		
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
           
    return sd_ble_gatts_characteristic_add(p_lbe->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbe->LUX_3_handles);
}
static uint32_t lux_4_level_char_add(ble_lbe_t * p_lbe, const ble_lbe_init_t * p_lbe_init)
{
     ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
        
    
    // Add Battery Level characteristic
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
    
    ble_uuid.type = p_lbe->uuid_type;
    ble_uuid.uuid = LBE_UUID_LUX_SENS_4_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    //attr_md.read_perm  = p_lbe_init->light_level_char_attr_md.read_perm;
    //attr_md.write_perm = p_lbe_init->light_level_char_attr_md.write_perm;
		
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
           
    return sd_ble_gatts_characteristic_add(p_lbe->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbe->LUX_4_handles);
}

static uint32_t Sample_rate_add(ble_lbe_t * p_lbe, const ble_lbe_init_t * p_lbs_init)
{
    ble_gatts_char_md_t char_md;
    //ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
        

    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.write =  1 ;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
	  char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;
    
    ble_uuid.type = p_lbe->uuid_type;
    ble_uuid.uuid = LBE_UUID_LUX_SAMPLE_RATE_CHAR;
    
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
    
    return sd_ble_gatts_characteristic_add(p_lbe->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_lbe->LUX_sample_handles);
}

uint32_t ble_lbe_init(ble_lbe_t * p_lbe, const ble_lbe_init_t * p_lbe_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
      //p_lbe->evt_handler               = p_lbe_init->evt_handler;
	    p_lbe->s_rate_write_handler      =p_lbe_init->s_rate_write_handler;
	    p_lbe->conn_handle               = BLE_CONN_HANDLE_INVALID;
      p_lbe->is_notification_supported = p_lbe_init->support_notification;
  
    
    
  // Add service
    ble_uuid128_t base_uuid = LBE_UUID_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lbe->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    ble_uuid.type = p_lbe->uuid_type;
    ble_uuid.uuid = LBE_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lbe->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    // Add Light level characteristic
	
		err_code = lux_1_level_char_add(p_lbe, p_lbe_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
	
	    err_code = lux_2_level_char_add(p_lbe, p_lbe_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		err_code = lux_3_level_char_add(p_lbe, p_lbe_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
		err_code = lux_4_level_char_add(p_lbe, p_lbe_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
  
		err_code = Sample_rate_add(p_lbe, p_lbe_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
			
		
		
    return NRF_SUCCESS;
}



/**@brief Function for encoding a Light Measurement.
 *
 * @param[in]   p_lbe              Light Band Exger Service structure.
 * @param[in]   p_lbe_meas         Measurement to be encoded.
 * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
 *
 * @return      Size of encoded data.
 */
static uint8_t lbe_measurement_encode(float float_light_level)
{
    uint8_t encoded_lux;
	
    encoded_lux = round(50*(log10(float_light_level)));

    return encoded_lux;
}

/**@brief Function for Updating  Light Measurement.
 *
 * @param[in]   p_ple              Light Band Exger Service structure.
 * @param[in]   p_lbe_meas         Pointer to new illuminance measurement..
 * 
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_lbe_LUX_1_update(ble_lbe_t * p_lbe, float light_level)
{
      uint32_t err_code = NRF_SUCCESS;
	    uint16_t               len;
			uint16_t               hvx_len;
			uint8_t                encoded_lbe_meas;
      
	   len=sizeof(uint8_t);
	
	   		
	 
			encoded_lbe_meas = lbe_measurement_encode(light_level);
			hvx_len = sizeof(encoded_lbe_meas);       
      len=hvx_len;
	
      err_code = sd_ble_gatts_value_set(p_lbe->LUX_1_handles.value_handle,
                                          0,
                                          &len,
                                          &encoded_lbe_meas);
	
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }  
	      	
	      // Send value if connected and notifying
        if ((p_lbe->conn_handle != BLE_CONN_HANDLE_INVALID) && p_lbe->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            
						
            
					  memset(&hvx_params, 0, sizeof(hvx_params));
                
            hvx_params.handle   = p_lbe->LUX_1_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &hvx_len;
            hvx_params.p_data   = &encoded_lbe_meas;
            
            err_code = sd_ble_gatts_hvx(p_lbe->conn_handle, &hvx_params);
								if ((err_code == NRF_SUCCESS) && (hvx_len != len))
						{
								err_code = NRF_ERROR_DATA_SIZE;
						}
				}
				else
				{
						err_code = NRF_ERROR_INVALID_STATE;
				}

    return err_code;
}

uint32_t ble_lbe_LUX_2_update(ble_lbe_t * p_lbe, float light_level)
{
      uint32_t err_code = NRF_SUCCESS;
	    uint16_t               len;
			uint16_t               hvx_len;
			uint8_t                encoded_lbe_meas2;
      
	   len=sizeof(uint8_t);
	
	   //uint8_t                encoded_lbe_meas[MAX_HTM_LEN];			
	 
			encoded_lbe_meas2 = lbe_measurement_encode(light_level);
			hvx_len = sizeof(encoded_lbe_meas2);       
      len=hvx_len;
	
      err_code = sd_ble_gatts_value_set(p_lbe->LUX_2_handles.value_handle,
                                          0,
                                          &len,
                                          &encoded_lbe_meas2);
	
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }  
	      	
	      // Send value if connected and notifying
        if ((p_lbe->conn_handle != BLE_CONN_HANDLE_INVALID) && p_lbe->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            
						
            
					  memset(&hvx_params, 0, sizeof(hvx_params));
                
            hvx_params.handle   = p_lbe->LUX_2_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &hvx_len;
            hvx_params.p_data   = &encoded_lbe_meas2;
            
            err_code = sd_ble_gatts_hvx(p_lbe->conn_handle, &hvx_params);
								if ((err_code == NRF_SUCCESS) && (hvx_len != len))
						{
								err_code = NRF_ERROR_DATA_SIZE;
						}
				}
				else
				{
						err_code = NRF_ERROR_INVALID_STATE;
				}

    return err_code;
}


uint32_t ble_lbe_LUX_3_update(ble_lbe_t * p_lbe, float light_level)
{
      uint32_t err_code = NRF_SUCCESS;
	    uint16_t               len;
			uint16_t               hvx_len;
			uint8_t                encoded_lbe_meas;
      
	   len=sizeof(uint8_t);
	
	   		
	 
			encoded_lbe_meas = lbe_measurement_encode(light_level);
			hvx_len = sizeof(encoded_lbe_meas);       
      len=hvx_len;
	
      err_code = sd_ble_gatts_value_set(p_lbe->LUX_3_handles.value_handle,
                                          0,
                                          &len,
                                          &encoded_lbe_meas);
	
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }  
	      	
	      // Send value if connected and notifying
        if ((p_lbe->conn_handle != BLE_CONN_HANDLE_INVALID) && p_lbe->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            
						
            
					  memset(&hvx_params, 0, sizeof(hvx_params));
                
            hvx_params.handle   = p_lbe->LUX_3_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &hvx_len;
            hvx_params.p_data   = &encoded_lbe_meas;
            
            err_code = sd_ble_gatts_hvx(p_lbe->conn_handle, &hvx_params);
								if ((err_code == NRF_SUCCESS) && (hvx_len != len))
						{
								err_code = NRF_ERROR_DATA_SIZE;
						}
				}
				else
				{
						err_code = NRF_ERROR_INVALID_STATE;
				}

    return err_code;
}

uint32_t ble_lbe_LUX_4_update(ble_lbe_t * p_lbe, float light_level)
{
      uint32_t err_code = NRF_SUCCESS;
	    uint16_t               len;
			uint16_t               hvx_len;
			uint8_t                encoded_lbe_meas;
      
	   len=sizeof(uint8_t);
	
	   		
	 
			encoded_lbe_meas = lbe_measurement_encode(light_level);
			hvx_len = sizeof(encoded_lbe_meas);       
      len=hvx_len;
	
      err_code = sd_ble_gatts_value_set(p_lbe->LUX_4_handles.value_handle,
                                          0,
                                          &len,
                                          &encoded_lbe_meas);
	
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }  
	      	
	      // Send value if connected and notifying
        if ((p_lbe->conn_handle != BLE_CONN_HANDLE_INVALID) && p_lbe->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            
						
            
					  memset(&hvx_params, 0, sizeof(hvx_params));
                
            hvx_params.handle   = p_lbe->LUX_4_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &hvx_len;
            hvx_params.p_data   = &encoded_lbe_meas;
            
            err_code = sd_ble_gatts_hvx(p_lbe->conn_handle, &hvx_params);
								if ((err_code == NRF_SUCCESS) && (hvx_len != len))
						{
								err_code = NRF_ERROR_DATA_SIZE;
						}
				}
				else
				{
						err_code = NRF_ERROR_INVALID_STATE;
				}

    return err_code;
}
/*
uint32_t ble_lbe_LUX_1_update(ble_lbe_t * p_lbe, float light_level)
{
      uint32_t err_code = NRF_SUCCESS;
	    uint16_t               len;
			uint16_t               hvx_len;
			uint8_t                encoded_lbe_meas;
      
	   len=sizeof(uint8_t);
	
	   //uint8_t                encoded_lbe_meas[MAX_HTM_LEN];			
	 
			encoded_lbe_meas = lbe_measurement_encode(light_level);
			hvx_len = sizeof(encoded_lbe_meas);       
      len=hvx_len;
	
      err_code = sd_ble_gatts_value_set(p_lbe->LUX_1_handles.value_handle,
                                          0,
                                          &len,
                                          &encoded_lbe_meas);
	
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }  
	      	
	      // Send value if connected and notifying
        if ((p_lbe->conn_handle != BLE_CONN_HANDLE_INVALID) && p_lbe->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;
            
						
            
					  memset(&hvx_params, 0, sizeof(hvx_params));
                
            hvx_params.handle   = p_lbe->LUX_1_handles.value_handle;
            hvx_params.type     = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset   = 0;
            hvx_params.p_len    = &hvx_len;
            hvx_params.p_data   = &encoded_lbe_meas;
            
            err_code = sd_ble_gatts_hvx(p_lbe->conn_handle, &hvx_params);
								if ((err_code == NRF_SUCCESS) && (hvx_len != len))
						{
								err_code = NRF_ERROR_DATA_SIZE;
						}
				}
				else
				{
						err_code = NRF_ERROR_INVALID_STATE;
				}

    return err_code;
}*/


uint32_t ble_lbe_Sample_rate_update(ble_lbe_t * p_lbe, uint8_t Sample_rate)
{
    ble_gatts_hvx_params_t hvx_params;
    uint16_t len = sizeof(Sample_rate);
    
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.handle = p_lbe->LUX_sample_handles.value_handle;
    hvx_params.p_data = &Sample_rate;
    hvx_params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_lbe->conn_handle, &hvx_params);
 
}


