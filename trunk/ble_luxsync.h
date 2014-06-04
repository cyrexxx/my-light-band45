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

/** @file
 *
 * @defgroup ble_sdk_srv_bas LUX Sync Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief LUX Sync Service module.
 *
 * @details This module implements the LUX Sync Service with the LUX Sync Level characteristic.
 *          During initialization it adds the LUX Sync Service and LUX Sync Level characteristic
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the LUX Sync Level characteristic (used when including the LUX Sync Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the LUX Sync Level characteristic
 *          through the ble_luxsync_LUX Sync_level_update() function.
 *          If an event handler is supplied by the application, the LUX Sync Service will
 *          generate LUX Sync Service events to the application.
 *
 * @note The application must propagate BLE stack events to the LUX Sync Service module by calling
 *       ble_luxsync_on_ble_evt() from the from the @ref ble_stack_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_LUXSYNC_H__
#define BLE_LUXSYNC_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

// BASE UUID : 1d-1d-3c-ac-23-c6-4d-db-ba-71-4c-7e-68-20-ae-39
#define LuxSync_UUID_BASE {0x39, 0xAE, 0x20, 0x68, 0x7E, 0x4C, 0x71, 0xBA, 0xDB, 0x4D, 0xC6, 0x23, 0x00, 0x00, 0x1D, 0x1D}
#define LuxSync_UUID_SERVICE 0x3CAC
#define LuxSync_UUID_SYNC_CHAR 0x3CAD
#define LuxSync_UUID_SYNC_ACK_CHAR 0x3CAE 


/**@brief Light Service event type. */
typedef enum
{
    BLE_LUXSYNC_EVT_NOTIFICATION_ENABLED,                             /**< LUX Sync value notification enabled event. */
    BLE_LUXSYNC_EVT_NOTIFICATION_DISABLED                             /**< LUX Sync value notification disabled event. */
}  ble_luxsync_evt_type_t;

/**@brief LUX Sync Service event. */
typedef struct
{
    ble_luxsync_evt_type_t evt_type;                                  /**< Type of event. */
} ble_luxsync_evt_t;


typedef struct
	{
		ble_luxsync_evt_type_t  lux_sync_notification_st;
    ble_luxsync_evt_type_t  lux_sync_act_notification_st;
		
	} luxsync_char_notifications_t;

// Forward declaration of the ble_luxsync_t type. 
typedef struct ble_luxsync_s ble_luxsync_t;

/**@brief LUX Sync Service event handler type. */
typedef void (*ble_luxsync_evt_handler_t) (ble_luxsync_t * p_luxsync, ble_luxsync_evt_t * p_evt);

/**@brief Sample rate event handler type. */
typedef void (*ble_luxsync_ack_write_handler_t) (ble_luxsync_t * p_luxsync, uint8_t new_rate);

/**@brief LUX Sync Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_luxsync_evt_handler_t     evt_handler;                    /**< Event handler to be called for handling events in the LUX Sync Service. */
    ble_luxsync_ack_write_handler_t  Luxsync_ack_write_handler;
    bool                          support_notification;           /**< TRUE if notification of LUX Sync Level measurement is supported. */
    //ble_srv_cccd_security_mode_t  LUX_Sync_level_char_attr_md;     /**< Initial security level for LUX Sync characteristics attribute */
    //ble_gap_conn_sec_mode_t       LUX_Sync_level_report_read_perm; /**< Initial security level for LUX Sync report read attribute */
} ble_luxsync_init_t;

/**@brief LUX Sync Service structure. This contains various status information for the service. */
typedef struct ble_luxsync_s
{
    ble_luxsync_evt_handler_t     evt_handler;                    /**< Event handler to be called for handling events in the LUX Sync Service. */
    uint16_t                      service_handle;                 /**< Handle of LUX Sync Service (as provided by the BLE stack). */
    
	  ble_gatts_char_handles_t      LuxSync_handles;
    ble_gatts_char_handles_t      LuxSync_ACK_handles;          /**< Handles related to the LUX Sync Level characteristic. */
    
	  ble_luxsync_ack_write_handler_t  Luxsync_ack_write_handler;
    uint8_t                       Lux_Ack;                     
    uint8_t                       uuid_type;
	
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of LUX Sync Level is supported. */
    
} ble_luxsync_t;


/**@brief Function for initializing the LUX Sync Service.
 *
 * @param[out]  p_luxsync       LUX Sync Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_luxsync_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_luxsync_init(ble_luxsync_t * p_luxsync, const ble_luxsync_init_t * p_luxsync_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the LUX Sync Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_luxsync_LUX Sync_level_update() must be called upon reconnection if the
 *       LUX Sync level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_luxsync      LUX Sync Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_luxsync_on_ble_evt(ble_luxsync_t * p_luxsync, ble_evt_t * p_ble_evt);


/**@brief Function for writing from memory.
 *
 * @details The application calls this function after it recievs a command over ACK . If
 *          notification has been enabled, the Memory contents are sent to the client one by one.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       this function must be called upon reconnection if the LUX Sync level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_luxsync          LUXSYNC Service structure.
 * @param[in]   mem_data      Data from memory .
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_luxsync_write_update(ble_luxsync_t * p_luxsync, uint8_t *mem_data,uint8_t length);


/**@brief Function for Syncing the data in memory.
 *
 * @details The application and client calls this function when if wants to exchange  
 *           hand-shake signals with the Client .  
 *           notification has been enabled, so the data is sent to the client.
 *
 * @note       0x00 - No sync
 *               0x1 - set my client to start sync
 *							 0x2 - data transmission begin
 *							 0x3 - Data tranmission over turn notification OFF on luxsync
                 0x4 - data uploaded erase memory.
                 
 *
 * @param[in]   p_luxsync          LUX Sync Service structure.
 * @param[in]   sync_ack  New LUX Sync measurement value (in percent of full capacity).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_luxsync_ACK_update(ble_luxsync_t * p_luxsync, uint8_t Lux_Ack);

void send_data_stream(ble_luxsync_t * p_luxsync,uint32_t present_mem_ptr);
void upload_done(ble_luxsync_t * p_luxsync);

#endif // BLE_LUXSYNC_H__

/** @} */
