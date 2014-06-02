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
 * @defgroup ble_sdk_srv_bas time Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief time Service module.
 *
 * @details This module implements the time Service with the time Level characteristic.
 *          During initialization it adds the time Service and time Level characteristic
 *          to the BLE stack database. Optionally it can also add a Report Reference descriptor
 *          to the time Level characteristic (used when including the time Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the time Level characteristic
 *          through the ble_time_time_update() function.
 *          If an event handler is supplied by the application, the time Service will
 *          generate time Service events to the application.
 *
 * @note The application must propagate BLE stack events to the time Service module by calling
 *       ble_time_on_ble_evt() from the from the @ref ble_stack_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_TIME_H__
#define BLE_TIME_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"


// BASE UUID : 99 ce ea 41-95 49-49 95-80 b8-5c 4d 5f 3b b8 31
//             F7-44-61C2-E9F5-11E3-AC10-08-00-20-0C-9A-66

#define TIME_UUID_BASE {0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x10, 0xAC, 0xE3, 0x11, 0xF5, 0xE9, 0x00, 0x00, 0x44, 0xF7}
#define TIME_UUID_SERVICE 0x61C2	
#define TIME_UUID_CURRENT_TIME_HIGH_CHAR 0x61C3
#define TIME_UUID_CURRENT_TIME_LOW_CHAR  0x61C4


/**@brief time Service event type. */
typedef enum
{
    BLE_TIME_EVT_NOTIFICATION_ENABLED,                             /**< time value notification enabled event. */
    BLE_TIME_EVT_NOTIFICATION_DISABLED                             /**< time value notification disabled event. */
} ble_time_evt_type_t;

/**@brief time Service event. */
typedef struct
{
    ble_time_evt_type_t evt_type;                                  /**< Type of event. */
} ble_time_evt_t;

// Forward declaration of the ble_time_t type. 
typedef struct ble_time_s ble_time_t;

/**@brief time Service event handler type. */
typedef void (*ble_time_evt_handler_t) (ble_time_t * p_time, ble_time_evt_t * p_evt);

/**  event write handler for time (HIGH byte) service*/
typedef void (*ble_time_high_write_handler_t)(ble_time_t * p_time,uint8_t new_time);
	
/**  event write handler for time (LOW) service*/
typedef void (*ble_time_low_write_handler_t)(ble_time_t * p_time,uint8_t new_time);

/**@brief time Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_time_evt_handler_t        			 evt_handler;                    /**< Event handler to be called for handling events in the time Service. */
    ble_time_high_write_handler_t       time_high_write_handler;         /**< Event handler for time write   */
    ble_time_low_write_handler_t        time_low_write_handler;  
		uint16_t                      			initial_time_on_start;
} ble_time_init_t;

/**@brief time Service structure. This contains various status information for the service. */
typedef struct ble_time_s
{
    uint16_t                      service_handle;                 /**< Handle of time Service (as provided by the BLE stack). */  
	  ble_time_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the time Service. */
    ble_gatts_char_handles_t      time_high_char_handles;          /**< Handles related to the time Level characteristic. */
	  ble_gatts_char_handles_t      time_low_char_handles;          /**< Handles related to the time Level characteristic. */
    uint16_t                       time_last;             /**< Last time Level measurement passed to the time Service. */
    
	  uint8_t                	       uuid_type;
	  uint16_t                       conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    ble_time_high_write_handler_t       time_high_write_handler;         /**< Event handler for time write   */
	  ble_time_low_write_handler_t        time_low_write_handler;
} ble_time_t;

/**@brief Function for initializing the time Service.
 *
 * @param[out]  p_time       time Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_time_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_time_init(ble_time_t * p_time, const ble_time_init_t * p_time_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the time Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_time_time_update() must be called upon reconnection if the
 *       time level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_time      time Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_time_on_ble_evt(ble_time_t * p_time, ble_evt_t * p_ble_evt);

/**@brief Function for updating the time level.
 *
 * @details The application calls this function after having performed a time measurement. If
 *          notification has been enabled, the time level characteristic is sent to the client.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       this function must be called upon reconnection if the time level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_time          time Service structure.
 * @param[in]   time  New time measurement value (in percent of full capacity).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_time_time_high_update(ble_time_t * p_time, uint8_t time);

uint32_t ble_time_time_low_update(ble_time_t * p_time, uint8_t time);

#endif // BLE_TIME_H__

/** @} */
