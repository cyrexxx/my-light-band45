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
 * @defgroup ble_sdk_srv_lbe Light Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Battery Service module.
 *
 * @details This module implements the Light Service with the Light Level characteristic.
 *          During initialization it adds the Light Service and Light Level characteristic
 *          to the BLE stack database. 
 *
 *          If specified, the module will support notification of the Light Level characteristic
 *          through the ble_lbe_battery_level_update() function.
 *          If an event handler is supplied by the application, the Light Service will
 *          generate Light Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Light Service module by calling
 *       ble_lse_on_ble_evt() from the from the @ref ble_stack_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */
 

#ifndef BLE_LBE_H__
#define BLE_LBE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

// BASE UUID : 99 ce ea 41-95 49-49 95-80 b8-5c 4d 5f 3b b8 31
#define LBE_UUID_BASE {0x31, 0xB8, 0x3B, 0x5F, 0x4D, 0x5C, 0xB8, 0x80, 0x95, 0x49, 0x49, 0x95, 0x00, 0x00, 0xCE, 0x99}
#define LBE_UUID_SERVICE 0x8501
#define LBE_UUID_LUX_SENS_1_CHAR 0x8502
#define LBE_UUID_LUX_SENS_2_CHAR 0x8503
#define LBE_UUID_LUX_SENS_3_CHAR 0x8504
#define LBE_UUID_LUX_SENS_4_CHAR 0x8505 
#define LBE_UUID_LUX_SAMPLE_RATE_CHAR 0x8506

/**@brief LiLPg\ht Service event type. */
typedef enum
	{
			BLE_LBE_EVT_NOTIFICATION_ENABLED,                             /**< Battery value notification enabled event. */
			BLE_LBE_EVT_NOTIFICATION_DISABLED                             /**< Battery value notification disabled event. */
	} ble_lbe_evt_type_t;

/**@brief Battery Service event. */
typedef struct
	{
			ble_lbe_evt_type_t evt_type;                                  /**< Type of event. */
	} ble_lbe_evt_t;

	
	
// Forward declaration of the ble_lbe_t type. 
typedef struct ble_lbe_s ble_lbe_t;

/**@brief Light Service event handler type. */
typedef void (*ble_lbe_evt_handler_t) (ble_lbe_t * p_lbe, ble_lbe_evt_t * p_evt);

/**@brief Sample rate event handler type. */
typedef void (*ble_lbe_s_rate_write_handler_t) (ble_lbe_t * p_lbe, uint8_t new_rate);

/**@brief FLOAT format (IEEE-11073 32-bit FLOAT, defined as a 32-bit value with a 24-bit mantissa
 *        and an 8-bit exponent. */
typedef struct
	{
		ble_lbe_evt_type_t  lux_1_notification_st;
		ble_lbe_evt_type_t  lux_2_notification_st;
		ble_lbe_evt_type_t  lux_3_notification_st;
		ble_lbe_evt_type_t  lux_4_notification_st;
		
	} lbe_char_notifications_t;
	


	
typedef struct ble_lbe_float_meas_s
	{
			float       light_level;                       /**< Light Reading In LUX */

} ble_lbe_float_meas_t;

/**@brief Light Band Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
	{
			  ble_lbe_evt_handler_t            evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
			  ble_lbe_s_rate_write_handler_t  s_rate_write_handler;         /**< Event handler for the sample rate   */
		    bool                            support_notification;           /**< TRUE if notification of Battery Level measurement is supported. */
			  
			//ble_srv_cccd_security_mode_t  light_level_char_attr_md;     /**< Initial security level for battery characteristics attribute */
			//ble_gap_conn_sec_mode_t       light_level_report_read_perm;  /**< Initial security level for battery report read attribute */
		 
		
	} ble_lbe_init_t;

/**@brief Light band Service structure. This contains various status information for the service. */
typedef struct ble_lbe_s
	{
			ble_lbe_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
			uint16_t                      service_handle;                 /**< Handle of  Service (as provided by the BLE stack). */
			
			ble_gatts_char_handles_t      LUX_1_handles;          			  /**< Handles related to the Light Level characteristic. */
			ble_gatts_char_handles_t      LUX_2_handles;         				  /**< Handles related to the Light Level characteristic. */
			ble_gatts_char_handles_t      LUX_3_handles;         				  /**< Handles related to the Light Level characteristic. */
			ble_gatts_char_handles_t      LUX_4_handles;         					/**< Handles related to the Batte      ry Level characteristic. */
			ble_gatts_char_handles_t      LUX_sample_handles;        		  /**< Handles related to the Light Level characteristic. */
			uint8_t                       sample_rate;
			
			uint8_t                       uuid_type;	
			uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
			bool                          is_notification_supported;      /**< TRUE if notification of Light Level is supported. */
			ble_lbe_s_rate_write_handler_t  s_rate_write_handler;
			
	} ble_lbe_t;



/**@brief Function for initializing the Light Service.
 *
 * @param[out]  p_lbe       Light Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_lbe_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_lbe_init(ble_lbe_t * p_lbe, const ble_lbe_init_t * p_lbe_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note For the requirements in the BAS specification to be fulfilled,
 *       ble_lbe_battery_level_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_lbe      Battery Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */

void ble_lbe_on_ble_evt(ble_lbe_t * p_lbe, ble_evt_t * p_ble_evt);


/**@brief Function for updating the Lux Sensor 1.
 *
 * @details The application calls this function after having performed a Light Level Measurement. If
 *          notification has been enabled, the Light level characteristic is sent to the client.
 *
 * @note For the requirements in the Light specification to be fulfilled,
 *       this function must be called upon reconnection if the Light level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_lbe          Light Service structure.
 * @param[in]   p_lbe_meas    Pointer to new illuminance measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_lbe_LUX_1_update(ble_lbe_t * p_lbe, uint8_t light_level);
//uint32_t ble_lbe_LUX_1_update(ble_lbe_t * p_lbe, ble_lbe_meas_t * p_lbe_meas);


/**@brief Function for updating the Lux Sensor 2.
 *
 * @details The application calls this function after having performed a Light Level Measurement. If
 *          notification has been enabled, the Light level characteristic is sent to the client.
 *
 * @note For the requirements in the Light specification to be fulfilled,
 *       this function must be called upon reconnection if the Light level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_lbe          Light Service structure.
 * @param[in]   p_lbe_meas    Pointer to new illuminance measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_lbe_LUX_2_update(ble_lbe_t * p_lbe, uint8_t light_level);

/**@brief Function for updating the Lux Sensor 1.
 *
 * @details The application calls this function after having performed a Light Level Measurement. If
 *          notification has been enabled, the Light level characteristic is sent to the client.
 *
 * @note For the requirements in the Light specification to be fulfilled,
 *       this function must be called upon reconnection if the Light level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_lbe          Light Service structure.
 * @param[in]   p_lbe_meas    Pointer to new illuminance measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_lbe_LUX_3_update(ble_lbe_t * p_lbe, uint8_t light_level);

/**@brief Function for updating the Lux Sensor 4.
 *
 * @details The application calls this function after having performed a Light Level Measurement. If
 *          notification has been enabled, the Light level characteristic is sent to the client.
 *
 * @note For the requirements in the Light specification to be fulfilled,
 *       this function must be called upon reconnection if the Light level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_lbe          Light Service structure.
 * @param[in]   p_lbe_meas    Pointer to new illuminance measurement.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_lbe_LUX_4_update(ble_lbe_t * p_lbe, uint8_t light_level);


/**@brief Function for updating the Sample rate.
 *
 * @details The application calls this function If it intends to change the sampling rate . If
 *          notification has been enabled, the battery level characteristic is sent to the client.
 *
 * @note  
 *
 * @param[in]   p_lbe          Sample rate  Service structure.
 * @param[in]   Sample_rate    new sampling frequency value (in Hz ).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_lbe_Sample_rate_update(ble_lbe_t * p_lbe, uint8_t Sample_rate);

#endif //BLE_BLE_H_

/** @} */
