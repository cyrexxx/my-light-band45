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
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"


#include "ble_lbe.h"
#include "ble_luxsync.h"
#include "ble_time.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"


#include "simple_uart.h"
//#include "flash_rw.h"
//#include "ble_flash.h"

#include "ble_dis.h"
//dasibel for custom HW
#include "cy_io.h"
#include "ble_date_time.h"
//#include "boards.h"
//For sumlated sensor value

#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "nrf_delay.h"
//#include "ble_error_log.h"

#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "max_44009.h"
#include "m24m02.h"

#include "ble_sensorsim.h"

#include "defines.h"


#define WAKEUP_BUTTON_PIN               BUTTON_0                                    /**< Button used to wake up the application. */
// YOUR_JOB: Define any other buttons to be used by the applications:
// #define MY_BUTTON_PIN                   BUTTON_1

#define ADVERTISING_LED_PIN_NO          LED_1                                       /**< Is on when device is advertising. */
#define CONNECTED_LED_PIN_NO            LED_0                                       /**< Is on when device has connected. */
#define ASSERT_LED_PIN_NO               LED_2                                       /**< Is on when application has asserted. */

#define DEVICE_NAME                     "Light Band"                               /**< Name of device. Will be included in the advertising data. */

//Device Information Service 
#define MANUFACTURER_NAME               "Exeger Sweden AB"                                   /**< Manufacturer. Will be passed to Device Information Service. */
#define SERIAL_NUMBER                   "(942)"                                      /**< Serial Number of the device.Will be passed to Device Information Service. */
#define HW_REVISION              				"V 2.5 (June,24 2014)"																		 /**< Hardware Version of the device.Will be passed to Device Information Service. */
#define FW_REVISION                     "V 3.0"																		  /**< Firmware Revision of the device.Will be passed to Device Information Service. */
#define MODEL_NUMBER                    "Kartik.karuna@exeger.com" 

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

// YOUR_JOB: Modify these according to requirements.
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            4                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10/*500*/, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40/*1000*/, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(300/*4000*/, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               300                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
//static ble_gap_adv_params_t             m_adv_params;                          /**< Parameters to be passed to the stack when starting advertising. */
//Variables for flash


//for Sumating sensor values 
static bool                                  m_lux_meas_ind_conf_pending = false;       /**< Flag to keep track of when an indication confirmation is pending. */
//static ble_sensorsim_cfg_t                   m_LUX_sim_cfg;                    /**< Temperature simulator configuration. */
//static ble_sensorsim_state_t                 m_LUX_sim_state;                  /**< Temperature simulator state. */
static app_timer_id_t                         m_lux_timer_id;
#define lux_LEVEL_MEAS_INTERVAL               APP_TIMER_TICKS(10000, APP_TIMER_PRESCALER) /**< Lux level measurement interval (ticks). now set to every 2 sec ,modift later for 10 sec*/ 

static app_timer_id_t                         m_mem_write_id;
#define MEM_WRITE_MEAS_INTERVAL               APP_TIMER_TICKS(22000, APP_TIMER_PRESCALER) /**< Lux level measurement interval (ticks). now set to every 2 sec ,modift later for 10 sec*/ 



static ble_lbe_t                        m_lbe;
static ble_luxsync_t       							m_luxsync;
static ble_time_t                       m_current_time;
static light_reading_t                  m_lux_values;               // Lux values in Float 
static encoded_light_reading_t          m_encoded_light_reading;   // Encoded lux values 
static ble_date_time_t                  m_current_date_time_value;
uint8_t date_timebuffer[3];   //Buffer to store encoded date time values
uint8_t write_to_mem_buffer[40];//[128]; //Buffer to save reading and timestamp
//uint32_t mem_pointer = 0x0;   /// pointer to keep  track of eeprom memory location 
uint8_t buffpointer=0;

// YOUR_JOB: Modify these according to requirements (e.g. if other event types are to pass through
//           the scheduler).
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

uint8_t sendDataFlag = 0;
extern bool flag_send_data;

uint8_t fetchMoreData = 0;


char print_buffer[200];
#ifdef DEBUG
#define print(...) { sprintf(print_buffer, __VA_ARGS__); simple_uart_putstring(print_buffer);}
#else
#define print(...)
#endif 











// Persistent storage system event handler
void pstorage_sys_event_handler (uint32_t p_evt);

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_clear(ASSERT_LED_PIN_NO);
	  nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
	  nrf_delay_ms(500);
	
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
     ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
    //NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
/*
// YOUR_JOB: Uncomment this function and make it handle error situations sent back to your
//           application by the services it uses.
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
} */

  


/**@brief Function for  sending one Light Measurement. This Function is called evey 10 seconds
 */
static void lux_measurement_send(void)
{
   uint32_t       err_code;
	
	 read_sensors(&m_lux_values);
	 encode_sensor_value(&m_lux_values,&m_encoded_light_reading);  // Encode the lux values from Float to log 
    if (m_lux_meas_ind_conf_pending==true)
		{			
	   		    
      
			err_code = ble_lbe_LUX_1_update(&m_lbe, &m_encoded_light_reading); 
        switch (err_code)
        {
            case NRF_SUCCESS:
                // Measurement was successfully sent, wait for confirmation.
                //m_lux_meas_ind_conf_pending = true;
                break;
            case BLE_ERROR_GATTS_SYS_ATTR_MISSING:
							  // Ignore error.
                break;
						
            case NRF_ERROR_INVALID_STATE:
                // Ignore error.
                break;

            default:
                APP_ERROR_HANDLER(err_code);
                break;
        }
			 
		}		
}

void append_mem_buffer(encoded_light_reading_t *p_encoded_light_reading,uint8_t *p_write_to_mem_buffer,uint8_t buff_pntr )
{
	uint8_t i;
	for(i=0;i<3;i++)
	{
  *(p_write_to_mem_buffer+buff_pntr+i)=date_timebuffer[i];
	}
	*(p_write_to_mem_buffer+buff_pntr+3)=p_encoded_light_reading->u4_top;
	*(p_write_to_mem_buffer+buff_pntr+4)=p_encoded_light_reading->u5_up;
	*(p_write_to_mem_buffer+buff_pntr+5)=p_encoded_light_reading->u6_down_1;
	*(p_write_to_mem_buffer+buff_pntr+6)=p_encoded_light_reading->u7_down_2;
}
//Time out handler for the m_lux_timer_id timer 

static void lux_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    update_current_time(&m_current_time,&m_current_date_time_value,date_timebuffer);
	  lux_measurement_send();
	if (m_lux_meas_ind_conf_pending==false)
	{
	  append_mem_buffer(&m_encoded_light_reading,write_to_mem_buffer,buffpointer);
	  buffpointer+=7;
	  if(buffpointer>40){buffpointer=0;}
		uint32_t err_code = sd_app_evt_wait();
	  APP_ERROR_CHECK(err_code);
	}
	  
	//fetchMoreData = 1;
	print("TIMER\r\n");
	 if(flag_send_data)
	 { 
		 send_data_stream_ble(&m_luxsync, 1, 0);
		 
	 }
	
/*	if ( sendDataFlag > 0 )
	{
		sendDataFlag = 0;
	
		if(flag_send_data)
		{ 
			 send_data_stream_ble(&m_luxsync, 1, 0);
		}
	}*/
}

//Time out handler for the m_lux_timer_id timer 

static void mem_write_timeout_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
	if (m_lux_meas_ind_conf_pending==false)
	{ 
	 print("Saving2mem\r\n");
	 nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO); // turn on light blue led (B+G) to signal advertisement and also start mem write
	 nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
		uint32_t mem_pointer=eeprom_find_add_pointer();
		print("eeeprom add %d \r\n",mem_pointer);
	 uint8_t length = buffpointer;
		
   i2c_eeprom_write(mem_pointer, write_to_mem_buffer,length);
	
	 mem_pointer+=length;
	 buffpointer=0;
	 eeprom_updateadd_pointer((uint32_t)mem_pointer);
	
	  mem_pointer =0;
	 nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO); // turn on light blue led (B+G) to signal advertisement and also end mem write
	 nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
	 uint32_t err_code = sd_app_evt_wait();
	 APP_ERROR_CHECK(err_code);
		
	}
	 //eeprom_updateadd_pointer(mem_pointer);
	
	
}

/**@brief Function for stopping timers.
*/
static void lux_timers_stop(void)
{
	//add all times that are started so that they can be paused while mem operation
	uint32_t err_code;	
	err_code =app_timer_stop(m_lux_timer_id);
	APP_ERROR_CHECK(err_code);
	err_code =app_timer_stop(m_mem_write_id);
	APP_ERROR_CHECK(err_code);
	
}

/**@brief Function for starting timers.
*/
static void timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
     */
	  uint32_t err_code;

    err_code = app_timer_start(m_lux_timer_id, lux_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_mem_write_id, MEM_WRITE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);	
	
}

/** Function to handle Write requests to Sync ACK char

*/
static void Luxsync_ack_write_handler(ble_luxsync_t * p_luxsync, uint8_t ACK_data)
{
  switch(ACK_data)
	{		
		case 1:                               
			    //Phone ready to recieve data
		      //Change ACK value to 2
		      //lux_timers_stop();
		      
		      send_data_stream(p_luxsync);//send_data_stream(&m_luxsync,mem_pointer);
		      //timers_start();
		break;	
		//case 2:
			  //Sending data to device
       
		//case 4:
			  //end of data stream;
		case 9:
			//Erase memory
		  upload_done(p_luxsync);
			 break;
		
		default:
			   break;
	}

}
/** Function to handle Write requests to Byte2 time byte

*/
static void ble_time_byte2_write_handler(ble_time_t * p_time , uint8_t new_time)
{
  // decode value & call update time function 
	date_timebuffer[0] = new_time;
}

/** Function to handle Write requests to byte1 time byte

*/
static void ble_time_byte1_write_handler(ble_time_t * p_time , uint8_t new_time)
{
  // decode value & call update time function 
	 date_timebuffer[1] = new_time;
}
/** Function to handle Write requests to byte1 time byte

*/
static void ble_time_byte0_write_handler(ble_time_t * p_time , uint8_t new_time)
{
  // decode value & call update time function 
	date_timebuffer[2] = new_time;
	cy_date_time_decode(&m_current_date_time_value,date_timebuffer);
}

/**@brief Function for the power bus initialization.
 *
 * @details Initializes power bussses to be used by the I2C bus.
 */

static void pw_bus_init(void)
{
    nrf_gpio_cfg_output(i2c_vcc1);
    nrf_gpio_pin_clear(i2c_vcc1);  
	  nrf_gpio_cfg_output(i2c_vcc2);
    nrf_gpio_pin_clear(i2c_vcc2);
	  nrf_gpio_cfg_output(MEM_EN );   //EEPROM Enable
    nrf_gpio_pin_clear(MEM_EN );
	  nrf_gpio_cfg_output(MEM_WC);    //EEPROM WC
    nrf_gpio_pin_clear(MEM_WC);
    

}
/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
    nrf_gpio_cfg_output(ASSERT_LED_PIN_NO);
	  nrf_gpio_pin_set(LED_0);
    nrf_gpio_pin_set(LED_1);
    nrf_gpio_pin_set(LED_2);

    // YOUR_JOB: Add additional LED initialiazations if needed.
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{  uint32_t err_code;

    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS,
                             	APP_TIMER_OP_QUEUE_SIZE, true);

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, 
	                                                timer_timeout_handler);
    APP_ERROR_CHECK(err_code); */
	   err_code = app_timer_create(&m_lux_timer_id,APP_TIMER_MODE_REPEATED,
                                lux_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_mem_write_id,APP_TIMER_MODE_REPEATED,
                                mem_write_timeout_handler);
    APP_ERROR_CHECK(err_code); 	
	
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);
	  
	 //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.*/
																					
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_WATCH_SPORTS_WATCH);
    APP_ERROR_CHECK(err_code); 

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
  	ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    // YOUR_JOB: Use UUIDs for service(s) used in your application.
    ble_uuid_t adv_uuids[] = {{LBE_UUID_SERVICE, m_lbe.uuid_type}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
		
		memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t      		 	err_code;
	  ble_dis_init_t 			dis_init;
	  ble_lbe_init_t 			lbe_init;
	  ble_luxsync_init_t  luxsync_init;
	  ble_time_init_t     current_time_init;
	
	  
	  
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
	  ble_srv_ascii_to_utf8(&dis_init.serial_num_str, (char *)SERIAL_NUMBER);
     ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, (char *)HW_REVISION);
	  ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char *)FW_REVISION);
	  ble_srv_ascii_to_utf8(&dis_init.model_num_str, (char *)MODEL_NUMBER);
	 
	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
	
	//Initialize Light band service 
	
	//lbe_init.evt_handler = on_lbe_evt;   
	lbe_init.evt_handler = NULL;
	lbe_init.support_notification = true;
	err_code = ble_lbe_init(&m_lbe, &lbe_init);
  APP_ERROR_CHECK(err_code);
	
	//Initialize sync service 
	memset(&luxsync_init, 0, sizeof(luxsync_init));
	luxsync_init.support_notification=true;
	luxsync_init.Luxsync_ack_write_handler = Luxsync_ack_write_handler;
  err_code = ble_luxsync_init(&m_luxsync, &luxsync_init);
  APP_ERROR_CHECK(err_code);
	
	//Initialize Time service
	//m_current_time
	memset(&current_time_init, 0, sizeof(current_time_init));
	current_time_init.time_byte2_write_handler =ble_time_byte2_write_handler;
	current_time_init.time_byte1_write_handler =ble_time_byte1_write_handler;
	current_time_init.time_byte0_write_handler =ble_time_byte0_write_handler;
	err_code = ble_time_init(&m_current_time, &current_time_init);
  APP_ERROR_CHECK(err_code);
	
	
}

/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
			print("ERROR!");
			
//        uint32_t err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
//        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            m_lux_meas_ind_conf_pending = true;
            /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events (assuming that the button events are only needed in connected
                         state). If this is uncommented out here,
                            1. Make sure that app_button_disable() is called when handling
                               BLE_GAP_EVT_DISCONNECTED below.
                            2. Make sure the app_button module is initialized.
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            */
            break;

        case BLE_GAP_EVT_DISCONNECTED:
           nrf_gpio_pin_set(CONNECTED_LED_PIN_NO); 
				   nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
				    
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            m_lux_meas_ind_conf_pending = false;
            
				    /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events. This should be done to save power when not connected
                         to a peer.
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            */
            print("Disconnected.");
                advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
              
							

                // Configure buttons with sense level low as wakeup source.
                nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                                         BUTTON_PULL,
                                         NRF_GPIO_PIN_SENSE_LOW);
                
                // Go to system-off mode (this function will not return; wakeup will cause a reset)                
                //err_code = sd_power_system_off();
                //APP_ERROR_CHECK(err_code);
							  m_lux_meas_ind_conf_pending = false;
							  nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);  
							  nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
						  	advertising_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
	  ble_lbe_on_ble_evt(&m_lbe, p_ble_evt);
	  ble_luxsync_on_ble_evt(&m_luxsync, p_ble_evt);
	  ble_time_on_ble_evt(&m_current_time,p_ble_evt);
	
    /*
    YOUR_JOB: Add service ble_evt handlers calls here, like, for example:
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    */
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_RC_250_PPM_4000MS_CALIBRATION, false);
    //SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for handling a button event.
 *
 * @param[in]   pin_no         Pin that had an event happen.
 * @param[in]   button_event   APP_BUTTON_PUSH or APP_BUTTON_RELEASE.
 */
/* YOUR_JOB: Uncomment this function if you need to handle button events.
static void button_event_handler(uint8_t pin_no, uint8_t button_event)
{
    if (button_action == APP_BUTTON_PUSH)
    {
        switch (pin_no)
        {
            case MY_BUTTON_PIN:
                // Code to handle MY_BUTTON keypresses
                break;

            // Handle any other buttons

            default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
    }
}
*/

/**@brief Function for initializing the GPIOTE handler module.
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    // Note: Array must be static because a pointer to it will be saved in the Button handler
    //       module.
    static app_button_cfg_t buttons[] =
    {
        {WAKEUP_BUTTON_PIN, APP_BUTTON_ACTIVE_LOW, BUTTON_PULL, NULL},
        // YOUR_JOB: Add other buttons to be used:
        // {MY_BUTTON_PIN,     false, BUTTON_PULL, button_event_handler}
    };

    APP_BUTTON_INIT(buttons, sizeof(buttons) / sizeof(buttons[0]), BUTTON_DETECTION_DELAY, true);

    // Note: If the only use of buttons is to wake up, the app_button module can be omitted, and
    //       the wakeup button can be configured by
    // GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
	  
    APP_ERROR_CHECK(err_code);
}

void date_time_struc_init()
{
m_current_date_time_value.year=2014;
m_current_date_time_value.month=6;
m_current_date_time_value.day=3;

}

/**@brief Function for application main entry.
 */
int main(void)
{ 
	  

    // Initialize 
    leds_init();
    timers_init();
    gpiote_init();
    buttons_init();
	  pw_bus_init();
	  i2c_eeprom_init();
    ble_stack_init();
    scheduler_init();    
    gap_params_init();
    services_init();
	  date_time_struc_init();  
	   	  
    advertising_init();
	  conn_params_init();
    sec_params_init();
	  //eeprom_updateadd_pointer((uint32_t)0x00);  // erase the memory pointer only once 
	  
	 /*
	  uint32_t temp;
	  temp = mem_pointer;
	  
	 
		 
	  uint8_t i;
	  for (i=0;i<=240;i++)
   	{uint8_t temp[3];
			uint8_t j;
			update_current_time(&m_current_time,&m_current_date_time_value,date_timebuffer);
			for (j=0;j<3;j++)			{
			temp[j]=date_timebuffer[j];
			}
			}
	 	*/
		/*
	  //i2c_eeprom_erase();
		uint8_t err_cy =0x00;
	  uint8_t he_data_buff[50]={0x31,0x1B,0xC8,0x00,0x00,0x00,0x00,0x31,0x1B,0xD0,0x44,0x32,0x7F,0xF0,0x31,0x1C,0x18,0x00,0x55,0xA2,0x07,0x31,0x1C,0x20,0x60,0xC0,0xE3,0x00,0x31,0x1C,0x28,0x29,0x10,0x0A,0xFF};
    uint32_t addss=0;
		for(addss=0;addss<12075;addss=addss+35)
			{
			if(!i2c_eeprom_write(addss,(uint8_t*)&he_data_buff[0],35))
				{
				err_cy=0xFF;
				}
			}
			eeprom_updateadd_pointer((uint32_t)12075);
		*/
		//uint32_t addss=0xFFF5;	
		//uint8_t read_by[40];
		//i2c_eeprom_read(0x00,(uint8_t*)&read_by[0],(uint32_t)0x27);
		//
	  //read_by[0] = 03;
		//read_by[1] = 25;
		//read_by[0] = i2c_eeprom_read_byte( 0xAE,(uint16_t)(addss & 0x0FFF5));
		//i2c_eeprom_read(addss,data_buff, 42);
	  //read_by[1]=2;
		
//Debug	
/*
#if 1		
#define TX_PIN_NUMBER (2)
#define RX_PIN_NUMBER	(3)

	 simple_uart_config(0, TX_PIN_NUMBER,0, RX_PIN_NUMBER, 0);

#endif
*/


	
	
   print("Starting....\r\n");
				
				
    //Start execution
    timers_start();
		advertising_start();
		nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
					  

    // Enter main loop
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}

/**
 * @}
 */
