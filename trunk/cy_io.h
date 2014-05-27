/*
*Lib defining the I/O ports used in the Exeger Light band project . 
*13/05/2014
*Copyright Exeger Systems AB  2014
*
*Author:Kartik karuna (Kartik.karuna@exeger.com) 
*
*/
#ifndef CY_IO_H__ 
#define CY_IO_H__

#include "nrf_gpio.h"

#define LED_START      29
#define LED_0          29
#define LED_1          28
#define LED_2					 27

#define LED_STOP       15

#define BUTTON_START   0
#define BUTTON_0       0
#define BUTTON_1       1

#define BUTTON_3       5
#define BUTTON_4       17

#define BUTTON_STOP    7
#define BUTTON_PULL    NRF_GPIO_PIN_NOPULL

//I2C power bus

#define i2c_vcc1       17
#define i2c_vcc2       5

#define RX_PIN_NUMBER  16    // UART RX pin number.
#define TX_PIN_NUMBER  17    // UART TX pin number.
#define CTS_PIN_NUMBER 18    // UART Clear To Send pin number. Not used if HWFC is set to false
#define RTS_PIN_NUMBER 19    // Not used if HWFC is set to false
#define HWFC           false // UART hardware flow control

#define SPIS_MISO_PIN  20    // SPI MISO signal. 
#define SPIS_CSN_PIN   21    // SPI CSN signal. 
#define SPIS_MOSI_PIN  22    // SPI MOSI signal. 
#define SPIS_SCK_PIN   23    // SPI SCK signal. 

#endif  // CY_IO_H__
