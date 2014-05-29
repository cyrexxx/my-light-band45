/*
*Lib defining the MAX 44009 for the Exeger Light band project . 
*20/05/2014
*Copyright Exeger Systems AB  2014
*
*Author:Kartik karuna (Kartik.karuna@exeger.com) 
*
*/


#ifndef M24m02_H__
#define M24m02_H__


#include <stdbool.h>
#include <stdint.h>

	




void i2c_eeprom_init();
/**
 *	Function:	i2c_eeprom_erase
 *
 *	Arguments:	n/a
 *
 *	Returns:	true/false
 *
 *	Description:	Erases the EEPROM.
**/
bool i2c_eeprom_erase();
/**
 *	Function:	i2c_eeprom_write_page
 *
 *	Arguments:	18 bit address 
 *              pointer to the data.
 *              length of data
 *
 *	Returns:	true/false
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/
bool i2c_eeprom_write(uint32_t address, uint8_t* data, uint32_t length);
bool i2c_eeprom_write_buffer(uint8_t dev_id, uint16_t address, uint8_t* data, uint16_t length);
/**
 *	Function:	i2c_eeprom_write_page
 *
 *	Arguments:	Device Address
 *              16 bit eeprom address 
 *              pointer to the data.
 *              length of data
 *
 *	Returns:	true/false
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/
bool i2c_eeprom_write_page(uint8_t dev_id, uint16_t eeaddress, uint8_t* data, uint8_t length );
/**
 *	Function:	i2c_eeprom_read_byte
 *
 *	Arguments:	Device Address
 *              16 bit eeprom address
 *
 *	Returns:	true/false
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/

uint8_t i2c_eeprom_read_byte(uint8_t dev_id, uint16_t eeaddress);
/**
 *	Function:	i2c_eeprom_read_buffer
 *
 *	Arguments:	18 bit eeprom address 
 *              pointer to the data.
 *              length of data
 *
 *	Returns:	true/false
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/
bool i2c_eeprom_read(uint32_t address, uint8_t* data, uint32_t length);
/**
 *	Function:	i2c_eeprom_read_buffer
 *
 *	Arguments:	Device Address
 *              16 bit eeprom address 
 *              pointer to the data.
 *              length of data
 *
 *	Returns:	true/false
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/
bool i2c_eeprom_read_buffer(uint8_t dev_id, uint16_t address, uint8_t *buffer, uint16_t length); 





#endif // M24m02_H__
