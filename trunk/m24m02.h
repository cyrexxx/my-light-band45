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

//uint8_t lightReadingCounter;	

typedef struct light_reading_s
{
    uint8_t 

} light_reading_t;

/**
 *	Function:	write_buffer
 *
 *	Arguments:	Address of the data to be written
 *              Data 
 *              length of data
 *
 *	Returns:	true/false
 *
 *	Description:	configures MAX 44009
**/
void write_buffer(uint32_t addr, uint8_t* data, uint32_t length);

/**
 *	Function:	getLightLevel
 *
 *	Arguments:	n/a
 *
 *	Returns:	the ambient light level, in lux
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/
float getLightLevel(uint8_t slaveAddr);

/**
 *	Function:	read_sensors
 *
 *	Arguments:	n/a
 *
 *	Returns:	the ambient light level, in lux for all the sensors
 *
 *	Description:	Reads both the light registers on the device and returns the 
 *			computed light level
**/

void read_sensors(light_reading_t * p_lux_value);


#endif // M24m02_H__
