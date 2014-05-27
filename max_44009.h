/*
*Lib defining the MAX 44009 for the Exeger Light band project . 
*20/05/2014
*Copyright Exeger Systems AB  2014
*
*Author:Kartik karuna (Kartik.karuna@exeger.com) 
*
*/


#ifndef MAX_44009_H__
#define MAX_44009_H__

#include <stdbool.h>
#include <stdint.h>

//uint8_t lightReadingCounter;	

typedef struct light_reading_s
{
    float u4_top;
    float u5_up;
    float u6_down_1;
    float u7_down_2;

} light_reading_t;

/**
 *	Function:	config_max44009
 *
 *	Arguments:	Slave address
 *
 *	Returns:	true/false
 *
 *	Description:	configures MAX 44009
**/

bool config_max44009(uint8_t slaveAddr);
	
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


#endif // MAX_44009_H__
