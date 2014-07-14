
/*
*Lib defining the MAX 44009 for the Exeger Light band project . 
*20/05/2014
*Copyright Exeger Systems AB  2014
*
*Author:Kartik karuna (Kartik.karuna@exeger.com) 
*
*/

#include "max_44009.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "cy_io.h"
#include "math.h"

//Device address  
#define MAX_LUX_ADDR_1	0x96            //!< 6 MSBs of the MAX 44009 I2C ADD
#define MAX_LUX_ADDR_2	0x94            //!< 6 MSBs of the MAX 44009 I2C ADD

#define BUS             true            //Bool valve to select bus 

static uint8_t m_device_address; 		//!< Device address in bits [7:1]

#define INT_STATUS		0x00
#define INT_ENABLE		0x01
const uint8_t CONFIG_REG = 0x02;
const uint8_t HIGH_BYTE	 =  0x03;
const uint8_t LOW_BYTE	 =  0x04;
const uint8_t THRESH_HIGH	= 0x05;
const uint8_t THRESH_LOW	=	0x06;
const uint8_t THRESH_TIMER =	0x07;

extern float SCALE_FACTOR;	// captures scaling factors to map from % brightness to PWM

#define BUS_UP          i2c_vcc1 
#define BUS_DOWN        i2c_vcc2


bool config_max44009(uint8_t slaveAddr)
{
  bool transfer_succeeded = true;
	uint8_t data_buffer[2];
	data_buffer[0] = CONFIG_REG;
  data_buffer[1] = 0x02;
  
	transfer_succeeded= twi_master_transfer(m_device_address, data_buffer, 2, TWI_ISSUE_STOP);

	return transfer_succeeded;
}


float getLightLevel(uint8_t slaveAddr) 
{
	uint8_t  lowByte;
	uint8_t  highByte;
	uint8_t exponent;
	uint8_t mantissa;
	float result;
	//Modify accordign to nrf81522
	
	if (twi_master_transfer(slaveAddr,(uint8_t*)&HIGH_BYTE, 1, TWI_DONT_ISSUE_STOP))
	{
	  if (twi_master_transfer(slaveAddr| TWI_READ_BIT,&highByte, 1, TWI_ISSUE_STOP))
		{
		  //Read the Hight Bit
		}
	
	}
	if (twi_master_transfer(slaveAddr,(uint8_t*)&LOW_BYTE, 1, TWI_DONT_ISSUE_STOP))
	{
	  if (twi_master_transfer(slaveAddr| TWI_READ_BIT,&lowByte, 1, TWI_ISSUE_STOP))
		{
		 //Read the LOW bit
		}
	
	}
	
	
	exponent = (highByte & 0xF0) >> 4;// upper four bits of high byte register
	mantissa = (highByte & 0x0F) << 4;// lower four bits of high byte register = 
 	mantissa =mantissa|(lowByte & 0x0F); // upper four bits of mantissa
	result = mantissa * (1 << exponent) * 0.045;
	
	return result;
} //getLightLevel

void read_sensors(light_reading_t * p_lux_value)
{
	 
	 if (!twi_master_init())
    {
		//Wait for Initilization 
		
		}
	  nrf_gpio_pin_clear(BUS_DOWN);    //Turn off the sensors on the lower line --
		nrf_gpio_pin_set(BUS_UP);        //Turn on the sensors on the UP line --
	  
	  config_max44009(MAX_LUX_ADDR_1);
		config_max44009(MAX_LUX_ADDR_2);
	  nrf_delay_ms(400);
    p_lux_value->u4_top = getLightLevel(MAX_LUX_ADDR_1);
		p_lux_value->u5_up = getLightLevel(MAX_LUX_ADDR_2);
	 
		//switch power line 
		nrf_gpio_pin_clear(BUS_UP);         //Turn off the sensors on the UP line --
		nrf_gpio_pin_set(BUS_DOWN);        //Turn on the sensors on the Down line    
	  //Read sensors on the Down line 
		config_max44009(MAX_LUX_ADDR_1);
		config_max44009(MAX_LUX_ADDR_2);
		nrf_delay_ms(400);
		p_lux_value->u6_down_1= getLightLevel(MAX_LUX_ADDR_1);

		p_lux_value->u7_down_2 = getLightLevel(MAX_LUX_ADDR_2);
	  
		
   
   //nrf_gpio_pin_clear(ASSERT_LED_PIN_NO);

}
/**@brief Function for encoding a Light Measurement.
 *
 * @param[in]   p_lux_value        Light reading float.
 * @param[in]   p_encoded_light_reading         Measurement to be encoded.
 * 
 *
 * @return      n/a.
 */


void encode_sensor_value(light_reading_t * p_lux_value,encoded_light_reading_t * p_encoded_light_reading)
{
		p_encoded_light_reading->u4_top    =(uint8_t)round(50*(log10(p_lux_value->u4_top)));
		p_encoded_light_reading->u5_up     =(uint8_t)round(50*(log10(p_lux_value->u5_up)));
		p_encoded_light_reading->u6_down_1 =(uint8_t)round(50*(log10(p_lux_value->u6_down_1)));
		p_encoded_light_reading->u7_down_2 =(uint8_t)round(50*(log10(p_lux_value->u7_down_2)));
	
}
	
	

