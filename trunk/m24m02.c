
/*
*Lib defining the M24m)@ eeeprom form ST for the Exeger Light band project . 
*26/05/2014
*Copyright Exeger Systems AB  2014
*
*Author:Kartik karuna (Kartik.karuna@exeger.com) 
*
*/

#include "m24m02.h"
#include "twi_master.h"
#include "nrf_delay.h"
#include "cy_io.h"

//Device address  
#define MEM_BASE_ADD  0xA0            //!< 6 MSBs of the MAX 44009 I2C ADD
#define PAGE_0        0x08            // PAGE 0 (E2 =1 , A17=A16=0,W/R=D.C)
#define PAGE_1        0x0A            // PAGE 0 (E2 =1 , A17=0 A16=1,W/R=D.C)
#define PAGE_2        0x0C            // PAGE 0 (E2 =1 , A17=1 A16=0,W/R=D.C)
#define PAGE_3        0x0E            // PAGE 0 (E2 =1 , A17=A16=1,W/R=D.C)

//Memory Blocks 
#define BLOCK_SIZE        0x10000
#define BLOCK_1_MAX       0x20000
#define BLOCK_2_MAX       0x30000
#define BLOCK_3_MAX       0x40000


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


void write_buffer(uint32_t addr, uint8_t* data, uint32_t length)
{
  //bool transfer_succeeded = true;
  uint8_t dev_id = MEM_BASE_ADD;

   if (addr < BLOCK_SIZE && (addr + length) < BLOCK_SIZE)                        // if address falls withing BLock 1 and length of data to be written is less than Block SIZE
     {
     dev_id = dev_id | PAGE_0 ;
     write_device_buffer(dev_id, (uint16_t)addr, data, (uint16_t)length);
     }
    else if(addr < BLOCK_SIZE && (addr + length) > BLOCK_SIZE)                  // if address falls withing BLock 1 but length of data to be written spans 2 Blocks
     { 
      // break it into two writes	
      uint8_t first_length = BLOCK_SIZE - addr;
      dev_id = dev_id | PAGE_0 ;
      write_device_buffer(dev_id, (uint16_t)addr, data, first_length); 

      dev_id = dev_id | PAGE_1 ;
      write_device_buffer(dev_id, 0x0, (uint8_t*)&data[first_length], length - (first_length));
     }   
    else if (addr > BLOCK_SIZE && (addr + length) < BLOCK_1_MAX)
     {
      dev_id = dev_id | PAGE_1 ;
      write_device_buffer(dev_id, (uint16_t)addr, data, (uint16_t)length);
     }
     else if(addr > BLOCK_SIZE && (addr + length) > BLOCK_1_MAX)                  // if address falls withing BLock 1 but length of data to be written spans 2 Blocks
     { 
      // break it into two writes	
      uint8_t first_length = BLOCK_1_MAX - addr;
      dev_id = dev_id | PAGE_1 ;
      write_device_buffer(dev_id, (uint16_t)addr, data, first_length); 

      dev_id = dev_id | PAGE_2 ;
      write_device_buffer(dev_id, 0x0, (uint8_t*)&data[first_length], length - (first_length));
     }   	
    else if (addr > BLOCK_1_MAX && (addr + length) < BLOCK_2_MAX)
     {
      dev_id = dev_id | PAGE_2 ;
      write_device_buffer(dev_id, (uint16_t)addr, data, (uint16_t)length);
     }
     else if(addr > BLOCK_1_MAX && (addr + length) > BLOCK_2_MAX)                  // if address falls withing BLock 1 but length of data to be written spans 2 Blocks
     { 
      // break it into two writes	
      uint8_t first_length = BLOCK_2_MAX - addr;
      dev_id = dev_id | PAGE_2 ;
      write_device_buffer(dev_id, (uint16_t)addr, data, first_length); 

      dev_id = dev_id | PAGE_3 ;
      write_device_buffer(dev_id, 0x0, (uint8_t*)&data[first_length], length - (first_length));
     }  
     else if (addr > BLOCK_2_MAX && (addr + length) < BLOCK_3_MAX)
     {
      dev_id = dev_id | PAGE_3 ;
      write_device_buffer(dev_id, (uint16_t)addr, data, (uint16_t)length);
     }
     else if(addr > BLOCK_2_MAX && (addr + length) > BLOCK_3_MAX)                  // if address falls withing BLock 1 but length of data to be written spans 2 Blocks
     { 
      // break it into two writes	
      uint8_t first_length = BLOCK_3_MAX - addr;
      dev_id = dev_id | PAGE_3 ;
      write_device_buffer(dev_id, (uint16_t)addr, data, first_length); 

      dev_id = dev_id | PAGE_0 ;
      write_device_buffer(dev_id, 0x0, (uint8_t*)&data[first_length], length - (first_length));
     }  

  //return transfer_succeeded;
}

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
	 float lux;
	 if (!twi_master_init())
    {
		//Wait for Initilization 
		
		}
	  nrf_gpio_pin_clear(BUS_DOWN);    //Turn off the sensors on the lower line --
		nrf_gpio_pin_set(BUS_UP);        //Turn on the sensors on the UP line --
	  
	  config_max44009(MAX_LUX_ADDR_1);
	  nrf_delay_ms(400);
    lux = getLightLevel(MAX_LUX_ADDR_1);
	  p_lux_value->u4_top= lux;
		config_max44009(MAX_LUX_ADDR_2);
		lux = getLightLevel(MAX_LUX_ADDR_2);
	  p_lux_value->u5_up= lux;
		//switch power line 
		nrf_gpio_pin_clear(BUS_UP);         //Turn off the sensors on the UP line --
		nrf_gpio_pin_set(BUS_DOWN);        //Turn on the sensors on the Down line    
	  //Read sensors on the Down line 
		nrf_delay_ms(400);
		lux = getLightLevel(MAX_LUX_ADDR_1);
	  p_lux_value->u6_down_1= lux;
		config_max44009(MAX_LUX_ADDR_2);
		lux = getLightLevel(MAX_LUX_ADDR_2);
	  p_lux_value->u7_down_2= lux;
		
   // getLightLevel(MAX_LUX_ADDR_2);
   //light_reading_t.u4_top temp1= 
//nrf_gpio_pin_clear(ASSERT_LED_PIN_NO);

}
