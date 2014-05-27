
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

uint8_t DEV_REVERSE_LOOKUP[] = { 0x08,0x0A,0x0C,0x0E };

/*
#define PAGE_0        0x08            // PAGE 0 (E2 =1 , A17=A16=0,W/R=D.C)
#define PAGE_1        0x0A            // PAGE 1 (E2 =1 , A17=0 A16=1,W/R=D.C)
#define PAGE_2        0x0C            // PAGE 2 (E2 =1 , A17=1 A16=0, W/R=D.C)
#define PAGE_3        0x0E            // PAGE 3 (E2 =1 , A17=A16=1, W/R=D.C)
terms block and devices is used interchangeably
*/

//Memory Blocks 
#define BLOCK_SIZE        0x10000
#define BLOCK_MAX         (4*BLOCK_SIZE)   //0x40000


#define BUS             true            //Bool valve to select bus 

static uint8_t m_device_address; 		    //!< Device address in bits [7:1]

#define INT_STATUS		0x00
#define INT_ENABLE		0x01
#define PAGE_SIZE     0x100

extern float SCALE_FACTOR;	       // captures scaling factors to map from % brightness to PWM

#define WC        MEM_WC 
#define E2        MEM_EN



void i2c_eeprom_init() 
{
  nrf_gpio_pin_set(E2);
}

bool i2c_eeprom_erase() {
  // initialize all bytes to 0
  uint8_t data[256] = { 0 };
  uint32_t addr = 0x0;
   //  Erasing EEPROM
  while (addr < MAX_ADDR) 
  {
    i2c_eeprom_write_buffer(addr, data, sizeof(data));
    addr += sizeof(data); 
  }
//Erase done
}

bool i2c_eeprom_write_buffer(uint32_t address, uint8_t* data, uint32_t length) 
{
  if (address > MAX_ADDR)  {return false;}
  if (address + length > MAX_ADDR) {return false;}

  uint32_t start_byte        = address;
  uint32_t end_byte          = address + length;
  uint32_t curr_device_start = (address / BLOCK_SIZE) * BLOCK_SIZE;
  uint32_t next_device_start = curr_device_start + BLOCK_SIZE;

  bool success = true;
  while (end_byte > next_device_start && success) 
  {
    uint8_t dev_offset = start_byte / BLOCK_SIZE;
    uint8_t dev_id     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[dev_offset];
    success = i2c_eeprom_write_buffer(dev_id, (uint16_t)(start_byte % BLOCK_SIZE),(uint8_t*)&(data[start_byte - address]), next_device_start - start_byte);

    curr_device_start = next_device_start;
    next_device_start = curr_device_start + BLOCK_SIZE;
    start_byte        = curr_device_start;
  }
  if (!success) {return false;}

  uint8_t dev_offset = start_byte / BLOCK_SIZE;
  uint8_t dev_id     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[dev_offset];
  return i2c_eeprom_write_buffer(dev_id, (uint16_t)(start_byte % BLOCK_SIZE), (uint8_t*)&(data[start_byte - address]), end_byte - start_byte);
}

bool i2c_eeprom_write_buffer(uint8_t dev_id, uint16_t address, uint8_t* data, uint16_t length) {
  uint16_t start_byte = address;
  uint16_t end_byte   = address + length;

  uint16_t curr_page_start = (address / PAGE_SIZE) * PAGE_SIZE;
  uint16_t next_page_start = curr_page_start + PAGE_SIZE;

  bool success = true;
  while (end_byte > next_page_start && success) 
  {
    success = i2c_eeprom_write_page(dev_id, start_byte, &(data[start_byte - address]), next_page_start - start_byte);
    curr_page_start = next_page_start;
    next_page_start = curr_page_start + PAGE_SIZE;
    start_byte = curr_page_start;
  }
  if (!success) {return false; }

  return i2c_eeprom_write_page(dev_id, start_byte, &(data[start_byte - address]), end_byte - start_byte);
}

bool i2c_eeprom_write_page(uint8_t dev_id, uint16_t eeaddress, uint8_t* data, uint8_t length ) {
  
  **if (my_twi->start(dev_id, I2C_WRITE)) 
    {
      my_twi->write((uint8_t)((eeaddress >> 8) &0xFF));
      my_twi->write((uint8_t)(eeaddress & 0xFF));
      for (uint8_t c = 0; c < length; c++) {
        my_twi->write(data[c]);
    }
    my_twi->stop();
    // 24AA1025 will not acknowledge start conditions until the write cycle is complete
    while(!my_twi->start(dev_id, I2C_WRITE)) {
    };
    my_twi->stop();

    return true;
  } 
  else {
//    Serial.println("nack for dev_id / write");
    return false;
  }
}

uint8_t i2c_eeprom_read_byte(uint8_t dev_id, uint16_t eeaddress ) {
  uint8_t read_byte = 0;
  ** if (my_twi->start(dev_id, I2C_WRITE)) {
    my_twi->write((uint8_t)((eeaddress >> 8) &0xFF));
    my_twi->write((uint8_t)(eeaddress & 0xFF));
    my_twi->start(dev_id, I2C_READ);
    read_byte = my_twi->read(true);
    my_twi->stop();
  } 
  else {
//    Serial.println("nack for dev_id / write");
  } 
  return b;
}







/*
******************************************
*/

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
