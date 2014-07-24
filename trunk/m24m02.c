
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

#include "defines.h"

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
#define BLOCK_MAX         0x3FFF0 //((4*BLOCK_SIZE)-15)   //0x40000 //saving last 15 bytes to save memory location.


#define BUS             true            //Bool valve to select bus 

//static uint8_t m_device_address; 		    //!< Device address in bits [7:1]

#define INT_STATUS		0x00
#define INT_ENABLE		0x01
#define PAGE_SIZE     0x80

extern float SCALE_FACTOR;	       // captures scaling factors to map from % brightness to PWM

#define WC        MEM_WC 
#define E2        MEM_EN

#define pointer_address 0xFFF5  //on AE device

void i2c_eeprom_init() 
{
  if (!twi_master_init())
    {
		//Wait for Initilization 
		
		}
  nrf_gpio_pin_set(E2);
	nrf_delay_ms(1);
}

bool i2c_eeprom_erase() 
{
  // initialize all bytes to 0
  uint8_t data[128] = {0};
  uint32_t addr = 0x0;
   //  Erasing EEPROM
  while (addr < BLOCK_MAX) 
  {
    i2c_eeprom_write(addr, data, sizeof(data));
    addr += sizeof(data); 
  }
return true;
//Erase done
}

bool i2c_eeprom_write(uint32_t address, uint8_t* data, uint32_t length) 
{
  if (address > BLOCK_MAX)  {return false;}
  if (address + length > BLOCK_MAX) {return false;}

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
	uint16_t lenght_send = end_byte - start_byte;
  return i2c_eeprom_write_buffer(dev_id, (uint16_t)(start_byte % BLOCK_SIZE),
             (uint8_t*)&(data[start_byte - address]),lenght_send);
}

bool i2c_eeprom_write_buffer(uint8_t dev_id, uint16_t address, uint8_t* data, uint16_t length) {
  uint16_t start_byte = address;
  uint16_t end_byte   = address + length;

  uint16_t curr_page_start = (address / PAGE_SIZE) * PAGE_SIZE;
  uint32_t next_page_start = curr_page_start + PAGE_SIZE;
  print("i2c %d %d %d %d \r\n",start_byte,end_byte,curr_page_start,next_page_start );
  bool success = true;
  while (end_byte > next_page_start && success) 
  {
    success = i2c_eeprom_write_page(dev_id, start_byte, &(data[start_byte - address]), next_page_start - start_byte);
    print("i2c page overflow \r\n");
		curr_page_start = next_page_start;
    next_page_start = curr_page_start + PAGE_SIZE;
    start_byte = curr_page_start;
  }
  if (!success) {return false; }
  uint16_t lenght_send = end_byte - start_byte;
  return i2c_eeprom_write_page(dev_id, start_byte, &(data[start_byte - address]), lenght_send);
}

bool i2c_eeprom_write_page(uint8_t dev_id, uint16_t eeaddress, uint8_t* data, uint8_t length ) 
{
  nrf_gpio_pin_clear(WC);
	uint8_t buffer_len=length+2;
	uint8_t data_buffer[130];
	
	uint8_t add_high=(uint8_t)((eeaddress >> 8) &0xFF);
	data_buffer[0]=add_high;
	
	uint8_t add_low=(uint8_t)(eeaddress & 0xFF);
	data_buffer[1]=add_low;
	uint16_t i;
	for (i=0;i<length;i++)
	{
	 data_buffer[i+2]= *(data+i);
	}
	if (twi_master_transfer(dev_id,data_buffer,buffer_len,TWI_ISSUE_STOP))
	{
	  nrf_gpio_pin_set(WC);
		nrf_delay_ms(10);
		return true;
	
	}
  else 
	{
    nrf_gpio_pin_set(WC);
		return false;
  }	
	  
}
/*
uint32_t i2c_eeprom_current_address()
{
uint8_t device_address,i;     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[dev_offset];
uint16_t address_pointer[4];
uint32_t return_address;

for (i=0;1<4;i++)
  {
  device_address = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[i];
  address_pointer[i]=i2c_eeprom_address_read(device_address)
  }

return return_address;  
}

uint16_t i2c_eeprom_address_read(uint8_t dev_id)
{
if(twi_master_transfer(dev_id|TWI_READ_BIT,&read_byte,1,TWI_ISSUE_STOP))
   {
   }

}
*/

uint8_t i2c_eeprom_read_byte(uint8_t dev_id, uint16_t eeaddress ) 
{
  nrf_gpio_pin_set(WC); 
	uint8_t read_byte = 0xDB;
	uint8_t add_buffer[2];
	add_buffer[0]=(uint8_t)((eeaddress >> 8) &0xFF);
	add_buffer[1]=(uint8_t)(eeaddress & 0xFF);
	if(twi_master_transfer(dev_id,add_buffer,2,TWI_DONT_ISSUE_STOP))
	{
	 if(twi_master_transfer(dev_id|TWI_READ_BIT,&read_byte,1,TWI_ISSUE_STOP))
	 {
	  //Read successful
	 }
	}
return read_byte;
}

bool i2c_eeprom_read(uint32_t address, uint8_t* data, uint32_t length) {
  if (address > BLOCK_MAX) {return false;}
  if (address + length > BLOCK_MAX) {return false;}

  uint32_t start_byte        = address;
  uint32_t end_byte          = address + length;
  uint32_t curr_device_start = (address / BLOCK_SIZE) * BLOCK_SIZE;
  uint32_t next_device_start = curr_device_start + BLOCK_SIZE;

  bool success = true;
  while (end_byte > next_device_start && success) 
		{
    uint8_t dev_offset = start_byte / BLOCK_MAX;
    uint8_t dev_id     = (MEM_BASE_ADD | DEV_REVERSE_LOOKUP[dev_offset]);
    success = i2c_eeprom_read_buffer(dev_id, (uint16_t)(start_byte % BLOCK_SIZE), (uint8_t*)&(data[start_byte - address]), next_device_start - start_byte);

    curr_device_start = next_device_start;
    next_device_start = curr_device_start + BLOCK_SIZE;
    start_byte        = curr_device_start;
    }
  if (!success) 
	{
    return false;
  }
  uint8_t dev_offset = start_byte / BLOCK_SIZE;
  uint8_t dev_id     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[dev_offset];
  return i2c_eeprom_read_buffer(dev_id, (uint16_t)(start_byte % BLOCK_SIZE), (uint8_t*)&(data[start_byte - address]), (uint16_t)(end_byte - start_byte));
}

bool i2c_eeprom_read_buffer(uint8_t dev_id, uint16_t address, uint8_t *buffer, uint16_t length ) 
{
    bool success;
    nrf_gpio_pin_set(WC);
	  nrf_delay_ms(5);
	  uint8_t add_buffer[2];
	  add_buffer[0]=(uint8_t)((address >> 8) &0xFF);
	  add_buffer[1]=(uint8_t)(address & 0xFF);
	  if(twi_master_transfer(dev_id,add_buffer,2,TWI_DONT_ISSUE_STOP))
	  { 
			if(twi_master_transfer(dev_id|TWI_READ_BIT,buffer,length,TWI_ISSUE_STOP))
			{
				success= true;
			}
		}
  else
		{
     success = false;
    } 
	return success;	
}

uint32_t eeprom_find_add_pointer(void)
{
  uint8_t dev_id     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[3];
  uint8_t pointer_address_buffer[4];
  uint8_t length = 4;
	

  if(i2c_eeprom_read_buffer(dev_id,(uint16_t)pointer_address,(uint8_t*)&pointer_address_buffer[0], (uint16_t)length))
	{
    uint32_t t_buffer = ((((uint32_t)pointer_address_buffer[0])<<24)|(((uint32_t)pointer_address_buffer[1])<<16)|(((uint32_t)pointer_address_buffer[2])<<8)|(((uint32_t)pointer_address_buffer[3])));       ;
		 // t_buffer = (uint32_t)pointer_address_buffer;
	 return t_buffer;
	}

 return ((uint32_t)0);
}

bool eeprom_updateadd_pointer(uint32_t address)
{
  uint8_t dev_id     = MEM_BASE_ADD | DEV_REVERSE_LOOKUP[3];
  uint8_t pointer_address_buffer[4];
  uint8_t lenght_send = sizeof(uint32_t);
	
	//conver 32 bit address in to 8 bytes chunks.
	
	pointer_address_buffer[0]=(uint8_t)((address >> 24) &0x000000FF);
	pointer_address_buffer[1]=(uint8_t)((address >> 16) &0x000000FF);
	pointer_address_buffer[2]=(uint8_t)((address >> 8) &0x000000FF);
	pointer_address_buffer[3]=(uint8_t)(address & 0x000000FF);
	
	return i2c_eeprom_write_page(dev_id,(uint16_t)pointer_address,pointer_address_buffer,lenght_send);
}


