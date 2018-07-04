/**
  ******************************************************************************
  * File Name          :drivers_i2c.c
  * Description        : gpioÄ£Äâi2cÍ¨ÐÅ
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
	
#include <cmsis_os.h>
#include <gpio.h>
#include "drivers_led_user.h"
#include "drivers_i2c.h"
#include "peripheral_gpio.h"

#define SDALow() HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_RESET)
#define SDAHigh() HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin, GPIO_PIN_SET)
#define SCLLow() HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET)
#define SCLHigh() HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET)

uint16_t reciveBuffer[8];

void set_SDA(uint8_t x)
{
	if(x) SDAHigh();
	else SDALow();
}

void i2c_start()
{
	SDAHigh();
	for(uint8_t i=0;i<4;i++)
	{
		SCLHigh();
		osDelay(100);
		SCLLow();
		osDelay(100);
	}
}
	
void i2c_write_byte(unsigned char b)  
{
	i2c_start();
	//set_gpio_direction(SDA, OUTP);          //??SDA?????  
	for (uint8_t i=7; i>=0; i--) {  
		set_SDA(b & (1<<i));
		SCLHigh();
		osDelay(100);
		SCLLow();
		osDelay(100);
//	SCLLow();             // SCL??  
//	osDelay(100);  
//	set_SDA(b & (1<<i));        //????????????????  
//	SCLHigh();             // SCL??  
//	osDelay(100);  
	}  
	//i2c_read_ack();                 //???????ACK??  
} 

void i2c_read_byte()
{
	for(uint8_t i=0;i<7;i++)
	{
		reciveBuffer[i+1] = reciveBuffer[i];
	}
	//reciveBuffer[0] = SDARead();
}