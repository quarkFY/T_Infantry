/**
  ******************************************************************************
  * File Name          : peripheral_pc.h
  * Description        : 光电开关
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 光电开关读取
  ******************************************************************************
  */
#include "peripheral_pc.h"
#include "utilities_debug.h"


int8_t pc_Mood;
PC_e PC;
int8_t GetPhotoElectricSwitchMood(PC_e PC)
{
	switch(PC)
	{
		case PC1:
			pc_Mood = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2);break;
		case PC2:
			pc_Mood = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3);break;
		default:
			fw_Error_Handler();
	}
	return pc_Mood;
}
