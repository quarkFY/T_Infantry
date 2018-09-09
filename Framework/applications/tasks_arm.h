/**
  ******************************************************************************
  * File Name          : tasks_arm.h
  * Description        : 取弹机械臂电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 
	* 
  ******************************************************************************
  */

#ifndef TASKS_ARM_H
#define TASKS_ARM_H

#include "application_motorcontrol.h"
#include "rtos_semaphore.h"

#define AM1Reduction 19.0
#define AM23Reduction 19.0

#define PM1Reduction 36.0
#define PM2Reduction 36.0
#define PM3Reduction 36.0

void Can2ControlTask(void const * argument);
void ControlAM1L(void);
void ControlAM1R(void);
void ControlAM2L(void);



void ControlPM1(void);
void ControlPM2(void);
void ControlPM3(void);
		
void shootOneGolf(void);
void shootOneGolfConpensation(void);
void PMRotate(void);
void shootLoad(void);

#define PIR_R_Ready  (HAL_GPIO_ReadPin(PIR_R_GPIO_Port,GPIO_PIN_5))
					
#define PIR_L_Ready  (HAL_GPIO_ReadPin(PIR_L_GPIO_Port,GPIO_PIN_6))

#define PIR_C_Free  (!HAL_GPIO_ReadPin(PIR_C_GPIO_Port,GPIO_PIN_0))
		

#endif
