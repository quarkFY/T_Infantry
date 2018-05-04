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

#define AM1Reduction 189.0
#define AM23Reduction 189.0

void Can2ControlTask(void const * argument);
void ControlAM1L(void);
void ControlAM1R(void);
void ControlAM2L(void);
void ControlAM2R(void);
void ControlAM3R(void);

//void setAMAngle(MotorId id, float angle);
//void getGolf(void);
void armReset(void);
//void armStretch(void);
void ARM_INIT(void);

//void GripLoadProcess(void);

uint8_t Hero_Angle_Track(float final,float realAngle,float *angleTarget,uint8_t *time_milis);
uint8_t taskDelay(uint32_t time_milis);
#endif
