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

void Can2ControlTask(void const * argument);
void ControlAM1L(void);
void ControlAM1R(void);
void ControlAM2L(void);
void ControlAM2R(void);
void ControlAM3L(void);

#endif
