/**
  ******************************************************************************
  * File Name          : tasks_upper.h
  * Description        : 上位机(妙算)通信任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef TASKS_UPPER_H
#define TASKS_UPPER_H

#include "utilities_iopool.h"
#include "utilities_minmax.h"
//#include "application_gimbalcontrol.h"
IOPoolDeclare(upperIOPool, struct{float yawAdd; float pitchAdd;});

void getCtrlUartTask(void const * argument);

void zykProcessData(void);
void wave_task(void const * argument);

#endif
