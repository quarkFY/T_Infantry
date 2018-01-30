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
#ifndef _PC_H_
#define _PC_H_
#include "gpio.h"

typedef enum 
{
  PC1,
  PC2,
}PC_e;

int8_t GetPhotoElectricSwitchMood(PC_e);

#endif

