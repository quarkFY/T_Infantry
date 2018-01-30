/**
  ******************************************************************************
  * File Name          : peripheral_pc.h
  * Description        : ��翪��
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * ��翪�ض�ȡ
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

