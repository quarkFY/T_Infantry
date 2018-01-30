/**
  ******************************************************************************
  * File Name          : application_motorcontrol.h
  * Description        : 电机控制驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 设定电机电流
	* 陀螺仪复位
  ******************************************************************************
  */
#ifndef APPLICATION_MOTORCONTROL_H
#define APPLICATION_MOTORCONTROL_H

#include "stdint.h"

typedef enum {CMFL, CMFR, CMBL, CMBR, GMYAW, GMPITCH, PM1, PM2, AM1L, AM1R, AM2L, AM2R, AM3R,SM} MotorId;

void setMotor(MotorId motorId, int16_t Intensity);
void GYRO_RST(void);
#endif
