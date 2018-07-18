/**
  ******************************************************************************
  * File Name          : drivers_cmpower.h
  * Description        : 底盘功率限制
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 底层函数
  ******************************************************************************
  */
#ifndef DRIVERS_CMPOWER_H
#define DRIVERS_CMPOWER_H

#define    CM_current_bottom      3000.f
#define 	 CMFLIntensity_bottom   1000.f
#define    CMFRIntensity_bottom   1000.f
#define  	 CMBLIntensity_bottom   1000.f
#define	   CMBRIntensity_bottom   1000.f

#define    CM_current_lower      4096.f
#define 	 CMFLIntensity_lower   1024.f
#define    CMFRIntensity_lower   1024.f
#define  	 CMBLIntensity_lower   1024.f
#define	   CMBRIntensity_lower   1024.f

#define    CM_current_MAX_LOW      30000.0f
#define    CM_current_MAX      24000.0f       //CM_current_MAX对应的电流值*24V*100ms<=60J
#define    CMFLIntensity_MAX   6000.0f
#define    CMFRIntensity_MAX   6000.0f
#define    CMBLIntensity_MAX   6000.0f
#define    CMBRIntensity_MAX   6000.0f

#define			CM_current_full			4096.0f

void RestrictPower(int16_t *intensity1, int16_t *intensity2, int16_t *intensity3, int16_t *intensity4);
void dynamicUpperBound(void);
#endif
