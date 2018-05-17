/**
  ******************************************************************************
  * File Name          : application_motorcontrol.c
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
#include "application_motorcontrol.h"
#include "drivers_uartrc_user.h"
#include "can.h"
#include "peripheral_define.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"
#include "utilities_debug.h"
#include "tasks_timed.h"
#include "math.h"
#include <math.h>
#include <stdlib.h>
#include "tasks_motor.h"
#include "drivers_uartjudge_low.h"
#include "drivers_cmpower.h"
#include <stdbool.h>

extern extPowerHeatData_t PowerHeatData;
extern uint8_t JUDGE_State;
//extern uint8_t going;

extern bool g_bInited;

void setMotor(MotorId motorId, int16_t Intensity){
	static int16_t CMFLIntensity = 0, CMFRIntensity = 0, CMBLIntensity = 0, CMBRIntensity = 0;
	static int8_t CMReady = 0;
	
	static int16_t GMYAWIntensity = 0, GMPITCHIntensity = 0;
	static int8_t GMReady = 0;
	
	static int16_t PM1Intensity = 0, PM2Intensity = 0, PM3Intensity = 0;
	static int8_t PMReady = 0;
	static int8_t PM2Ready = 0;
	static int8_t PM3Ready = 0;
	
	static int16_t AM1LIntensity = 0;
	static int16_t AM1RIntensity = 0;
	static int16_t AM2LIntensity = 0;
	static int16_t AM2RIntensity = 0;
	static int16_t AM3RIntensity = 0;
	static int8_t AMReady = 0;
	extern uint8_t going;
	
	switch(motorId)
	{
//		case CMFL:
//			if(CMReady & 0x1){CMReady = 0xF;}else{CMReady |= 0x1;}
//			CMFLIntensity = Intensity;break;
//		case CMFR:
//			if(CMReady & 0x2){CMReady = 0xF;}else{CMReady |= 0x2;}
//			CMFRIntensity = Intensity;break;
//		case CMBL:
//			if(CMReady & 0x4){CMReady = 0xF;}else{CMReady |= 0x4;}
//			CMBLIntensity = Intensity;break;
//		case CMBR:
//			if(CMReady & 0x8){CMReady = 0xF;}else{CMReady |= 0x8;}
//			CMBRIntensity = Intensity;break;
		case CMFL:
			if(CMReady & 0x1){CMReady = 0xF;}else{CMReady |= 0x1;}
			CMFLIntensity = Intensity;break;
		case CMFR:
			if(CMReady & 0x2){CMReady = 0xF;}else{CMReady |= 0x2;}
			CMFRIntensity = Intensity;break;
		case CMBL:
			if(CMReady & 0x4){CMReady = 0xF;}else{CMReady |= 0x4;}
			CMBLIntensity = Intensity;break;
		case CMBR:
			if(CMReady & 0x8){CMReady = 0xF;}else{CMReady |= 0x8;}
			CMBRIntensity = Intensity;break;		
		
		case GMYAW:
			if(GMReady & 0x1){GMReady = 0x3;}else{GMReady |= 0x1;}
			GMYAWIntensity = Intensity;break;
		case GMPITCH:
			if(GMReady & 0x2){GMReady = 0x3;}else{GMReady |= 0x2;}
			GMPITCHIntensity = Intensity;break;
		
//		case PM1:
//			if(PMReady & 0x1){}else{PMReady |= 0x1;}
//			PM1Intensity = Intensity;break;   
//PM1Intensity = 1000;break;   
		case PM1:
			if(PM2Ready & 0x2){}else{PM2Ready |= 0x2;}
		//	PM1Intensity = 6000;break; 	
			PM1Intensity = Intensity;break; 			
		case PM2:
			if(PM2Ready & 0x1){}else{PM2Ready |= 0x1;}
			PM2Intensity = Intensity;break;		
//		case PM3:
//			if(PM3Ready & 0x1){PM3Ready = 0x1;}else{PM3Ready |= 0x1;}
//			PM3Intensity = Intensity;break;
		
		case AM1L:
			if(AMReady & 0x01){}else{AMReady |= 0x01;}
			AM1LIntensity = Intensity;break;
		case AM1R:
			if(AMReady & 0x02){}else{AMReady |= 0x02;}
			AM1RIntensity = Intensity;break;
		case AM2L:
			if(AMReady & 0x04){}else{AMReady |= 0x04;}
			AM2LIntensity = Intensity;break;
		case AM2R:
			if(AMReady & 0x08){}else{AMReady |= 0x08;}
			AM2RIntensity = Intensity;break;
		case AM3R:
			if(AMReady & 0x10){}else{AMReady |= 0x10;}
			AM3RIntensity = Intensity;break;
			
		default:
			fw_Error_Handler();
	}

	//底盘功率限制，80W，能量槽满60，低于0掉血
//    RestrictPower(&CMFLIntensity, &CMFRIntensity, &CMBLIntensity, &CMBRIntensity);
	float CM_current_max = CM_current_MAX;
	float CMFLIntensity_max = CMFLIntensity_MAX;
	float CMFRIntensity_max = CMFRIntensity_MAX;
	float CMBLIntensity_max = CMBLIntensity_MAX;
	float CMBRIntensity_max = CMBRIntensity_MAX;

	if (PowerHeatData.chassisPowerBuffer > 15 & PowerHeatData.chassisPowerBuffer < 40){
			
		 CM_current_max = CM_current_lower;
		 CMFLIntensity_max = CMFLIntensity_lower;
		 CMFRIntensity_max = CMFRIntensity_lower;
		 CMBLIntensity_max = CMBLIntensity_lower;
		 CMBRIntensity_max = CMBRIntensity_lower;
	}
	
	if (PowerHeatData.chassisPowerBuffer < 15 ){
			
		 CM_current_max = 0;
		 CMFLIntensity_max = 0;
		 CMFRIntensity_max = 0;
		 CMBLIntensity_max = 0;
		 CMBRIntensity_max = 0;
	}

//	if (PowerHeatData.chassisPowerBuffer < 10 ){
//			
//		 CM_current_max = CM_current_bottom;
//		 CMFLIntensity_max = CMFLIntensity_bottom;
//		 CMFRIntensity_max = CMFRIntensity_bottom;
//		 CMBLIntensity_max = CMBLIntensity_bottom;
//		 CMBRIntensity_max = CMBRIntensity_bottom;
//	}

	if (JUDGE_State == OFFLINE)
	{
		 CM_current_max = 19000;
		 CMFLIntensity_max = 5000;
		 CMFRIntensity_max = 5000;
		 CMBLIntensity_max = 5000;
		 CMBRIntensity_max = 5000;
	}
	if(going)
	{
		CM_current_max = 53200;
		 CMFLIntensity_max = 13200;
		 CMFRIntensity_max = 13200;
		 CMBLIntensity_max = 13200;
		 CMBRIntensity_max = 13200;
	}
//	CM_current_max = 38000;
//		 CMFLIntensity_max = 7700;
//		 CMFRIntensity_max = 7700;
//		 CMBLIntensity_max = 00;
//		 CMBRIntensity_max = 9800;
	

	float sum = (abs(CMFLIntensity) + abs(CMFRIntensity) + abs(CMBLIntensity) + abs(CMBRIntensity));
	
	if ((CMFLIntensity > CMFLIntensity_max))
	{
		CMFLIntensity = CMFLIntensity_max;
	}
	else if ((CMFLIntensity < -CMFLIntensity_max))
	{
		CMFLIntensity = -CMFLIntensity_max;
	}
	if(CMFRIntensity > CMFRIntensity_max)
	{
	  CMFRIntensity = CMFRIntensity_max;
	}
	else if(CMFRIntensity < -CMFRIntensity_max)
	{
	  CMFRIntensity = -CMFRIntensity_max;
	}
	if(CMBLIntensity > CMBLIntensity_max)
	{
	  CMBLIntensity = CMBLIntensity_max;
	}
	else if(CMBLIntensity < -CMBLIntensity_max)
	{
	  CMBLIntensity = -CMBLIntensity_max;
	}
	if(CMBRIntensity > CMBRIntensity_max)
	{
	  CMBRIntensity = CMBRIntensity_max;
	}
	else	if(CMBRIntensity < -CMBRIntensity_max)
	{
	  CMBRIntensity = -CMBRIntensity_max;
	}
	if( sum > CM_current_max){
		CMFLIntensity = (CM_current_max/sum)*CMFLIntensity;
		CMFRIntensity = (CM_current_max/sum)*CMFRIntensity;
		CMBLIntensity = (CM_current_max/sum)*CMBLIntensity;
		CMBRIntensity = (CM_current_max/sum)*CMBRIntensity;
	}
	
	if(GetWorkState() == STOP_STATE || g_bInited != 1)
	{
		CMFLIntensity = 0;
		CMFRIntensity = 0;
		CMBLIntensity = 0;
		CMBRIntensity = 0;
		GMYAWIntensity = 0;
		GMPITCHIntensity = 0;
		PM1Intensity = 0;
		PM2Intensity = 0;
		PM3Intensity = 0;
		AM1LIntensity = 0;
		AM1RIntensity = 0;
		AM2LIntensity = 0;
		AM2RIntensity = 0;
		AM3RIntensity = 0;
	}

	if(CMReady == 0xF)
	{
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(CMTxIOPool);
		
		pData->StdId = CM_TXID;
  	pData->Data[0] = (uint8_t)(CMFLIntensity >> 8);
		pData->Data[1] = (uint8_t)CMFLIntensity;
		pData->Data[2] = (uint8_t)(CMFRIntensity >> 8);
		pData->Data[3] = (uint8_t)CMFRIntensity;
		pData->Data[4] = (uint8_t)(CMBLIntensity >> 8);
		pData->Data[5] = (uint8_t)CMBLIntensity;
		pData->Data[6] = (uint8_t)(CMBRIntensity >> 8);
		pData->Data[7] = (uint8_t)CMBRIntensity;
 	
		IOPool_getNextWrite(CMTxIOPool);
		
		TransmitCAN1();
		CMReady = 0;
    }
	
	if(GMReady == 0x3)
	{
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(GMTxIOPool);
		pData->StdId = GM_TXID;
//		pData->Data[0] = (uint8_t)(GMYAWIntensity >> 8);
//		pData->Data[1] = (uint8_t)GMYAWIntensity;
//		pData->Data[2] = (uint8_t)(GMPITCHIntensity >> 8);
//		pData->Data[3] = (uint8_t)GMPITCHIntensity;
		pData->Data[0] = (uint8_t)(GMPITCHIntensity >> 8);
		pData->Data[1] = (uint8_t)GMPITCHIntensity;
		pData->Data[2] = (uint8_t)(GMYAWIntensity >> 8);
		pData->Data[3] = (uint8_t)GMYAWIntensity;
		pData->Data[4] = 0;
		pData->Data[5] = 0;
		pData->Data[6] = 0;
		pData->Data[7] = 0;
		IOPool_getNextWrite(GMTxIOPool);	
		
    TransmitCAN1();
		GMReady = 0;
	}
	
//	if(PMReady == 0x1)
//	{
//		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(PMTxIOPool);
//		pData->StdId = PM_TXID;
//		pData->Data[0] = (uint8_t)(PM1Intensity >> 8);
//		pData->Data[1] = (uint8_t)PM1Intensity;
//		pData->Data[2] = 0;
//		pData->Data[3] = 0;
//		pData->Data[4] = 0;
//		pData->Data[5] = 0;
//		pData->Data[6] = 0;
//		pData->Data[7] = 0;
//		IOPool_getNextWrite(PMTxIOPool);
//		
//		TransmitCAN1();
//		PMReady = 0;
//	}
	
		if(PM2Ready == 0x3)
	{
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(PM2TxIOPool);
		pData->StdId = PM2_TXID;
		pData->Data[0] = 0;
		pData->Data[1] = 0;
		pData->Data[2] = 0;
		pData->Data[3] = 0;
		pData->Data[4] = 0;
		pData->Data[5] = 0;
		pData->Data[6] = (uint8_t)(PM2Intensity >> 8);
		pData->Data[7] = (uint8_t)PM2Intensity;
		IOPool_getNextWrite(PM2TxIOPool);
		
		pData = IOPool_pGetWriteData(PMTxIOPool);
		pData->StdId = PM_TXID;
		pData->Data[0] = 0;
		pData->Data[1] = 0;
		pData->Data[2] = 0;
		pData->Data[3] = 0;
		pData->Data[4] = (uint8_t)(PM1Intensity >> 8);
		pData->Data[5] = (uint8_t)PM1Intensity;
		pData->Data[6] = 0;
		pData->Data[7] = 0;
		IOPool_getNextWrite(PMTxIOPool);
		
		TransmitCAN2();
		PM2Ready = 0;
	}
//	if(PM3Ready == 0x1)
//	{
//		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(PM3TxIOPool);
//		pData->StdId = PM3_TXID;
//		pData->Data[0] = 0;
//		pData->Data[1] = 0;
//		pData->Data[2] = 0;
//		pData->Data[3] = 0;
//		pData->Data[4] = 0;
//		pData->Data[5] = 0;
//		pData->Data[6] = (uint8_t)(PM3Intensity >> 8);
//		pData->Data[7] = (uint8_t)PM3Intensity;
//		IOPool_getNextWrite(PM3TxIOPool);
//		
//		TransmitCAN2();
//		PM3Ready = 0;
//	}
	
	if(AMReady == 0x1F)
	{
		CanTxMsgTypeDef *pData = IOPool_pGetWriteData(AM1TxIOPool);
		pData->StdId = AM1_TXID;
		pData->Data[0] = (uint8_t)(AM1LIntensity >> 8);
		pData->Data[1] = (uint8_t)AM1LIntensity;
		pData->Data[2] = (uint8_t)(AM1RIntensity >> 8);
		pData->Data[3] = (uint8_t)AM1RIntensity;
		pData->Data[4] = 0;
		pData->Data[5] = 0;
		pData->Data[6] = 0;
		pData->Data[7] = 0;
		IOPool_getNextWrite(AM1TxIOPool);
		
		pData = IOPool_pGetWriteData(AM23TxIOPool);
		pData->StdId = AM23_TXID;
		pData->Data[0] = (uint8_t)(AM2LIntensity >> 8);
		pData->Data[1] = (uint8_t)AM2LIntensity;
		pData->Data[2] = (uint8_t)(AM2RIntensity >> 8);
		pData->Data[3] = (uint8_t)AM2RIntensity;
		pData->Data[4] = (uint8_t)(AM3RIntensity >> 8);
		pData->Data[5] = (uint8_t)AM3RIntensity;
		pData->Data[6] = 0;
		pData->Data[7] = 0;
		IOPool_getNextWrite(AM23TxIOPool);
		
		TransmitCAN2();
		AMReady = 0;
		
	}
}
	
//删除所有的GYRO
//void GYRO_RST(void)
//{
//	CanTxMsgTypeDef *pData = IOPool_pGetWriteData(ZGYROTxIOPool);
//	pData->StdId = ZGYRO_TXID;
//	pData->Data[0] = 0x00;
//	pData->Data[1] = 0x01;
//	pData->Data[2] = 0x02;
//	pData->Data[3] = 0x03;
//	pData->Data[4] = 0x04;
//	pData->Data[5] = 0x05;
//	pData->Data[6] = 0x06;
//	pData->Data[7] = 0x07;
//	IOPool_getNextWrite(ZGYROTxIOPool);

//	TransmitCAN2();
//}
