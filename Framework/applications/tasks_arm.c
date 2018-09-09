/**
  ******************************************************************************
  * File Name          : tasks_arm.c
  * Description        : CAN2电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 取弹机械臂电机控制任务
	* 处于阻塞态等待CAN接收任务释放信号量
	* 对CAN收到的数据进行PID计算，再将电流值发送到CAN
  ******************************************************************************
  */
	
#include "tasks_arm.h"
#include "rtos_semaphore.h"
#include "pid_regulator.h"
#include "utilities_iopool.h"
#include "drivers_canmotor_user.h"
#include "application_motorcontrol.h"
#include "utilities_debug.h"
#include "tasks_remotecontrol.h"
#include "drivers_uartrc_user.h"
#include "peripheral_sov.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <usart.h>

//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
//机械臂电机PID

fw_PID_Regulator_t AM1LPositionPID = fw_PID_INIT(160.0, 0.0, 0.0, 20000.0, 10000.0, 10000.0, 16384.0);
fw_PID_Regulator_t AM1RPositionPID = fw_PID_INIT(160.0, 0.0, 0.0, 20000.0, 10000.0, 10000.0, 16384.0);


fw_PID_Regulator_t AM3RPositionPID = fw_PID_INIT(80.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);

fw_PID_Regulator_t AM1LSpeedPID = fw_PID_INIT(12.0, 0.0, 0.0, 20000.0, 10000.0, 10000.0, 16384.0);
fw_PID_Regulator_t AM1RSpeedPID = fw_PID_INIT(12.0, 0.0, 0.0, 20000.0, 10000.0, 10000.0, 16384.0);


fw_PID_Regulator_t AM3RSpeedPID = fw_PID_INIT(8.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 8000.0);

extern float PM1AngleTarget;
extern Emergency_Flag emergency_Flag;
extern int16_t PM2ThisTorque;

#define LengthOfArm1 500
#define LengthOfArm2 250
#define PI 3.141592653589


float am1rreal,am1lreal;
//机械臂电机目标物理角度值
float AM1LAngleTarget = 0.0;
float AM1RAngleTarget = 0.0;


float AM3RAngleTarget = 0.0;

double Last_Arm_Horizontal_Position;
double Last_Arm_Vertical_Position;

float LastAM1LAngleTarget;
float LastAM1RAngleTarget;


float LastAM3RAngleTarget;

//机械臂电机实际物理角度值
float AM1LRealAngle = 0.0;
float AM1RRealAngle = 0.0;


float AM3RRealAngle = 0.0;


//用于减小系统开销
static uint8_t s_AM1LCount = 0;
static uint8_t s_AM1RCount = 0;


static uint8_t s_AM3RCount = 0;

//推弹电机
#define PM1ZeroAngle 0
#define PM2ZeroAngle 0
#define PM3ZeroAngle 0
float PM1RealAngle = 0.0;
float PM2RealAngle = 0.0;
float PM3RealAngle = 0.0;
float PM1AngleTarget = 0.0;
float PM2AngleTarget = 0.0;
float PM3AngleTarget = 0.0;
uint16_t PM1ThisAngle = 0;
uint16_t PM1LastAngle = 0;
int16_t PM1ThisTorque = 0;
uint8_t isPM1FirstEnter = 1;
uint16_t PM2ThisAngle = 0;
uint16_t PM2LastAngle = 0;
int16_t PM2ThisTorque = 0;
uint16_t PM3ThisAngle = 0;
uint16_t PM3LastAngle = 0;
uint8_t isPM2FirstEnter = 1;
uint8_t isPM3FirstEnter = 1;
complete_PID_Regulator_t PM1PositionPID = complete_PID_INIT(150.0, 1, 200.0, 10000.0, 10000.0, 10000.0, 8000.0);
fw_PID_Regulator_t PM2PositionPID = fw_PID_INIT(100.0, 0.0, 200.0, 10000.0, 10000.0, 10000.0, 10000.0);
complete_PID_Regulator_t PM1SpeedPID = complete_PID_INIT(100.0, 0, 100.0, 10000.0, 10000.0, 10000.0, 8000.0);
fw_PID_Regulator_t PM2SpeedPID = fw_PID_INIT(15, 0.0, 40.0, 10000.0, 10000.0, 10000.0, 8000.0);
fw_PID_Regulator_t PM3PositionPID = fw_PID_INIT(250.0, 0.0, 200.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t PM3SpeedPID = fw_PID_INIT(150, 0.0, 40.0, 10000.0, 10000.0, 10000.0, 8000.0);

uint16_t PM1RotateCount = 0;
uint8_t PM1RotateFlag = 0;

static uint8_t s_PM1Count = 0;
static uint8_t s_PM2Count = 0;
static uint8_t s_PM3Count = 0;

void Can2ControlTask(void const * argument)
{
	while(1)
	{
		osSemaphoreWait(Can2RefreshSemaphoreHandle, osWaitForever);
		
		ControlAM1L();
		ControlAM1R();
		
		ControlPM1();
		ControlPM2();
		ControlPM3();		
	}
}


uint8_t isAM1LFirstEnter = 1;
uint16_t AM1LThisAngle = 0;
uint16_t AM1LLastAngle = 0;
void ControlAM1L()
{
	if(IOPool_hasNextRead(AM1LRxIOPool, 0))
	{
		if(s_AM1LCount == 1)
		{		
			IOPool_getNextRead(AM1LRxIOPool, 0);
			AM1LThisAngle = IOPool_pGetReadData(AM1LRxIOPool, 0)->angle;
			am1lreal = IOPool_pGetReadData(AM1LRxIOPool, 0)->realIntensity;
			
			if(isAM1LFirstEnter==1) {AM1LLastAngle = AM1LThisAngle;isAM1LFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM1LThisAngle<=AM1LLastAngle)
			{
				if((AM1LLastAngle-AM1LThisAngle)>3000)//编码器上溢
					AM1LRealAngle = AM1LRealAngle + (AM1LThisAngle+8192-AM1LLastAngle) * 360 / 8192.0 / AM1Reduction;
				else//反转
					AM1LRealAngle = AM1LRealAngle - (AM1LLastAngle - AM1LThisAngle) * 360 / 8192.0 / AM1Reduction;
			}
			else
			{
				if((AM1LThisAngle-AM1LLastAngle)>3000)//编码器下溢
					AM1LRealAngle = AM1LRealAngle - (AM1LLastAngle+8192-AM1LThisAngle) *360 / 8192.0 / AM1Reduction;
				else//正转
					AM1LRealAngle = AM1LRealAngle + (AM1LThisAngle - AM1LLastAngle) * 360 / 8192.0 / AM1Reduction;
			}
			
				
			AM1LPositionPID.target = AM1LAngleTarget;
			AM1LPositionPID.feedback = AM1LRealAngle;
			AM1LPositionPID.Calc(&AM1LPositionPID);
			
			AM1LSpeedPID.target = AM1LPositionPID.output;
			AM1LSpeedPID.feedback = IOPool_pGetReadData(AM1LRxIOPool, 0)->RotateSpeed;
			AM1LSpeedPID.Calc(&AM1LSpeedPID);
			
			setMotor(AM1L, AM1LSpeedPID.output);
			s_AM1LCount = 0;
			AM1LLastAngle = AM1LThisAngle;
		}
		else
		{
			s_AM1LCount++;
		}
	}
}

uint8_t isAM1RFirstEnter = 1;
uint16_t AM1RThisAngle = 0;
uint16_t AM1RLastAngle = 0;
void ControlAM1R()
{
	if(IOPool_hasNextRead(AM1RRxIOPool, 0))
	{
		if(s_AM1RCount == 1)
		{
			IOPool_getNextRead(AM1RRxIOPool, 0);
			AM1RThisAngle = IOPool_pGetReadData(AM1RRxIOPool, 0)->angle;
			am1rreal = IOPool_pGetReadData(AM1RRxIOPool, 0)->realIntensity;
			
			if(isAM1RFirstEnter==1) {AM1RLastAngle = AM1RThisAngle;isAM1RFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM1RThisAngle<=AM1RLastAngle)
			{
				if((AM1RLastAngle-AM1RThisAngle)>3000)//编码器上溢
					AM1RRealAngle = AM1RRealAngle + (AM1RThisAngle+8192-AM1RLastAngle) * 360 / 8192.0 / AM1Reduction;
				else//反转
					AM1RRealAngle = AM1RRealAngle - (AM1RLastAngle - AM1RThisAngle) * 360 / 8192.0 / AM1Reduction;
			}
			else
			{
				if((AM1RThisAngle-AM1RLastAngle)>3000)//编码器下溢
					AM1RRealAngle = AM1RRealAngle - (AM1RLastAngle+8192-AM1RThisAngle) *360 / 8192.0 / AM1Reduction;
				else//正转
					AM1RRealAngle = AM1RRealAngle + (AM1RThisAngle - AM1RLastAngle) * 360 / 8192.0 / AM1Reduction;
			}
			
			
			AM1RPositionPID.target = AM1RAngleTarget;
			AM1RPositionPID.feedback = AM1RRealAngle;
			AM1RPositionPID.Calc(&AM1RPositionPID);
			
			AM1RSpeedPID.target = AM1RPositionPID.output;
			AM1RSpeedPID.feedback = IOPool_pGetReadData(AM1RRxIOPool, 0)->RotateSpeed;
			AM1RSpeedPID.Calc(&AM1RSpeedPID);
			
			setMotor(AM1R, AM1RSpeedPID.output);
			s_AM1RCount = 0;
			AM1RLastAngle = AM1RThisAngle;
		}
		else
		{
			s_AM1RCount++;
		}
	}
}

float AM3intensity;
uint8_t isAM3RFirstEnter = 1;
uint16_t AM3RThisAngle = 0;
uint16_t AM3RLastAngle = 0;
void ControlAM3R()
{
	if(IOPool_hasNextRead(AM3RRxIOPool, 0))
	{
		if(s_AM3RCount == 1)
		{
			IOPool_getNextRead(AM3RRxIOPool, 0);
			AM3RThisAngle = IOPool_pGetReadData(AM3RRxIOPool, 0)->angle;
			
			if(isAM3RFirstEnter==1) {AM3RLastAngle = AM3RThisAngle;isAM3RFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM3RThisAngle<=AM3RLastAngle)
			{
				if((AM3RLastAngle-AM3RThisAngle)>4000)//编码器上溢
					AM3RRealAngle = AM3RRealAngle + (AM3RThisAngle+8192-AM3RLastAngle) * 360 / 8192.0 / AM23Reduction;
				else//反转
					AM3RRealAngle = AM3RRealAngle - (AM3RLastAngle - AM3RThisAngle) * 360 / 8192.0 / AM23Reduction;
			}
			else
			{
				if((AM3RThisAngle-AM3RLastAngle)>4000)//编码器下溢
					AM3RRealAngle = AM3RRealAngle - (AM3RLastAngle+8192-AM3RThisAngle) *360 / 8192.0 / AM23Reduction;
				else//正转
					AM3RRealAngle = AM3RRealAngle + (AM3RThisAngle - AM3RLastAngle) * 360 / 8192.0 / AM23Reduction;
			}
			
			
			AM3RPositionPID.target = AM3RAngleTarget;
			AM3RPositionPID.feedback = AM3RRealAngle;
			AM3RPositionPID.Calc(&AM3RPositionPID);
			
			AM3RSpeedPID.target = AM3RPositionPID.output;
			AM3RSpeedPID.feedback = IOPool_pGetReadData(AM3RRxIOPool, 0)->RotateSpeed;
			AM3RSpeedPID.Calc(&AM3RSpeedPID);
			
			AM3intensity = AM3RSpeedPID.output;
			setMotor(AM3R, AM3RSpeedPID.output);
			s_AM3RCount = 0;
			AM3RLastAngle = AM3RThisAngle;
		}
		else
		{
			s_AM3RCount++;
		}
	}
}

void ControlPM1()
{
	if(IOPool_hasNextRead(PM1RxIOPool, 0))
	{
		if(s_PM1Count == 1)
		{
			IOPool_getNextRead(PM1RxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(PM1RxIOPool, 0);
			PM1ThisAngle = pData->angle;
			PM1ThisTorque = pData->realIntensity;
			
			if(isPM1FirstEnter) {PM1LastAngle = PM1ThisAngle;isPM1FirstEnter = 0;}
			
			if(PM1ThisAngle<=PM1LastAngle)
			{
				if((PM1LastAngle-PM1ThisAngle)>4000)//编码器上溢
					PM1RealAngle = PM1RealAngle + (PM1ThisAngle+8192-PM1LastAngle) * 360 / 8192.0 / PM1Reduction;
				else//反转
					PM1RealAngle = PM1RealAngle - (PM1LastAngle - PM1ThisAngle) * 360 / 8192.0 / PM1Reduction;
			}
			else
			{
				if((PM1ThisAngle-PM1LastAngle)>4000)//编码器下溢
					PM1RealAngle = PM1RealAngle - (PM1LastAngle+8192-PM1ThisAngle) *360 / 8192.0 / PM1Reduction;
				else//正转
					PM1RealAngle = PM1RealAngle + (PM1ThisAngle - PM1LastAngle) * 360 / 8192.0 / PM1Reduction;
			}
			
			PM1PositionPID.feedback = PM1RealAngle;
			PM1PositionPID.target = PM1AngleTarget;
			PM1PositionPID.Calc(&PM1PositionPID);
			
			PM1SpeedPID.target = PM1PositionPID.output;
			PM1SpeedPID.feedback = pData->RotateSpeed;
			PM1SpeedPID.Calc(&PM1SpeedPID);

			PM1LastAngle = PM1ThisAngle;
	
			setMotor(PM1, PM1SpeedPID.output);
			//setMotor(PM1, 800);
			
			s_PM1Count = 0;
		}
		else
		{
			s_PM1Count++;
		}
	}
}


void ControlPM2()
{
	if(IOPool_hasNextRead(PM2RxIOPool, 0))
	{
		if(s_PM2Count == 1)
		{
			IOPool_getNextRead(PM2RxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(PM2RxIOPool, 0);
			
			PM2ThisAngle = pData->angle;
			PM2ThisTorque = pData->realIntensity;
			if(isPM2FirstEnter) {PM2LastAngle = PM2ThisAngle;isPM2FirstEnter = 0;}
			
			if(PM2ThisAngle<=PM2LastAngle)
			{
				if((PM2LastAngle-PM2ThisAngle)>3000)//编码器上溢
					PM2RealAngle = PM2RealAngle + (PM2ThisAngle+8192-PM2LastAngle) * 360 / 8192.0 / PM2Reduction;
				else//反转
					PM2RealAngle = PM2RealAngle - (PM2LastAngle - PM2ThisAngle) * 360 / 8192.0 / PM2Reduction;
			}
			else
			{
				if((PM2ThisAngle-PM2LastAngle)>3000)//编码器下溢
					PM2RealAngle = PM2RealAngle - (PM2LastAngle+8192-PM2ThisAngle) *360 / 8192.0 / PM2Reduction;
				else//正转
					PM2RealAngle = PM2RealAngle + (PM2ThisAngle - PM2LastAngle) * 360 / 8192.0 / PM2Reduction;
			}
			
			PM2PositionPID.feedback = PM2RealAngle;
			PM2PositionPID.target = PM2AngleTarget;
			PM2PositionPID.Calc(&PM2PositionPID);
			
			PM2SpeedPID.target = PM2PositionPID.output;
			PM2SpeedPID.feedback = pData->RotateSpeed;
			PM2SpeedPID.Calc(&PM2SpeedPID);

			PM2LastAngle = PM2ThisAngle;
	
			setMotor(PM2, PM2SpeedPID.output);
			
			s_PM2Count = 0;
		}
		else
		{
			s_PM2Count++;
		}
	}
}

void ControlPM3()
{
	if(IOPool_hasNextRead(PM3RxIOPool, 0))
	{
		if(s_PM3Count == 1)
		{
			IOPool_getNextRead(PM3RxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(PM3RxIOPool, 0);
			
			PM3ThisAngle = pData->angle;
			if(isPM3FirstEnter) {PM3LastAngle = PM3ThisAngle;isPM3FirstEnter = 0;}
			
			if(PM3ThisAngle<=PM3LastAngle)
			{
				if((PM3LastAngle-PM3ThisAngle)>3000)//编码器上溢
					PM3RealAngle = PM3RealAngle + (PM3ThisAngle+8192-PM3LastAngle) * 360 / 8192.0 / PM3Reduction;
				else//反转
					PM3RealAngle = PM3RealAngle - (PM3LastAngle - PM3ThisAngle) * 360 / 8192.0 / PM3Reduction;
			}
			else
			{
				if((PM3ThisAngle-PM3LastAngle)>3000)//编码器下溢
					PM3RealAngle = PM3RealAngle - (PM3LastAngle+8192-PM3ThisAngle) *360 / 8192.0 / PM3Reduction;
				else//正转
					PM3RealAngle = PM3RealAngle + (PM3ThisAngle - PM3LastAngle) * 360 / 8192.0 / PM3Reduction;
			}
			
			PM3PositionPID.feedback = PM3RealAngle;
			PM3PositionPID.target = PM3AngleTarget;
			PM3PositionPID.Calc(&PM3PositionPID);
			
			PM3SpeedPID.target = PM3PositionPID.output;
			PM3SpeedPID.feedback = pData->RotateSpeed;
			PM3SpeedPID.Calc(&PM3SpeedPID);

			PM3LastAngle = PM3ThisAngle;
	
			setMotor(PM3, PM3SpeedPID.output);
			
			s_PM3Count = 0;
		}
		else
		{
			s_PM3Count++;
		}
	}
}
uint8_t heatFlag = 0;
float heatUpperLimit = 80;
extern uint8_t realLevel;
extern float realHeat42;
void heatJudge()
{
	heatUpperLimit = 80*(pow(2,realLevel -1));
	if ((heatUpperLimit - realHeat42) >= 40)
	{
		heatFlag = 1;
	}
	else
	{
		heatFlag = 0;
	}
}
	
uint8_t DirOfRotate = 1;
float tmpPM1AngleTarget;
void shootOneGolf()
{
	heatJudge();
	if(heatFlag == 1 && fabs(PM1RealAngle-PM1AngleTarget)<180)
	{
		tmpPM1AngleTarget = PM1AngleTarget;
		PM1AngleTarget = PM1AngleTarget + 72;
	}
				//PM2AngleTarget = PM2AngleTarget + 240;
}


void shootOneGolfConpensation()
{
	if((PM1RealAngle-PM1AngleTarget)>300 || (PM2AngleTarget-PM2RealAngle)>300)
		{
			
		}
		else
		{
	//PM2AngleTarget = PM2AngleTarget - 30;
		}
}

uint8_t bulletNum = 8;
uint8_t PM1RotateFlag2 =1;
void shootLoad()
{
	PM1RotateFlag2 = 0;
	uint16_t PM2tmp = PM2AngleTarget;
//	for(uint8_t i=0;i<100;i++)
//	{
//		if(fabs(PM2AngleTarget-PM2RealAngle)< 180 )
//		{
//			PM2AngleTarget += 20;
//			osDelay(20);
//		}
////		PM2AngleTarget += 20;
////		osDelay(10);
//	}
	osDelay(100);
	for(uint8_t i=0;i<bulletNum;i++)
	{
		shootOneGolf();
		//PM2AngleTarget += 240;
		osDelay(300);
	}
	PM1RotateFlag2 = 1;
}

RotateDirection_e PMRotateDirection = CLOCKWISE;

void PMRotate()
{
	if(PM1RotateFlag2)
	{
		if(PM1RotateFlag == 1)
		{
			switch(PMRotateDirection)
			{
				case CLOCKWISE:
				{
					if(abs(PM2ThisTorque) < 7000)
						PM2AngleTarget = PM2AngleTarget + 150;
					else
						PM2AngleTarget = PM2RealAngle;
					PMRotateDirection = ANTICLOCKWISE;
				}break;
				case ANTICLOCKWISE:
				{
					if(abs(PM2ThisTorque) < 7000)
						PM2AngleTarget = PM2AngleTarget - 30;
					else
						PM2AngleTarget = PM2RealAngle;
					PMRotateDirection = CLOCKWISE;
				}break;
			}
			
			PM1RotateFlag = 0;
		}
	}	
}
