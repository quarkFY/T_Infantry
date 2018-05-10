/**
  ******************************************************************************
  * File Name          : tasks_motor.c
  * Description        : CAN1电机控制任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 云台、底盘、推弹电机控制任务
	* 处于阻塞态等待CAN接收任务释放信号量
	* 对CAN收到的数据进行PID计算，再将电流值发送到CAN
  ******************************************************************************
  */
#include "tasks_motor.h"
#include "drivers_canmotor_user.h"
#include "rtos_semaphore.h"
#include "drivers_uartrc_user.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <usart.h>
#include "utilities_debug.h"
#include "tasks_upper.h"
#include "tasks_timed.h"
#include "tasks_remotecontrol.h"
#include "drivers_led_user.h"
#include "utilities_minmax.h"
#include "pid_regulator.h"
#include "application_motorcontrol.h"
#include "drivers_sonar_user.h"
#include "peripheral_define.h"
#include "drivers_uartupper_user.h"


//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
//云台
int yaw_zero = 3500;
int yaw_zero_revise =4100;
int pitch_zero = 6300;
float yawEncoder = 0;
float GMYAWThisAngle, GMYAWLastAngle;
float yawRealAngle = 0.0;
float yawAngleTarget = 0.0;
float pitchEncoder = 0;
float GMPITCHThisAngle, GMPITCHLastAngle;
float pitchRealAngle = 0.0;
float pitchAngleTarget = 0.0;
float pitchMotorTarget = 0.0;
int GMPITCHCurrent,GMYAWCurrent;
uint8_t This_GM_RST,Last_GM_RST;

int isGMYAWFirstEnter = 1;
int isGMPITCHFirstEnter = 1;
int isGMSet;

fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(100, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(100.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(5.0, 0.0, 5.0, 3000.0, 10000.0, 10000.0, 5000);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(5.0, 0.0, 5.0, 3000.0, 10000.0, 10000.0, 5000.0);

//底盘
PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

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
uint8_t isPM1FirstEnter = 1;
uint16_t PM2ThisAngle = 0;
uint16_t PM2LastAngle = 0;
uint16_t PM3ThisAngle = 0;
uint16_t PM3LastAngle = 0;
uint8_t isPM2FirstEnter = 1;
uint8_t isPM3FirstEnter = 1;
fw_PID_Regulator_t PM1PositionPID = fw_PID_INIT(120, 0.0, 200.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t PM2PositionPID = fw_PID_INIT(100.0, 0.0, 200.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t PM1SpeedPID = fw_PID_INIT(15, 0.0, 40.0, 10000.0, 10000.0, 10000.0, 8000.0);
fw_PID_Regulator_t PM2SpeedPID = fw_PID_INIT(15, 0.0, 40.0, 10000.0, 10000.0, 10000.0, 8000.0);
fw_PID_Regulator_t PM3PositionPID = fw_PID_INIT(100.0, 0.0, 200.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t PM3SpeedPID = fw_PID_INIT(15, 0.0, 40.0, 10000.0, 10000.0, 10000.0, 8000.0);

uint16_t PM1RotateCount = 0;
uint8_t PM1RotateFlag = 0;

//陀螺仪角速度（板载）
extern float gYroXs, gYroYs, gYroZs;

//extern uint8_t g_isGYRO_Rested;//没用到
//外接单轴陀螺仪角度
//extern float ZGyroModuleAngle;	//这就是yawRealAngle

float gap_angle = 0.0;
float rotateSpeed = 0.0;

//减小系统开销
static uint8_t s_yawCount = 0;
static uint8_t s_pitchCount = 0;
static uint8_t s_CMFLCount = 0;
static uint8_t s_CMFRCount = 0;
static uint8_t s_CMBLCount = 0;
static uint8_t s_CMBRCount = 0;
static uint8_t s_PM1Count = 0;
static uint8_t s_PM2Count = 0;
static uint8_t s_PM3Count = 0;



void Can1ControlTask(void const * argument)
{
	while(1)
	{
		//等待CAN接收回调函数信号量
		//fw_printfln("Can1ControlTask is working");
		
		osSemaphoreWait(Can1RefreshSemaphoreHandle, osWaitForever);

		//ChassisSpeedRef.rotate_ref = 0;//取消底盘跟随
		ControlCMFL();
		ControlCMFR();
		ControlCMBL();
		ControlCMBR();
		
		ControlPitch();		
		ControlYaw();
		
		ControlPM1();
		ControlPM2();
		
		ControlPM3();
		
	  }//end of while
}


int16_t yawIntensityForDebug = 0;
/*Yaw电机*/
void ControlYaw(void)
{
	int16_t yawIntensity = 0;
	if(IOPool_hasNextRead(GMYAWRxIOPool, 0))
	{
		if(s_yawCount == 3)
		{
			int16_t yawZeroAngle = 0;
      yawZeroAngle = yaw_zero;
			
			/*从IOPool读编码器*/
			IOPool_getNextRead(GMYAWRxIOPool, 0); 
			//fw_printfln("yaw%d",IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle);
			//yawRealAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle- yawZeroAngle) * 360 * 11 / (8192.0f * 50);
			
			GMYAWThisAngle = IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle;
			GMYAWCurrent = IOPool_pGetWriteData(GMYAWRxIOPool)->realIntensity;
			yawEncoder = IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle;
			if(isGMYAWFirstEnter==1) 
			{
				GMYAWLastAngle = GMYAWThisAngle;
				yawRealAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle- yawZeroAngle) * 360 * 11 / (8192.0f * 50);
				isGMYAWFirstEnter = 0;
			}	//初始化时，记录下当前编码器的值
			
			if(GMYAWThisAngle <= GMYAWLastAngle)
			{
				if((GMYAWLastAngle-GMYAWThisAngle)>3000)//编码器上溢
					yawRealAngle = yawRealAngle + (GMYAWThisAngle+8192-GMYAWLastAngle) * 360 * 11 / (8192.0 * 50) ;
				else//反转
				 yawRealAngle =  yawRealAngle + (GMYAWThisAngle - GMYAWLastAngle) * 360 * 11 / (8192.0f * 50) ;
			}
			else
			{
				if((GMYAWThisAngle-GMYAWLastAngle)>3000)//编码器下溢
					yawRealAngle = yawRealAngle - (GMYAWLastAngle+8192-GMYAWThisAngle) * 360 * 11 / (8192.0 * 50)  ;
				else//正转
					yawRealAngle = yawRealAngle + (GMYAWThisAngle - GMYAWLastAngle) * 360 * 11 / (8192.0f * 50) ;
			}
							
			//NORMALIZE_ANGLE180(yawRealAngle);
			//限位
			MINMAX(yawAngleTarget, -30.0f, 30.0f);	
			yawIntensity = ProcessYawPID(yawAngleTarget, yawRealAngle, -gYroZs);
			yawIntensityForDebug = yawIntensity;
			GMYAWLastAngle = GMYAWThisAngle ;
			
//			if (isGMSet == 1)
//			{
				setMotor(GMYAW, -yawIntensity);
//			}


			s_yawCount = 0;
			
		//	ControlRotate();
		}
		else
		{
			s_yawCount++;
		}
	}
}


/*Pitch电机*/
void ControlPitch(void)
{
	int16_t pitchIntensity = 0;
	if(IOPool_hasNextRead(GMPITCHRxIOPool, 0))
	{
		if(s_pitchCount == 3)
		{
		  int16_t pitchZeroAngle = 0;
      pitchZeroAngle = pitch_zero;
		  
			/*从IOPool读编码器*/
			IOPool_getNextRead(GMPITCHRxIOPool, 0); 
			//fw_printfln("pitch%d",IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle);
			//pitchRealAngle = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle- pitchZeroAngle) * 360 * 11 / (8192.0f * 50);
			
			GMPITCHThisAngle = IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle;
			GMPITCHCurrent = IOPool_pGetWriteData(GMPITCHRxIOPool)->realIntensity;

			pitchEncoder = IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle;
			if(isGMPITCHFirstEnter==1) 
			{
				GMPITCHLastAngle = GMPITCHThisAngle;
				pitchRealAngle = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle- pitchZeroAngle) * 360 * 11 / (8192.0f * 50);
				isGMPITCHFirstEnter = 0;
			}	//初始化时，记录下当前编码器的值
			
			if(GMPITCHThisAngle <= GMPITCHLastAngle)
			{
				if((GMPITCHLastAngle-GMPITCHThisAngle)>3000)//编码器上溢
					pitchRealAngle = pitchRealAngle + (GMPITCHThisAngle+8192-GMPITCHLastAngle) * 360 * 11 / (8192.0 * 50) ;
				else//反转
				 pitchRealAngle =  pitchRealAngle + (GMPITCHThisAngle - GMPITCHLastAngle) * 360 * 11 / (8192.0f * 50) ;
			}
			else
			{
				if((GMPITCHThisAngle-GMPITCHLastAngle)>3000)//编码器下溢
					pitchRealAngle = pitchRealAngle - (GMPITCHLastAngle+8192-GMPITCHThisAngle) * 360 * 11 / (8192.0 * 50)  ;
				else//正转
					pitchRealAngle = pitchRealAngle + (GMPITCHThisAngle - GMPITCHLastAngle) * 360 * 11 / (8192.0f * 50) ;
			}
			
			//NORMALIZE_ANGLE180(pitchRealAngle);
			//限位
			MINMAX(pitchAngleTarget, -10.0f, 30.0f);	
		  pitchMotorTarget = pitchAngleTarget - yawAngleTarget ; 
			pitchIntensity = ProcessPitchPID(-pitchMotorTarget,pitchRealAngle,-gYroXs);
			GMPITCHLastAngle = GMPITCHThisAngle;
	
//		  if (isGMSet == 1)
//			{
					setMotor(GMPITCH, -pitchIntensity);
//			}

			s_pitchCount = 0;
		}
		else
		{
			s_pitchCount++;
		}
	}
}
/*底盘转动控制：跟随云台等,英雄没有*/
//void ControlRotate(void)
//{
//	gap_angle  = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle - yaw_zero) * 360 / 8192.0f;
//  NORMALIZE_ANGLE180(gap_angle);	
//	
//	if(GetWorkState() == NORMAL_STATE) 
//	{			
//			/*底盘跟随编码器旋转PID计算*/		
//			 CMRotatePID.ref = 0;
//			 CMRotatePID.fdb = gap_angle;
//			 CMRotatePID.Calc(&CMRotatePID);   
//			 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
//	}
//}

/*底盘电机控制FL(ForwardLeft)FR BL BR*/
void ControlCMFL(void)
{		
	if(IOPool_hasNextRead(CMFLRxIOPool, 0))
	{
		if(s_CMFLCount == 3)
		{
			IOPool_getNextRead(CMFLRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMFLRxIOPool, 0);
			
			CM2SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 
											 - ChassisSpeedRef.left_right_ref*0.075 
											 - ChassisSpeedRef.rotate_ref
			                 ;
			CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;
			
	
			CM2SpeedPID.fdb = pData->RotateSpeed;
			CM2SpeedPID.Calc(&CM2SpeedPID);
			
			setMotor(CMFR, -CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output);
			
			s_CMFLCount = 0;
		}
		else
		{
			s_CMFLCount++;
		}
	}
}

void ControlCMFR(void)
{
	if(IOPool_hasNextRead(CMFRRxIOPool, 0))
	{
		if(s_CMFRCount == 3)
		{
			IOPool_getNextRead(CMFRRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMFRRxIOPool, 0);
			
			CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 
											 - ChassisSpeedRef.left_right_ref*0.075 
											 - ChassisSpeedRef.rotate_ref
			                 ;	
			CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;
			CM1SpeedPID.fdb = pData->RotateSpeed;
			
			CM1SpeedPID.Calc(&CM1SpeedPID);
			
			setMotor(CMFL, -CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output);
			
			s_CMFRCount = 0;
		}
		else
		{
			s_CMFRCount++;
		}
	}
}
	
void ControlCMBL(void)
{
	if(IOPool_hasNextRead(CMBLRxIOPool, 0))
	{
		if(s_CMBLCount == 3)
		{
			IOPool_getNextRead(CMBLRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMBLRxIOPool, 0);
			
			CM3SpeedPID.ref = -ChassisSpeedRef.forward_back_ref*0.075 
											 + ChassisSpeedRef.left_right_ref*0.075 
											 - ChassisSpeedRef.rotate_ref
			                 ;
			CM3SpeedPID.ref = 160 * CM3SpeedPID.ref;
			CM3SpeedPID.fdb = pData->RotateSpeed;
			
			CM3SpeedPID.Calc(&CM3SpeedPID);
			
			setMotor(CMBL, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output);
			
			s_CMBLCount = 0;
		}
		else
		{
			s_CMBLCount++;
		}
	}
}

void ControlCMBR()
{
	if(IOPool_hasNextRead(CMBRRxIOPool, 0))
	{
		if(s_CMBRCount ==3)
		{
			IOPool_getNextRead(CMBRRxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(CMBRRxIOPool, 0);
			
			CM4SpeedPID.ref = ChassisSpeedRef.forward_back_ref*0.075 
											 + ChassisSpeedRef.left_right_ref*0.075 
											 - ChassisSpeedRef.rotate_ref
			                 ;
			CM4SpeedPID.ref = 160 * CM4SpeedPID.ref;
			CM4SpeedPID.fdb = pData->RotateSpeed;
					
			CM4SpeedPID.Calc(&CM4SpeedPID);
			
			setMotor(CMBR, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);
			
			s_CMBRCount = 0;
		}
		else
		{
			s_CMBRCount++;
		}
	}
}


void ControlPM1()
{
	if(IOPool_hasNextRead(PM1RxIOPool, 0))
	{
		if(s_PM1Count == 3)
		{
			IOPool_getNextRead(PM1RxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(PM1RxIOPool, 0);
			PM1ThisAngle = pData->angle;
			
			if(isPM1FirstEnter) {PM1LastAngle = PM1ThisAngle;isPM1FirstEnter = 0;}
			
			if(PM1ThisAngle<=PM1LastAngle)
			{
				if((PM1LastAngle-PM1ThisAngle)>3000)//编码器上溢
					PM1RealAngle = PM1RealAngle + (PM1ThisAngle+8192-PM1LastAngle) * 360 / 8192.0 / PM1Reduction;
				else//反转
					PM1RealAngle = PM1RealAngle - (PM1LastAngle - PM1ThisAngle) * 360 / 8192.0 / PM1Reduction;
			}
			else
			{
				if((PM1ThisAngle-PM1LastAngle)>3000)//编码器下溢
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
		if(s_PM2Count == 3)
		{
			IOPool_getNextRead(PM2RxIOPool, 0);
			Motor820RRxMsg_t *pData = IOPool_pGetReadData(PM2RxIOPool, 0);
			
			PM2ThisAngle = pData->angle;
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
void shootOneGolf()
{
	heatJudge();
	//PM1是下边电机
	if(DirOfRotate)
	{
	 if((PM1RealAngle-PM1AngleTarget)>300 || (PM2AngleTarget-PM2RealAngle)>300)
		{
			DirOfRotate = !DirOfRotate;
			PM1AngleTarget = PM1RealAngle;
			PM2AngleTarget = PM2RealAngle;
		}
		else
		{
			if (heatFlag == 1)
			{
				PM1AngleTarget = PM1AngleTarget - 160;
				PM2AngleTarget = PM2AngleTarget + 160;
			}
		}
	}
	else
	{
		PM2AngleTarget = PM2AngleTarget - 360;
//	  PM2AngleTarget = PM2AngleTarget - 160;
		if((PM1AngleTarget-PM1RealAngle)<60)
		{
	    DirOfRotate = !DirOfRotate;
			PM1AngleTarget = PM1RealAngle;
		}
	}
	
	
}

void shootOneGolfConpensation()
{
	if((PM1RealAngle-PM1AngleTarget)>300 || (PM2AngleTarget-PM2RealAngle)>300)
		{
			
		}
		else
		{
	PM2AngleTarget = PM2AngleTarget - 30;
		}
}

RotateDirection_e PMRotateDirection = CLOCKWISE;

void PMRotate()
{
	if(PM1RotateFlag == 1)
	{
		switch(PMRotateDirection)
		{
			case CLOCKWISE:
			{
				PM2AngleTarget = PM2AngleTarget - 90;
				PMRotateDirection = ANTICLOCKWISE;
			}break;
			case ANTICLOCKWISE:
			{
				PM2AngleTarget = PM2AngleTarget + 90;
				PMRotateDirection = CLOCKWISE;
			}break;
		}
		
		PM1RotateFlag = 0;
	}
}

void spitOneBullet()
{
	PM3AngleTarget+=30;
}

//可能需要除抖，待调试
void GetGMRealZero(void)
{
	This_GM_RST = HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4);
	if(This_GM_RST != Last_GM_RST)
	{
		IOPool_getNextRead(GMYAWRxIOPool, 0); 
		yaw_zero_revise = IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle;
	}
	Last_GM_RST = This_GM_RST;
}
	
void GMReset(void)
{
	yaw_zero = yaw_zero_revise;
}

