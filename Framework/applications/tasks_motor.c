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
#include "tasks_hero.h"

float 	CMBLreal,CMBRreal,CMFLreal,CMFRreal;

fw_PID_Regulator_t CMFLPositionPID = fw_PID_INIT(80.0, 0.0, 0.0, 20000.0, 10000.0, 10000.0, 16384.0);
fw_PID_Regulator_t CMFRPositionPID = fw_PID_INIT(80.0, 0.0, 0.0, 20000.0, 10000.0, 10000.0, 16384.0);
fw_PID_Regulator_t CMBLPositionPID = fw_PID_INIT(80.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t CMBRPositionPID = fw_PID_INIT(80.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);

//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
//云台
int yaw_zero = 620;
int yaw_zero_revise =2900;
int pitch_zero = 7200;
float yawEncoder = 0;
float GMYAWThisAngle, GMYAWLastAngle;
float yawRealAngle = 0.0;
float yawAngleTarget = 0.0;
float pitchEncoder = 0;
float GMPITCHThisAngle, GMPITCHLastAngle;
float pitchRealAngle = 0.0;
float pitchAngleTarget = 0.0;
float pitchMotorTarget = 0.0;
//底盘位置
float CMFLAngleTarget = 0.0;
float CMFRAngleTarget = 0.0;
float CMBLAngleTarget = 0.0;
float CMBRAngleTarget = 0.0;

float CMFLRealAngle = 0.0;
float CMFRRealAngle = 0.0;
float CMBLRealAngle = 0.0;
float CMBRRealAngle = 0.0;

int GMPITCHCurrent,GMYAWCurrent;
uint8_t This_GM_RST,Last_GM_RST;

int isGMYAWFirstEnter = 1;
int isGMPITCHFirstEnter = 1;
int isGMSet;

fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(100, 0.0, 10.0, 4000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(100.0, 0.0, 0.0, 4000.0, 10000.0, 10000.0, 10000.0);//等幅振荡P37.3 I11.9 D3.75  原26.1 8.0 1.1
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(150.0, 0.0, 0.0, 3000.0, 10000.0, 10000.0, 5000);
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(130.0, 0.0, 5.0, 3000.0, 10000.0, 10000.0, 5000.0);

//底盘
PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

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



void Can1ControlTask(void const * argument)
{
	while(1)
	{
		//等待CAN接收回调函数信号量
		//fw_printfln("Can1ControlTask is working");
		
		osSemaphoreWait(Can1RefreshSemaphoreHandle, osWaitForever);

		//ChassisSpeedRef.rotate_ref = 0;//取消底盘跟随
		
		
		if(HERO_Order == HERO_AUTO_GET3BOX)
		{
			Control_ANGLE_CMFL();
			Control_ANGLE_CMFR();
			Control_ANGLE_CMBL();
			Control_ANGLE_CMBR();
		}
		else
		{
			ControlCMFL();
			ControlCMFR();
			ControlCMBL();
			ControlCMBR();
		}
		
		
		ControlPitch();		
		ControlYaw();
		
		
		//ControlPM3();
		
	  }//end of while
}


int16_t yawIntensityForDebug = 0;
/*Yaw电机*/
void ControlYaw(void)
{
	int16_t yawIntensity = 0;
	if(IOPool_hasNextRead(GMYAWRxIOPool, 0))
	{
		if(s_yawCount == 1)
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
//		yawRealAngle = (IOPool_pGetReadData(GMYAWRxIOPool, 0)->angle- yawZeroAngle) * 360 * 1 / (8192.0f * 5);//初始化复位
				yawRealAngle = 0;
				isGMYAWFirstEnter = 0;
			}	//初始化时，记录下当前编码器的值
			
			if(GMYAWThisAngle <= GMYAWLastAngle)
			{
				if((GMYAWLastAngle-GMYAWThisAngle)>3000)//编码器上溢
					yawRealAngle = yawRealAngle + (GMYAWThisAngle+8192-GMYAWLastAngle) * 360 * 1 / (8192.0f * 5) ;
				else//反转
				 yawRealAngle =  yawRealAngle + (GMYAWThisAngle - GMYAWLastAngle) * 360 * 1 / (8192.0f * 5) ;
			}
			else
			{
				if((GMYAWThisAngle-GMYAWLastAngle)>3000)//编码器下溢
					yawRealAngle = yawRealAngle - (GMYAWLastAngle+8192-GMYAWThisAngle) * 360 * 1 / (8192.0f * 5)  ;
				else//正转
					yawRealAngle = yawRealAngle + (GMYAWThisAngle - GMYAWLastAngle) * 360 * 1 / (8192.0f * 5) ;
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
		if(s_pitchCount == 1)
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
				//pitchRealAngle = (IOPool_pGetReadData(GMPITCHRxIOPool, 0)->angle- pitchZeroAngle) * 360  / 8192.0f ; //初始化复位
				pitchRealAngle = 0;
				isGMPITCHFirstEnter = 0;
			}	//初始化时，记录下当前编码器的值
			
			if(GMPITCHThisAngle <= GMPITCHLastAngle)
			{
				if((GMPITCHLastAngle-GMPITCHThisAngle)>3000)//编码器上溢
					pitchRealAngle = pitchRealAngle + (GMPITCHThisAngle+8192-GMPITCHLastAngle) * 360  / 8192.0f  ;
				else//反转
				 pitchRealAngle =  pitchRealAngle + (GMPITCHThisAngle - GMPITCHLastAngle) * 360  / 8192.0f  ;
			}
			else
			{
				if((GMPITCHThisAngle-GMPITCHLastAngle)>3000)//编码器下溢
					pitchRealAngle = pitchRealAngle - (GMPITCHLastAngle+8192-GMPITCHThisAngle) * 360  / 8192.0f  ;
				else//正转
					pitchRealAngle = pitchRealAngle + (GMPITCHThisAngle - GMPITCHLastAngle) * 360  / 8192.0f  ;
			}
			
			//NORMALIZE_ANGLE180(pitchRealAngle);
			//限位
			MINMAX(pitchAngleTarget, 0.0f, 40.0f);	
			
//		  pitchMotorTarget = pitchAngleTarget - yawAngleTarget ;  //耦合
			pitchIntensity = ProcessPitchPID(-pitchAngleTarget,pitchRealAngle,-gYroXs); 
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
		if(s_CMFLCount == 1)
		{
			IOPool_getNextRead(CMFLRxIOPool, 0);
			MotorC620RxMsg_t *pData = IOPool_pGetReadData(CMFLRxIOPool, 0);
			
			CM2SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 
											 - ChassisSpeedRef.left_right_ref*0.075 
											 - ChassisSpeedRef.rotate_ref*0.075
			                 ;
			CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;
			
	
			CM2SpeedPID.fdb = pData->RotateSpeed;
			CM2SpeedPID.Calc(&CM2SpeedPID);
			
			setMotor(CMFR, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output);
//			setMotor(CMFR,1000);
			
			CMFLreal = IOPool_pGetReadData(CMFLRxIOPool, 0)->realIntensity;
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
		if(s_CMFRCount == 1)
		{
			IOPool_getNextRead(CMFRRxIOPool, 0);
			MotorC620RxMsg_t *pData = IOPool_pGetReadData(CMFRRxIOPool, 0);
			
			CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075 
											 - ChassisSpeedRef.left_right_ref*0.075 
											 - ChassisSpeedRef.rotate_ref*0.075
			                 ;	
			CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;
			CM1SpeedPID.fdb = pData->RotateSpeed;
			
			CM1SpeedPID.Calc(&CM1SpeedPID);
			
			setMotor(CMFL, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output);
//			setMotor(CMFL,1000);
			CMFRreal = IOPool_pGetReadData(CMFRRxIOPool, 0)->realIntensity;
			
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
		if(s_CMBLCount == 1)
		{
			IOPool_getNextRead(CMBLRxIOPool, 0);
			MotorC620RxMsg_t *pData = IOPool_pGetReadData(CMBLRxIOPool, 0);
			
			CM3SpeedPID.ref = -ChassisSpeedRef.forward_back_ref*0.075 
											 + ChassisSpeedRef.left_right_ref*0.075 
											 - ChassisSpeedRef.rotate_ref*0.075
			                 ;
			CM3SpeedPID.ref = 160 * CM3SpeedPID.ref;
			CM3SpeedPID.fdb = pData->RotateSpeed;
			
			CM3SpeedPID.Calc(&CM3SpeedPID);
			
			setMotor(CMBL, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output);
//			setMotor(CMBL,1000);
			
			CMBLreal = IOPool_pGetReadData(CMBLRxIOPool, 0)->realIntensity;
			
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
		if(s_CMBRCount ==1)
		{
			IOPool_getNextRead(CMBRRxIOPool, 0);
			MotorC620RxMsg_t *pData = IOPool_pGetReadData(CMBRRxIOPool, 0);
			
			CM4SpeedPID.ref = ChassisSpeedRef.forward_back_ref*0.075 
											 + ChassisSpeedRef.left_right_ref*0.075 
											 - ChassisSpeedRef.rotate_ref*0.075
			                 ;
			CM4SpeedPID.ref = 160 * CM4SpeedPID.ref;
			CM4SpeedPID.fdb = pData->RotateSpeed;
					
			CM4SpeedPID.Calc(&CM4SpeedPID);
			
			setMotor(CMBR, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);
//			setMotor(CMBR, 1000);
			CMBRreal = IOPool_pGetReadData(CMBRRxIOPool, 0)->realIntensity;
			s_CMBRCount = 0;
		}
		else
		{
			s_CMBRCount++;
		}
	}
}





uint8_t isCMFLFirstEnter = 1;
uint16_t CMFLThisAngle = 0;
uint16_t CMFLLastAngle = 0;
void Control_ANGLE_CMFL()
{
	if(IOPool_hasNextRead(CMFLRxIOPool, 0))
	{
		if(s_CMFLCount == 1)
		{		
			IOPool_getNextRead(CMFLRxIOPool, 0);
			CMFLThisAngle = IOPool_pGetReadData(CMFLRxIOPool, 0)->angle;
			
			if(isCMFLFirstEnter==1) {CMFLLastAngle = CMFLThisAngle;isCMFLFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(CMFLThisAngle<=CMFLLastAngle)
			{
				if((CMFLLastAngle-CMFLThisAngle)>3000)//编码器上溢
					CMFLRealAngle = CMFLRealAngle + (CMFLThisAngle+8192-CMFLLastAngle) * 360 / 8192.0 / CMReduction;
				else//反转
					CMFLRealAngle = CMFLRealAngle - (CMFLLastAngle - CMFLThisAngle) * 360 / 8192.0 / CMReduction;
			}
			else
			{
				if((CMFLThisAngle-CMFLLastAngle)>3000)//编码器下溢
					CMFLRealAngle = CMFLRealAngle - (CMFLLastAngle+8192-CMFLThisAngle) *360 / 8192.0 / CMReduction;
				else//正转
					CMFLRealAngle = CMFLRealAngle + (CMFLThisAngle - CMFLLastAngle) * 360 / 8192.0 / CMReduction;
			}
			
				
			CMFLPositionPID.target = CMFLAngleTarget;
			CMFLPositionPID.feedback = CMFLRealAngle;
			CMFLPositionPID.Calc(&CMFLPositionPID);
			
			CM2SpeedPID.ref = CMFLPositionPID.output;;
			
	
			CM2SpeedPID.fdb = IOPool_pGetReadData(CMFLRxIOPool, 0)->RotateSpeed;
			CM2SpeedPID.Calc(&CM2SpeedPID);
			
			setMotor(CMFR, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output);
			
			s_CMFLCount = 0;

			CMFLLastAngle = CMFLThisAngle;
		}
		else
		{
			s_CMFLCount++;
		}
	}
}

uint8_t isCMFRFirstEnter = 1;
uint16_t CMFRThisAngle = 0;
uint16_t CMFRLastAngle = 0;
void Control_ANGLE_CMFR()
{
	if(IOPool_hasNextRead(CMFRRxIOPool, 0))
	{
		if(s_CMFRCount == 1)
		{		
			IOPool_getNextRead(CMFRRxIOPool, 0);
			CMFRThisAngle = IOPool_pGetReadData(CMFRRxIOPool, 0)->angle;
			
			if(isCMFRFirstEnter==1) {CMFRLastAngle = CMFRThisAngle;isCMFRFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(CMFRThisAngle<=CMFRLastAngle)
			{
				if((CMFRLastAngle-CMFRThisAngle)>3000)//编码器上溢
					CMFRRealAngle = CMFRRealAngle + (CMFRThisAngle+8192-CMFRLastAngle) * 360 / 8192.0 / CMReduction;
				else//反转
					CMFRRealAngle = CMFRRealAngle - (CMFRLastAngle - CMFRThisAngle) * 360 / 8192.0 / CMReduction;
			}
			else
			{
				if((CMFRThisAngle-CMFRLastAngle)>3000)//编码器下溢
					CMFRRealAngle = CMFRRealAngle - (CMFRLastAngle+8192-CMFRThisAngle) *360 / 8192.0 / CMReduction;
				else//正转
					CMFRRealAngle = CMFRRealAngle + (CMFRThisAngle - CMFRLastAngle) * 360 / 8192.0 / CMReduction;
			}
			
				
			CMFRPositionPID.target = CMFRAngleTarget;
			CMFRPositionPID.feedback = CMFRRealAngle;
			CMFRPositionPID.Calc(&CMFRPositionPID);
			
			CM1SpeedPID.ref = CMFRPositionPID.output;;
			
	
			CM1SpeedPID.fdb = IOPool_pGetReadData(CMFRRxIOPool, 0)->RotateSpeed;
			CM1SpeedPID.Calc(&CM1SpeedPID);
			
			setMotor(CMFL, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output);
			
			s_CMFRCount = 0;

			CMFRLastAngle = CMFRThisAngle;
		}
		else
		{
			s_CMFRCount++;
		}
	}
}


uint8_t isCMBLFirstEnter = 1;
uint16_t CMBLThisAngle = 0;
uint16_t CMBLLastAngle = 0;
void Control_ANGLE_CMBL()
{
	if(IOPool_hasNextRead(CMBLRxIOPool, 0))
	{
		if(s_CMBLCount == 1)
		{		
			IOPool_getNextRead(CMBLRxIOPool, 0);
			CMBLThisAngle = IOPool_pGetReadData(CMBLRxIOPool, 0)->angle;
			
			if(isCMBLFirstEnter==1) {CMBLLastAngle = CMBLThisAngle;isCMBLFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(CMBLThisAngle<=CMBLLastAngle)
			{
				if((CMBLLastAngle-CMBLThisAngle)>3000)//编码器上溢
					CMBLRealAngle = CMBLRealAngle + (CMBLThisAngle+8192-CMBLLastAngle) * 360 / 8192.0 / CMReduction;
				else//反转
					CMBLRealAngle = CMBLRealAngle - (CMBLLastAngle - CMBLThisAngle) * 360 / 8192.0 / CMReduction;
			}
			else
			{
				if((CMBLThisAngle-CMBLLastAngle)>3000)//编码器下溢
					CMBLRealAngle = CMBLRealAngle - (CMBLLastAngle+8192-CMBLThisAngle) *360 / 8192.0 / CMReduction;
				else//正转
					CMBLRealAngle = CMBLRealAngle + (CMBLThisAngle - CMBLLastAngle) * 360 / 8192.0 / CMReduction;
			}
			
				
			CMBLPositionPID.target = CMBLAngleTarget;
			CMBLPositionPID.feedback = CMBLRealAngle;
			CMBLPositionPID.Calc(&CMBLPositionPID);
			
			CM3SpeedPID.ref = CMBLPositionPID.output;;
			
	
			CM3SpeedPID.fdb = IOPool_pGetReadData(CMBLRxIOPool, 0)->RotateSpeed;
			CM3SpeedPID.Calc(&CM3SpeedPID);
			
			setMotor(CMBL, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output);
			
			s_CMBLCount = 0;

			CMBLLastAngle = CMBLThisAngle;
		}
		else
		{
			s_CMBLCount++;
		}
	}
}

uint8_t isCMBRFirstEnter = 1;
uint16_t CMBRThisAngle = 0;
uint16_t CMBRLastAngle = 0;
void Control_ANGLE_CMBR()
{
	if(IOPool_hasNextRead(CMBRRxIOPool, 0))
	{
		if(s_CMBRCount == 1)
		{		
			IOPool_getNextRead(CMBRRxIOPool, 0);
			CMBRThisAngle = IOPool_pGetReadData(CMBRRxIOPool, 0)->angle;
			
			if(isCMBRFirstEnter==1) {CMBRLastAngle = CMBRThisAngle;isCMBRFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(CMBRThisAngle<=CMBRLastAngle)
			{
				if((CMBRLastAngle-CMBRThisAngle)>3000)//编码器上溢
					CMBRRealAngle = CMBRRealAngle + (CMBRThisAngle+8192-CMBRLastAngle) * 360 / 8192.0 / CMReduction;
				else//反转
					CMBRRealAngle = CMBRRealAngle - (CMBRLastAngle - CMBRThisAngle) * 360 / 8192.0 / CMReduction;
			}
			else
			{
				if((CMBRThisAngle-CMBRLastAngle)>3000)//编码器下溢
					CMBRRealAngle = CMBRRealAngle - (CMBRLastAngle+8192-CMBRThisAngle) *360 / 8192.0 / CMReduction;
				else//正转
					CMBRRealAngle = CMBRRealAngle + (CMBRThisAngle - CMBRLastAngle) * 360 / 8192.0 / CMReduction;
			}
			
				
			CMBRPositionPID.target = CMBRAngleTarget;
			CMBRPositionPID.feedback = CMBRRealAngle;
			CMBRPositionPID.Calc(&CMBRPositionPID);
			
			CM4SpeedPID.ref = CMBRPositionPID.output;;
			
	
			CM4SpeedPID.fdb = IOPool_pGetReadData(CMBRRxIOPool, 0)->RotateSpeed;
			CM4SpeedPID.Calc(&CM4SpeedPID);
			
			setMotor(CMBR, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);
			
			s_CMBRCount = 0;

			CMBRLastAngle = CMBRThisAngle;
		}
		else
		{
			s_CMBRCount++;
		}
	}
}

//void ControlPM3()
//{
//	if(IOPool_hasNextRead(PM3RxIOPool, 0))
//	{
//		if(s_PM3Count == 1)
//		{
//			IOPool_getNextRead(PM3RxIOPool, 0);
//			Motor820RRxMsg_t *pData = IOPool_pGetReadData(PM3RxIOPool, 0);
//			
//			PM3ThisAngle = pData->angle;
//			if(isPM3FirstEnter) {PM3LastAngle = PM3ThisAngle;isPM3FirstEnter = 0;}
//			
//			if(PM3ThisAngle<=PM3LastAngle)
//			{
//				if((PM3LastAngle-PM3ThisAngle)>3000)//编码器上溢
//					PM3RealAngle = PM3RealAngle + (PM3ThisAngle+8192-PM3LastAngle) * 360 / 8192.0 / PM3Reduction;
//				else//反转
//					PM3RealAngle = PM3RealAngle - (PM3LastAngle - PM3ThisAngle) * 360 / 8192.0 / PM3Reduction;
//			}
//			else
//			{
//				if((PM3ThisAngle-PM3LastAngle)>3000)//编码器下溢
//					PM3RealAngle = PM3RealAngle - (PM3LastAngle+8192-PM3ThisAngle) *360 / 8192.0 / PM3Reduction;
//				else//正转
//					PM3RealAngle = PM3RealAngle + (PM3ThisAngle - PM3LastAngle) * 360 / 8192.0 / PM3Reduction;
//			}
//			
//			PM3PositionPID.feedback = PM3RealAngle;
//			PM3PositionPID.target = PM3AngleTarget;
//			PM3PositionPID.Calc(&PM3PositionPID);
//			
//			PM3SpeedPID.target = PM3PositionPID.output;
//			PM3SpeedPID.feedback = pData->RotateSpeed;
//			PM3SpeedPID.Calc(&PM3SpeedPID);

//			PM3LastAngle = PM3ThisAngle;
//	
//			setMotor(PM3, PM3SpeedPID.output);
//			
//			s_PM3Count = 0;
//		}
//		else
//		{
//			s_PM3Count++;
//		}
//	}
//}

