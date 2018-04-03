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

fw_PID_Regulator_t AM1LPositionPID = fw_PID_INIT(80.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM1RPositionPID = fw_PID_INIT(100.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 16384.0);
fw_PID_Regulator_t AM2LPositionPID = fw_PID_INIT(80.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM2RPositionPID = fw_PID_INIT(80.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM3RPositionPID = fw_PID_INIT(80.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM1LSpeedPID = fw_PID_INIT(2.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 16384.0);
fw_PID_Regulator_t AM1RSpeedPID = fw_PID_INIT(5.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 16384.0);
fw_PID_Regulator_t AM2LSpeedPID = fw_PID_INIT(2.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t AM2RSpeedPID = fw_PID_INIT(2.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 6000.0);
fw_PID_Regulator_t AM3RSpeedPID = fw_PID_INIT(2.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 6000.0);

////待标定
//#define AM1L_zero 0
//#define AM1R_zero 0
//#define AM2L_zero 0
//#define AM2R_zero 0
//#define AM3R_zero 0
extern float PM1AngleTarget;
extern Emergency_Flag emergency_Flag;

#define LengthOfArm1 500
#define LengthOfArm2 250
#define PI 3.141592653589

//机械臂电机目标物理角度值
float AM1LAngleTarget = 0.0;
float AM1RAngleTarget = 0.0;
float AM2LAngleTarget = 0.0;
float AM2RAngleTarget = 0.0;
float AM3RAngleTarget = 0.0;

double Last_Arm_Horizontal_Position;
double Last_Arm_Vertical_Position;

float LastAM1LAngleTarget;
float LastAM1RAngleTarget;
float LastAM2LAngleTarget;
float LastAM2RAngleTarget;
float LastAM3RAngleTarget;

//机械臂电机实际物理角度值
float AM1LRealAngle = 0.0;
float AM1RRealAngle = 0.0;
float AM2LRealAngle = 0.0;
float AM2RRealAngle = 0.0;
float AM3RRealAngle = 0.0;

//末端水平垂直位置
double Arm_Horizontal_Position = LengthOfArm1 - LengthOfArm2;
double Arm_Vertical_Position = 0.0;
double SquareOfRadius = 250.0*250.0;
double AngleOfTarget = 0.0;

//水平垂直运动目标解算值
//double AM1R_AddUpAngle = 0;
//double AM1L_AddUpAngle = 0;
//double AM2R_AddUpAngle = 0;
//double AM2L_AddUpAngle = 0;
//double AM3R_AddUpAngle = 0;
	
extern ArmSpeed_Ref_t ArmSpeedRef;

//uint16_t AM1LRawAngle = 0;
//uint16_t AM1RRawAngle = 0;
//uint16_t AM2LRawAngle = 0;
//uint16_t AM2RRawAngle = 0;
//uint16_t AM3RRawAngle = 0;


//用于减小系统开销
static uint8_t s_AM1LCount = 0;
static uint8_t s_AM1RCount = 0;
static uint8_t s_AM2LCount = 0;
static uint8_t s_AM2RCount = 0;
static uint8_t s_AM3RCount = 0;

void Can2ControlTask(void const * argument)
{
	while(1)
	{
		osSemaphoreWait(Can2RefreshSemaphoreHandle, osWaitForever);
		
		ControlAM1L();
		ControlAM1R();
		ControlAM2L();
		ControlAM2R();
		ControlAM3R();
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

uint8_t isAM2LFirstEnter = 1;
uint16_t AM2LThisAngle = 0;
uint16_t AM2LLastAngle = 0;
void ControlAM2L()
{
	if(IOPool_hasNextRead(AM2LRxIOPool, 0))
	{
		if(s_AM2LCount == 1)
		{
			IOPool_getNextRead(AM2LRxIOPool, 0);
			AM2LThisAngle = IOPool_pGetReadData(AM2LRxIOPool, 0)->angle;
			
			if(isAM2LFirstEnter==1) {AM2LLastAngle = AM2LThisAngle;isAM2LFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM2LThisAngle<=AM2LLastAngle)
			{
				if((AM2LLastAngle-AM2LThisAngle)>3000)//编码器上溢
					AM2LRealAngle = AM2LRealAngle + (AM2LThisAngle+8192-AM2LLastAngle) * 360 / 8192.0 / AM23Reduction;
				else//反转
					AM2LRealAngle = AM2LRealAngle - (AM2LLastAngle - AM2LThisAngle) * 360 / 8192.0 / AM23Reduction;
			}
			else
			{
				if((AM2LThisAngle-AM2LLastAngle)>3000)//编码器下溢
					AM2LRealAngle = AM2LRealAngle - (AM2LLastAngle+8192-AM2LThisAngle) *360 / 8192.0 / AM23Reduction;
				else//正转
					AM2LRealAngle = AM2LRealAngle + (AM2LThisAngle - AM2LLastAngle) * 360 / 8192.0 / AM23Reduction;
			}
			
			
			AM2LPositionPID.target = AM2LAngleTarget;
			AM2LPositionPID.feedback = AM2LRealAngle;
			AM2LPositionPID.Calc(&AM2LPositionPID);
			
			AM2LSpeedPID.target = AM2LPositionPID.output;
			AM2LSpeedPID.feedback = IOPool_pGetReadData(AM2LRxIOPool, 0)->RotateSpeed;
			AM2LSpeedPID.Calc(&AM2LSpeedPID);
			
			setMotor(AM2L, AM2LSpeedPID.output);
			s_AM2LCount = 0;
			AM2LLastAngle = AM2LThisAngle;
		}
		else
		{
			s_AM2LCount++;
		}
	}
}

uint8_t isAM2RFirstEnter = 1;
uint16_t AM2RThisAngle = 0;
uint16_t AM2RLastAngle = 0;
void ControlAM2R()
{
	if(IOPool_hasNextRead(AM2RRxIOPool, 0))
	{
		if(s_AM2RCount == 1)
		{
			IOPool_getNextRead(AM2RRxIOPool, 0);
			AM2RThisAngle = IOPool_pGetReadData(AM2RRxIOPool, 0)->angle;
			
			if(isAM2RFirstEnter==1) {AM2RLastAngle = AM2RThisAngle;isAM2RFirstEnter = 0;return;}	//初始化时，记录下当前编码器的值
			
			if(AM2RThisAngle<=AM2RLastAngle)
			{
				if((AM2RLastAngle-AM2RThisAngle)>3000)//编码器上溢
					AM2RRealAngle = AM2RRealAngle + (AM2RThisAngle+8192-AM2RLastAngle) * 360 / 8192.0 / AM23Reduction;
				else//反转
					AM2RRealAngle = AM2RRealAngle - (AM2RLastAngle - AM2RThisAngle) * 360 / 8192.0 / AM23Reduction;
			}
			else
			{
				if((AM2RThisAngle-AM2RLastAngle)>3000)//编码器下溢
					AM2RRealAngle = AM2RRealAngle - (AM2RLastAngle+8192-AM2RThisAngle) *360 / 8192.0 / AM23Reduction;
				else//正转
					AM2RRealAngle = AM2RRealAngle + (AM2RThisAngle - AM2RLastAngle) * 360 / 8192.0 / AM23Reduction;
			}
			
			
			AM2RPositionPID.target = AM2RAngleTarget;
			AM2RPositionPID.feedback = AM2RRealAngle;
			AM2RPositionPID.Calc(&AM2RPositionPID);
			
			AM2RSpeedPID.target = AM2RPositionPID.output;
			AM2RSpeedPID.feedback = IOPool_pGetReadData(AM2RRxIOPool, 0)->RotateSpeed;
			AM2RSpeedPID.Calc(&AM2RSpeedPID);
			
			setMotor(AM2R, AM2RSpeedPID.output);
			s_AM1RCount = 0;
			AM2RLastAngle = AM2RThisAngle;
		}
		else
		{
			s_AM2RCount++;
		}
	}
}

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
				if((AM3RLastAngle-AM3RThisAngle)>3000)//编码器上溢
					AM3RRealAngle = AM3RRealAngle + (AM3RThisAngle+8192-AM3RLastAngle) * 360 / 8192.0 / AM23Reduction;
				else//反转
					AM3RRealAngle = AM3RRealAngle - (AM3RLastAngle - AM3RThisAngle) * 360 / 8192.0 / AM23Reduction;
			}
			else
			{
				if((AM3RThisAngle-AM3RLastAngle)>3000)//编码器下溢
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


//void setAMAngle(MotorId id, float angle)
//{
//	switch(id){
//		case AM1L:
//			AM1LAngleTarget = angle;break;
//		case AM1R:
//			AM1RAngleTarget = angle;break;
//		case AM2L:
//			AM2LAngleTarget = angle;break;
//		case AM2R:
//			AM2RAngleTarget = angle;break;
//		case AM3R:
//			AM3RAngleTarget = angle;break;
//		default:
//			fw_Error_Handler();
//	}
//}

//一整套动作为下达取弹指令后，展开机械臂，进行取弹，取弹完毕后下达收回指令，机械臂收回
//void getGolf()
//{
	//待完善
	//思路：
	//目前两个电机的task都是用于跟踪的，即令反馈值跟踪上目标值
	//在遥控器任务和2ms定时任务中对目标值进行修改，从而做出指定的动作
	//取弹flag置位，当2ms任务检测到置位后，计数器开始计数，根据计数器的值来进行相应的取弹动作
//}

void armReset()
{
	//待完善
	//思路：
	//取弹flag清零，回收flag置位，具体动作由2ms定时器任务完成，完成后flag清零
//	AM1RAngleTarget = 0;
//	AM2RAngleTarget = 0;
//	LastAM1RAngleTarget = 0;
//	LastAM2RAngleTarget = 0;
//	AM1LAngleTarget = 0;
//	AM2LAngleTarget = 0;
//	LastAM1LAngleTarget = 0;
//	LastAM2LAngleTarget = 0;
//	AM3RAngleTarget = 0;
//	LastAM3RAngleTarget = 0;
//	Arm_Horizontal_Position = 250;
//	Arm_Vertical_Position = 0;
	
}

void ARM_INIT()
{
	
	Arm_Horizontal_Position = 500;
	Arm_Vertical_Position = 250;
//	AM2RAngleTarget = 10;
}

void armStretch()
{
	Arm_Horizontal_Position -= ArmSpeedRef.forward_back_ref;
	Arm_Vertical_Position += ArmSpeedRef.up_down_ref;
	SquareOfRadius = Arm_Horizontal_Position*Arm_Horizontal_Position + Arm_Vertical_Position*Arm_Vertical_Position;
	if(SquareOfRadius <= 250*250 ||  Arm_Vertical_Position< 0 || SquareOfRadius >= 750*750 )
	{
		Arm_Horizontal_Position = Last_Arm_Horizontal_Position;
	  Arm_Vertical_Position = Last_Arm_Vertical_Position;
	}
	
		else
		{
			//AM1R_AddUpAngle = asin((SquareOfRadius+LengthOfArm1*LengthOfArm1-LengthOfArm2*LengthOfArm2)/(2*LengthOfArm1*sqrt(SquareOfRadius)))-acos(Arm_Vertical_Position/sqrt(SquareOfRadius))
			//AM2R_AddUpAngle = asin((SquareOfRadius+LengthOfArm2*LengthOfArm2-LengthOfArm1*LengthOfArm1)/(2*LengthOfArm2*sqrt(SquareOfRadius)))+acos(Arm_Vertical_Position/sqrt(SquareOfRadius))+AM1R_AddUpAngle
			//AM1RAngleTarget 0-180 ; AM2RAngleTarget 0-180 ;

			
			if(Arm_Horizontal_Position > 0 )
			{
				AngleOfTarget = 180*atan(Arm_Vertical_Position/Arm_Horizontal_Position)/PI;
				AM1RAngleTarget = AngleOfTarget - 180*acos((SquareOfRadius+187500)/(1000*sqrt(SquareOfRadius)))/PI;
				AM2RAngleTarget = -(180*acos((312500-SquareOfRadius)/250000)/PI-90);
				AM1LAngleTarget = -AngleOfTarget + 180*acos((SquareOfRadius+187500)/(1000*sqrt(SquareOfRadius)))/PI;
				AM2LAngleTarget = 180*acos((312500-SquareOfRadius)/250000)/PI-90;
			}
			else if(Arm_Horizontal_Position == 0)
			{
				AngleOfTarget = 90.0;
				AM1RAngleTarget = AngleOfTarget - 180*acos((SquareOfRadius+187500)/(1000*sqrt(SquareOfRadius)))/PI;
				AM2RAngleTarget = -(180*acos((312500-SquareOfRadius)/250000)/PI-90);
				AM1LAngleTarget = -AngleOfTarget + 180*acos((SquareOfRadius+187500)/(1000*sqrt(SquareOfRadius)))/PI;
				AM2LAngleTarget = 180*acos((312500-SquareOfRadius)/250000)/PI-90;
			}
			else
			{
				AngleOfTarget = 180*atan(Arm_Vertical_Position/Arm_Horizontal_Position)/PI + 180.0;
				AM1RAngleTarget = AngleOfTarget - 180*acos((SquareOfRadius+187500)/(1000*sqrt(SquareOfRadius)))/PI;
				AM2RAngleTarget = -(180*acos((312500-SquareOfRadius)/250000)/PI-90);
				AM1LAngleTarget = -AngleOfTarget + 180*acos((SquareOfRadius+187500)/(1000*sqrt(SquareOfRadius)))/PI;
				AM2LAngleTarget = 180*acos((312500-SquareOfRadius)/250000)/PI-90;
			}
			
		}
		
		
			Last_Arm_Horizontal_Position = Arm_Horizontal_Position;
			Last_Arm_Vertical_Position = Arm_Vertical_Position;
		
	    LastAM1LAngleTarget = AM1LAngleTarget;
      LastAM1RAngleTarget = AM1RAngleTarget;
      LastAM2LAngleTarget = AM2LAngleTarget;
      LastAM2RAngleTarget = AM2RAngleTarget;
      LastAM3RAngleTarget = AM3RAngleTarget;
}

//void GripLoadProcess()
//{
//  float final;
//	final = AM2RAngleTarget - AM1RAngleTarget + 180;
//	Hero_Angle_Track( final,AM3RRealAngle,&AM3RAngleTarget,&AM3Rtime_milis);
//	if(Hero_Angle_Track( final,AM3RRealAngle,&AM3RAngleTarget,&AM3Rtime_milis))
//	{
//		GRIP_SOV_ON();
//	}
	
//}

//角度跟踪精确控制，分段，精确到1°
//uint8_t Hero_Angle_Track(float final,float currentAngle,float *angleTarget,uint8_t *time_milis)
//{
//	uint8_t motorReady = 1;
//	float tmp = (final - currentAngle)/(*time_milis);
//	while(time_milis>0 && motorReady)
//	{
//		if(emergency_Flag==EMERGENCY)
//		{	
//			return 0;
//		}
//		if(-1<( *angleTarget - currentAngle ) < 1)
//		{
//		  *angleTarget += tmp;
//		  time_milis--;
//			motorReady = 1;
//		}
//		else motorReady = 0;
//		osDelay(1);
//	}
//	if(-1< (final - currentAngle ) < 1) return 1;
//	else return 0;
//}

//uint8_t taskDelay(uint32_t time_milis)
//{
//	for(int i=0;i<time_milis;i++)
//	{
//		if(emergency_Flag==EMERGENCY)
//		{	
//			return 0;
//		}
//		osDelay(1);
//	}
//	return 1;
//}
