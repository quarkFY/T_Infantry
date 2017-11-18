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

//PID_INIT(Kp, Ki, Kd, KpMax, KiMax, KdMax, OutputMax)
//机械臂电机PID

fw_PID_Regulator_t AM1LPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM1RPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM2LPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM2RPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM3LPositionPID = fw_PID_INIT(1.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t AM1LSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AM1RSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AM2LSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AM2RSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);
fw_PID_Regulator_t AM3LSpeedPID = fw_PID_INIT(10.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 4000.0);

//待标定
#define AM1L_zero 0
#define AM1R_zero 0
#define AM2L_zero 0
#define AM2R_zero 0
#define AM3L_zero 0

float AM1LAngleTarget = 0.0;
float AM1RAngleTarget = 0.0;
float AM2LAngleTarget = 0.0;
float AM2RAngleTarget = 0.0;
float AM3LAngleTarget = 0.0;

float AM1LRealAngle = 0.0;
float AM1RRealAngle = 0.0;
float AM2LRealAngle = 0.0;
float AM2RRealAngle = 0.0;
float AM3LRealAngle = 0.0;

uint16_t AM1LRawAngle = 0;
uint16_t AM1RRawAngle = 0;
uint16_t AM2LRawAngle = 0;
uint16_t AM2RRawAngle = 0;
uint16_t AM3LRawAngle = 0;

static uint8_t s_AM1LCount = 0;
static uint8_t s_AM1RCount = 0;
static uint8_t s_AM2LCount = 0;
static uint8_t s_AM2RCount = 0;
static uint8_t s_AM3LCount = 0;

void Can2ControlTask(void const * argument)
{
	while(1)
	{
		osSemaphoreWait(Can2RefreshSemaphoreHandle, osWaitForever);
		
		ControlAM1L();
		ControlAM1R();
		ControlAM2L();
		ControlAM2R();
		ControlAM3L();
	}
}

void ControlAM1L()
{
	if(IOPool_hasNextRead(AM1LRxIOPool, 0))
	{
		if(s_AM1LCount == 1)
		{
			uint16_t AM1LZeroAngle = AM1L_zero;
			
			IOPool_getNextRead(AM1LRxIOPool, 0);
			AM1LRawAngle = IOPool_pGetReadData(AM1LRxIOPool, 0)->angle;
			if(AM1LRawAngle<AM1LZeroAngle) AM1LRawAngle += 8192;
			AM1LRealAngle = (AM1LRawAngle - AM1LZeroAngle) * 360 / 8192.0;
				
			AM1LPositionPID.target = AM1LAngleTarget;
			AM1LPositionPID.feedback = AM1LRealAngle;
			AM1LPositionPID.Calc(&AM1LPositionPID);
			
			AM1LSpeedPID.target = AM1LPositionPID.output;
			AM1LSpeedPID.feedback = IOPool_pGetReadData(AM1LRxIOPool, 0)->RotateSpeed;
			AM1LSpeedPID.Calc(&AM1LSpeedPID);
			
			setMotor(AM1L, AM1LSpeedPID.output);
			s_AM1LCount = 0;
		}
		else
		{
			s_AM1LCount++;
		}
	}
}

void ControlAM1R()
{
	if(IOPool_hasNextRead(AM1RRxIOPool, 0))
	{
		if(s_AM1RCount == 1)
		{
			uint16_t AM1RZeroAngle = AM1R_zero;
			
			IOPool_getNextRead(AM1RRxIOPool, 0);
			AM1RRawAngle = IOPool_pGetReadData(AM1RRxIOPool, 0)->angle;
			if(AM1RRawAngle<AM1RZeroAngle) AM1RRawAngle += 8192;
			AM1RRealAngle = (AM1RRawAngle - AM1RZeroAngle) * 360 / 8192.0;
				
			AM1RPositionPID.target = AM1RAngleTarget;
			AM1RPositionPID.feedback = AM1RRealAngle;
			AM1RPositionPID.Calc(&AM1RPositionPID);
			
			AM1RSpeedPID.target = AM1RPositionPID.output;
			AM1RSpeedPID.feedback = IOPool_pGetReadData(AM1RRxIOPool, 0)->RotateSpeed;
			AM1RSpeedPID.Calc(&AM1RSpeedPID);
			
			setMotor(AM1R, AM1RSpeedPID.output);
			s_AM1RCount = 0;
		}
		else
		{
			s_AM1RCount++;
		}
	}
}

void ControlAM2L()
{
	if(IOPool_hasNextRead(AM2LRxIOPool, 0))
	{
		if(s_AM2LCount == 1)
		{
			uint16_t AM2LZeroAngle = AM2L_zero;
			
			IOPool_getNextRead(AM2LRxIOPool, 0);
			AM2LRawAngle = IOPool_pGetReadData(AM2LRxIOPool, 0)->angle;
			if(AM2LRawAngle<AM2LZeroAngle) AM2LRawAngle += 8192;
			AM2LRealAngle = (AM2LRawAngle - AM2LZeroAngle) * 360 / 8192.0;
				
			AM2LPositionPID.target = AM2LAngleTarget;
			AM2LPositionPID.feedback = AM2LRealAngle;
			AM2LPositionPID.Calc(&AM2LPositionPID);
			
			AM2LSpeedPID.target = AM2LPositionPID.output;
			AM2LSpeedPID.feedback = IOPool_pGetReadData(AM2LRxIOPool, 0)->RotateSpeed;
			AM2LSpeedPID.Calc(&AM2LSpeedPID);
			
			setMotor(AM2L, AM2LSpeedPID.output);
			s_AM2LCount = 0;
		}
		else
		{
			s_AM2LCount++;
		}
	}
}

void ControlAM2R()
{
	if(IOPool_hasNextRead(AM2RRxIOPool, 0))
	{
		if(s_AM2RCount == 1)
		{
			uint16_t AM2RZeroAngle = AM2R_zero;
			
			IOPool_getNextRead(AM2RRxIOPool, 0);
			AM2RRawAngle = IOPool_pGetReadData(AM2RRxIOPool, 0)->angle;
			if(AM2RRawAngle<AM2RZeroAngle) AM2RRawAngle += 8192;
			AM2RRealAngle = (AM2RRawAngle - AM2RZeroAngle) * 360 / 8192.0;
				
			AM2RPositionPID.target = AM2RAngleTarget;
			AM2RPositionPID.feedback = AM2RRealAngle;
			AM2RPositionPID.Calc(&AM2RPositionPID);
			
			AM2RSpeedPID.target = AM2RPositionPID.output;
			AM2RSpeedPID.feedback = IOPool_pGetReadData(AM2RRxIOPool, 0)->RotateSpeed;
			AM2RSpeedPID.Calc(&AM2RSpeedPID);
			
			setMotor(AM2R, AM2RSpeedPID.output);
			s_AM1RCount = 0;
		}
		else
		{
			s_AM2RCount++;
		}
	}
}

void ControlAM3L()
{
	if(IOPool_hasNextRead(AM3LRxIOPool, 0))
	{
		if(s_AM3LCount == 1)
		{
			uint16_t AM3LZeroAngle = AM3L_zero;
			
			IOPool_getNextRead(AM3LRxIOPool, 0);
			AM3LRawAngle = IOPool_pGetReadData(AM3LRxIOPool, 0)->angle;
			if(AM3LRawAngle<AM3LZeroAngle) AM3LRawAngle += 8192;
			AM3LRealAngle = (AM3LRawAngle - AM3LZeroAngle) * 360 / 8192.0;
				
			AM3LPositionPID.target = AM3LAngleTarget;
			AM3LPositionPID.feedback = AM3LRealAngle;
			AM3LPositionPID.Calc(&AM3LPositionPID);
			
			AM3LSpeedPID.target = AM3LPositionPID.output;
			AM3LSpeedPID.feedback = IOPool_pGetReadData(AM3LRxIOPool, 0)->RotateSpeed;
			AM3LSpeedPID.Calc(&AM3LSpeedPID);
			
			setMotor(AM3L, AM3LSpeedPID.output);
			s_AM3LCount = 0;
		}
		else
		{
			s_AM3LCount++;
		}
	}
}

