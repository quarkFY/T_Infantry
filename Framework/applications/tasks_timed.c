/**
  ******************************************************************************
  * File Name          : tasks_cmcontrol.c
  * Description        : 2ms定时任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
	* 2ms定时
	* 通过count可以获得500ms,1s等定时任务
	* 状态机切换，串口定时输出，看门狗等
  ******************************************************************************
  */
#include <tim.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <cmsis_os.h>
#include <task.h>
#include <usart.h>
#include "tasks_timed.h"
#include "pid_Regulator.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "tasks_remotecontrol.h"
#include "tasks_platemotor.h"
#include "application_motorcontrol.h"
#include "drivers_canmotor_low.h"
#include "drivers_canmotor_user.h"
#include "utilities_debug.h"
#include "rtos_semaphore.h"
#include "rtos_task.h"
#include "peripheral_define.h"
#include "drivers_platemotor.h"
#include "application_waveform.h"
#include "drivers_uartjudge_low.h"
#include "drivers_uartupper_user.h"
#include "utilities_minmax.h"
#include "drivers_ramp.h"
#include "peripheral_laser.h"
#include "drivers_uartrc_low.h"
#include "tasks_motor.h"//zy
#include "tasks_can2motor.h"
#include <stdbool.h>
#include "visualscope.h"

extern PID_Regulator_t CMRotatePID ; 
extern PID_Regulator_t CM1SpeedPID;
extern PID_Regulator_t CM2SpeedPID;
extern PID_Regulator_t CM3SpeedPID;
extern PID_Regulator_t CM4SpeedPID;
void CMControlInit(void)
{
//底盘电机PID初始化，copy from官方开源程序
	//ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}


///////////////////表示状态的变量/////////////////////
Shoot_State_e last_shoot_state = NO_SHOOT;
Shoot_State_e this_shoot_state = NO_SHOOT;
//uint32_t last_Encoder = 0;
//uint32_t this_Encoder = 0;

extern uint8_t JUDGE_STATE;
extern uint8_t JUDGE_Received;
extern uint8_t JUDGE_State;
extern Emergency_Flag emergency_Flag;

WorkState_e g_workState = PREPARE_STATE;
WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e GetWorkState()
{
	return g_workState;
}


/*2ms定时任务*/
extern float yawAngleTarget;
extern float pitchAngleTarget;
extern RampGen_t frictionRamp ;

static uint32_t s_time_tick_2ms = 0;
uint16_t checkRecTime=300;
uint16_t checkKeyTime=500;


void Timer_2ms_lTask(void const * argument)
{
	//RTOS提供，用来做2ms精确定时
	//与后面的vTaskDelayUntil()配合使用
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	//countwhile来获得不同定时任务
	static int s_countWhile = 0;

	
//static int shootwhile = 0;
//unsigned portBASE_TYPE StackResidue; //栈剩余
	while(1)  
	{       
		WorkStateFSM();//状态机
	  WorkStateSwitchProcess();//状态机动作
		
		getJudgeState();
		
		if(checkRecTime<65534)
		{
			checkRecTime++;
		}
		if(checkKeyTime<65534)
		{
			checkKeyTime++;
		}


//			VisualScope(&huart3, (int)PM1RealAngle, 0, 0, 0); 
		
		
//定时1s,发送调试信息		
		if(s_countWhile >= 2000)//150 1000
		{
			s_countWhile = 0;

			/*
			*****查看任务栈空间剩余示例*******
			//		StackResidue = uxTaskGetStackHighWaterMark( GMControlTaskHandle );
			//		fw_printfln("GM%ld",StackResidue);*/
			if(JUDGE_State == OFFLINE)
			{
				//fw_printfln("Judge not received");
			}
		}
		else
		{
			s_countWhile++;
		}
		
		vTaskDelayUntil( &xLastWakeTime, ( 2 / portTICK_RATE_MS ) );//这里进入阻塞态等待2ms
	}
}
	


/**********************************************************
*工作状态切换状态机
**********************************************************/

extern RemoteSwitch_t g_switch1; 
extern RC_Ctl_t RC_CtrlData; 
extern bool g_switchRead;


void WorkStateFSM(void)
{
	lastWorkState = g_workState;
	s_time_tick_2ms ++;
	
	switch(g_workState)
	{
		case PREPARE_STATE:
		{
			if(emergency_Flag == EMERGENCY )
			{
				g_workState = STOP_STATE;
			}
			else if(s_time_tick_2ms > PREPARE_TIME_TICK_MS)
			{
				LASER_ON();
				g_workState = NORMAL_STATE;
			}			
		}break;
		
		case NORMAL_STATE:     
		{
			if(emergency_Flag == EMERGENCY )
			{
				g_workState = STOP_STATE;
			}
		}break;
		
		case STOP_STATE:   
		{
			if(emergency_Flag == NORMAL )
			{
				g_workState = PREPARE_STATE;   
			}
			g_workState = STOP_STATE;
		}break;

		default:
		{
			
		}
	}	
}


extern float gap_angle;
extern float pitchRealAngle;
	
void WorkStateSwitchProcess(void)
{
	if((lastWorkState != g_workState) && (g_workState == STOP_STATE))  
	{
		LASER_OFF();
		SetShootState(NO_SHOOT);
		SetGetBulletState(NO_GETBULLET);
		SetFrictionWheelSpeed(1000);
		SetFrictionState(FRICTION_WHEEL_OFF);
		frictionRamp.ResetCounter(&frictionRamp);
	}
	//如果从其他模式切换到prapare模式，要将一系列参数初始化
	if((lastWorkState != g_workState) && (g_workState == PREPARE_STATE))  
	{
		//计数初始化
	  s_time_tick_2ms = 0;   
		yawAngleTarget = 0;
		pitchAngleTarget = 0;
		CMControlInit();
		RemoteTaskInit();
//		armReset();
	}
}

void getJudgeState(void)
{
	static int s_count_judge = 0;
	if(JUDGE_Received==1)
	{
		s_count_judge = 0;
		JUDGE_State = ONLINE;
		JUDGE_Received = 0;
	}
	else
	{
		s_count_judge++;
		if(s_count_judge > 150)
		{//300ms
			JUDGE_State = OFFLINE;
		}
	}
}

