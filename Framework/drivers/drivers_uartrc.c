/**
  ******************************************************************************
  * File Name          : drivers_uartrc.c
  * Description        : 遥控器串口
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 串口初始化
	* 串口数据读取
	* 数据处理函数
  ******************************************************************************
  */
#include "drivers_uartrc_user.h"
#include "drivers_uartrc_low.h"
#include "drivers_led_user.h"
#include "peripheral_laser.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "rtos_semaphore.h"
#include "drivers_ramp.h"
#include "peripheral_tim.h"
#include <stdlib.h>
#include <math.h>
#include "utilities_debug.h"
#include  "tim.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "peripheral_define.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartupper_user.h"
#include "stm32f4xx_hal_uart.h"
#include "tasks_platemotor.h"
#include "tasks_motor.h"
#include "tasks_arm.h"
#include "tasks_hero.h"
#include "peripheral_sov.h"

NaiveIOPoolDefine(rcUartIOPool, {0});

//遥控器串口初始化，操作系统初始化的时候调用
void InitRemoteControl(){
	//遥控器DMA接收开启(一次接收18个字节)
	if(HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18) != HAL_OK){
			Error_Handler();
	} 
//	__HAL_UART_ENABLE_IT(&RC_UART, UART_FLAG_IDLE);//空闲中断方式更优，未调通
	RemoteTaskInit();
}

void rcUartRxCpltCallback(){
	static portBASE_TYPE xHigherPriorityTaskWoken;
	 xHigherPriorityTaskWoken = pdFALSE; 
	//释放信号量
   xSemaphoreGiveFromISR(xSemaphore_rcuart, &xHigherPriorityTaskWoken);
	
	
	//切换上下文，RTOS提供
	//当在中断外有多个不同优先级任务等待信号量时
	//在退出中断前进行一次切换上下文
	//这里无用
	if( xHigherPriorityTaskWoken == pdTRUE ){
   portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	 }
 }



RC_Ctl_t RC_CtrlData;   //remote control data
ChassisSpeed_Ref_t ChassisSpeedRef; 
ArmSpeed_Ref_t ArmSpeedRef;
Gimbal_Ref_t GimbalRef; 
 
 ////////////////控制用的状态////////////////////
 //摩擦轮状态
FrictionWheelState_e g_friction_wheel_state = FRICTION_WHEEL_OFF; 
//发射状态
volatile Shoot_State_e shootState = NO_SHOOT; 
//控制状态
InputMode_e inputmode = REMOTE_INPUT;  
//取弹状态
Get_Bullet_e GetBulletState = NO_GETBULLET;
//云台底盘锁定状态
GMMode_e GMMode = LOCK;
 //取弹任务状态
 extern HERO_Order_t HERO_Order;
 //底盘状态
 Chassis_Mode_e FrontWheel_Mode = CHASSIS_NORMAL, Last_FrontWheel_Mode = CHASSIS_NORMAL,BehindWheel_Mode = CHASSIS_NORMAL, Last_BehindWheel_Mode = CHASSIS_NORMAL;


unsigned int zyLeftPostion; //大符用左拨杆位置
 
static uint32_t RotateCNT = 0;	//长按连发计数
static uint16_t CNT_1s = 75;	//用于避免四连发模式下两秒内连射8发过于密集的情况
static uint16_t CNT_250ms = 18;	//用于点射模式下射频限制

//左上拨杆用于调试云台，记得要注释掉
extern float yaw_zero, pitch_zero;
extern float yawEncoder, pitchEncoder;
extern float yawAngleTarget, pitchAngleTarget;
extern int isGMSet;

void SetGMZeroPoint(RemoteSwitch_t *sw, uint8_t val) 
{
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   //OFF->HL
	{
//		yaw_zero = yawEncoder;
//		pitch_zero = pitchEncoder;
		isGMSet = 1;
	}
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)	//CL->HL
	{
		pitchAngleTarget = 0;
		yawAngleTarget = 0;
	}
}
 

RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT; 
RampGen_t RotSpeedRamp = RAMP_GEN_DAFAULT;

void RemoteTaskInit()
{
  /*斜坡初始化，copy from官方程序，实现被封装在RMLib*/
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	RotSpeedRamp.SetScale(&RotSpeedRamp, MOUSR_ROT_RAMP_TICK_COUNT);
	frictionRamp.ResetCounter(&frictionRamp);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	RotSpeedRamp.ResetCounter(&RotSpeedRamp);
  /*底盘速度初始化*/
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
  /*摩擦轮*/
	SetFrictionWheelSpeed(800); 
	SetFrictionState(FRICTION_WHEEL_OFF);
	ArmSpeedRef.forward_back_ref = 0.0f;
	ArmSpeedRef.up_down_ref = 0.0f;
}

/*拨杆数据处理*/   
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	//value1 value2的值其实是一样的
	//value1高4位始终为0
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);

	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	//如果两次数据一样，即没有更新数据，拨杆不动
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}
	//如果拨杆维持了一定时间，即连续来了40帧一样的数据，则把拨杆数据写入switch_long_value
	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}
	//指向下一个缓冲区
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}
//return the state of the remote 0:no action 1:action 
uint8_t IsRemoteBeingAction(void)
{
	return (abs(ChassisSpeedRef.forward_back_ref)>=10 || abs(ChassisSpeedRef.left_right_ref)>=10 || fabs(GimbalRef.yaw_speed_ref)>=10 || fabs(GimbalRef.pitch_speed_ref)>=10);
}
/*取得右上角拨杆数据*/
void SetInputMode(Remote *rc)
{
	if(rc->s2 == 1)
	{
		inputmode = REMOTE_INPUT;
	}
	else if(rc->s2 == 3)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		inputmode = GETBULLET_INPUT;
	}	
}



InputMode_e GetInputMode()
{
	return inputmode;
}

/*
input: RemoteSwitch_t *sw, include the switch info
*/
extern PID_Regulator_t PM1PositionPID;
extern PID_Regulator_t PM2PositionPID;
uint16_t remoteShootDelay = 500;
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	switch(g_friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   
			{
				SetShootState(NO_SHOOT);
				frictionRamp.ResetCounter(&frictionRamp);
				g_friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
				FRONT_SOV1_OFF();
			}
//			else if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)		//收回取弹机械臂
//			{
//				armReset();
//				
//			}
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				LASER_OFF();//zy0802
				SetShootState(NO_SHOOT);
				SetFrictionWheelSpeed(800);
				g_friction_wheel_state = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			
	//		else if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)	
	//		{
//				LASER_OFF();
//				SetShootState(NO_SHOOT);
//				SetFrictionWheelSpeed(1000);
//				g_friction_wheel_state = FRICTION_WHEEL_OFF;
//				frictionRamp.ResetCounter(&frictionRamp);
				
	//		}
			
			else
			{
				/*斜坡函数必须有，避免电流过大烧坏主控板*/
				SetFrictionWheelSpeed(800 + (FRICTION_WHEEL_MAX_DUTY-800)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					g_friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				LASER_OFF();//zy0802
				g_friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(800); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NO_SHOOT);
			}
			else if(sw->switch_value_raw == 2)	//左侧拨杆拨到中间便会开枪
			{
				SetShootState(MANUL_SHOOT_ONE);
				if(remoteShootDelay!=0) 
					--remoteShootDelay;
				else
				{
					shootOneGolf();
					remoteShootDelay = 50;
				}
			}
			else
			{
				SetShootState(NO_SHOOT);
			}					 
		} break;				
	}
}
	 
void MouseShootControl(Mouse *mouse)
{
	++CNT_1s;
	++CNT_250ms;
	static int16_t closeDelayCount = 0;   
	switch(g_friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(mouse->last_press_r == 0 && mouse->press_r == 1)   
			{
				SetShootState(NO_SHOOT);
				frictionRamp.ResetCounter(&frictionRamp);
				g_friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
				closeDelayCount = 0;
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			if(closeDelayCount>50)   
			{
				LASER_OFF();//zy0802
				g_friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(800); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NO_SHOOT);
			}
			else
			{
		    /*摩擦轮转速修改 FRICTION_WHEEL_MAX_DUTY*/
				SetFrictionWheelSpeed(800 + (FRICTION_WHEEL_MAX_DUTY-800)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					g_friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(mouse->press_r == 1)
			{
				closeDelayCount++;
			}
			else
			{
				closeDelayCount = 0;
			}
			if(closeDelayCount>50)   //
			{
				LASER_OFF();//zy0802
				g_friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(800); 
				frictionRamp.ResetCounter(&frictionRamp);
				SetShootState(NO_SHOOT);
			}			
			else if(mouse->last_press_l == 0 && mouse->press_l== 1)  //检测鼠标左键单击动作
			{
				SetShootState(MANUL_SHOOT_ONE);
				if(getLaunchMode() == SINGLE_MULTI && GetFrictionState()==FRICTION_WHEEL_ON)		//单发模式下，点一下打一发
				{
					if(CNT_250ms>17)
					{
						CNT_250ms = 0;
						shootOneGolf();
						
					}
				}
				else if(getLaunchMode() == CONSTENT_4 && GetFrictionState()==FRICTION_WHEEL_ON)	//四连发模式下，点一下打四发
				{
					
					if(CNT_1s>75)
					{
						CNT_1s = 0;
						shootOneGolf();
						shootOneGolf();
						shootOneGolf();
						shootOneGolf();
					}
				}
			}
			else if(mouse->last_press_l == 0 && mouse->press_l== 0)	//松开鼠标左键的状态
			{
				SetShootState(NO_SHOOT);	
				RotateCNT = 0;			
			}			
			else if(mouse->last_press_l == 1 && mouse->press_l== 1 && getLaunchMode() == SINGLE_MULTI)//单发模式下长按，便持续连发
			{
				RotateCNT+=50;
				if(RotateCNT>=OneShoot)
				{
					shootOneGolf();
					RotateCNT = 0;
				}
					
			}
				
		} break;				
	}	
	mouse->last_press_r = mouse->press_r;
	mouse->last_press_l = mouse->press_l;
}




Shoot_State_e GetShootState()
{
	return shootState;
}

void SetShootState(Shoot_State_e v)
{
	shootState = v;
}

FrictionWheelState_e GetFrictionState()
{
	return g_friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
	g_friction_wheel_state = v;
}
void SetFrictionWheelSpeed(uint16_t x)
{
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_1, x);
	__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_2, x);
}


Emergency_Flag emergency_Flag = NORMAL;

Emergency_Flag GetEmergencyFlag()
{
	return emergency_Flag;
}

void SetEmergencyFlag(Emergency_Flag v)
{
	emergency_Flag = v;
}

Move_Speed_e movespeed = NORMAL_s;

Move_Speed_e GetMoveSpeed()
{
	return movespeed;
}

void SetMoveSpeed(Move_Speed_e v)
{
	movespeed = v;
}


/*
Slab_Mode_e slabmode = CLOSE;
Slab_Mode_e GetSlabState()
{
	return slabmode;
}

void SetSlabState(Slab_Mode_e v)
{
	slabmode = v;
}*/

void RemoteGetBulletControl(RemoteSwitch_t *sw, uint8_t val)
{
	if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   
	{
		ARM_INIT();
		SetGetBulletState(MANUL_GETBULLET);
		HERO_Order=HERO_MANUL_FETCH;
	}
	else if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)
	{
		SetGetBulletState(NO_GETBULLET);
		HERO_Order=HERO_STANDBY;
	}
	else if(sw->switch_value_raw == 1)
	{
		SetGetBulletState(NO_GETBULLET);
		HERO_Order=HERO_STANDBY;
	}
	else if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)
	{
		SetGetBulletState(AUTO_GETBULLET);
	//	HERO_Order=HERO_MANUL_LOAD;使用遥控器调试用
	}
	else if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_2TO3)
	{
		SetGetBulletState(MANUL_GETBULLET);
	//	HERO_Order=HERO_MANUL_DISCARD;
	}
	

}

Get_Bullet_e GetGetBulletState()
{
	return GetBulletState;
}


void SetGetBulletState(Get_Bullet_e v)
{
	GetBulletState = v;
}

//底盘升降控制
void RaiseControlProcess()
{
	//HIGH TO LOW|LOW TO HIGH可能需要一定延时
	if(Last_FrontWheel_Mode == CHASSIS_NORMAL && FrontWheel_Mode == CHASSIS_HIGH)
	{
		FRONT_SOV1_ON();
	}
	else if(Last_FrontWheel_Mode == CHASSIS_NORMAL && FrontWheel_Mode == CHASSIS_LOW)
	{
		FRONT_SOV2_ON();
	}
	else if(Last_FrontWheel_Mode == CHASSIS_HIGH && FrontWheel_Mode == CHASSIS_NORMAL)
	{
		FRONT_SOV1_OFF();
	}
	else if(Last_FrontWheel_Mode == CHASSIS_HIGH && FrontWheel_Mode == CHASSIS_LOW)
	{
		FRONT_SOV1_OFF();
		FRONT_SOV2_ON();
	}
	else if(Last_FrontWheel_Mode == CHASSIS_LOW && FrontWheel_Mode == CHASSIS_NORMAL)
	{
		FRONT_SOV2_OFF();
	}
	else if(Last_FrontWheel_Mode == CHASSIS_LOW && FrontWheel_Mode == CHASSIS_HIGH)
	{
		FRONT_SOV2_OFF();
		FRONT_SOV1_ON();
	}
	Last_FrontWheel_Mode = FrontWheel_Mode;
	
	if(Last_BehindWheel_Mode == CHASSIS_NORMAL && BehindWheel_Mode == CHASSIS_HIGH)
	{
		BEHIND_SOV1_ON();
	}
	else if(Last_BehindWheel_Mode == CHASSIS_NORMAL && BehindWheel_Mode == CHASSIS_LOW)
	{
		BEHIND_SOV2_ON();
	}
	else if(Last_BehindWheel_Mode == CHASSIS_HIGH && BehindWheel_Mode == CHASSIS_NORMAL)
	{
		BEHIND_SOV1_OFF();
	}
	else if(Last_BehindWheel_Mode == CHASSIS_HIGH && BehindWheel_Mode == CHASSIS_LOW)
	{
		BEHIND_SOV1_OFF();
		BEHIND_SOV2_ON();
	}
	else if(Last_BehindWheel_Mode == CHASSIS_LOW && BehindWheel_Mode == CHASSIS_NORMAL)
	{
		BEHIND_SOV2_OFF();
	}
	else if(Last_BehindWheel_Mode == CHASSIS_LOW && BehindWheel_Mode == CHASSIS_HIGH)
	{
		BEHIND_SOV2_OFF();
		BEHIND_SOV1_ON();
	}
	Last_BehindWheel_Mode = BehindWheel_Mode;
}

