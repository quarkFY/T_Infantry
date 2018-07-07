/**
  ******************************************************************************
  * File Name          : tasks_remotecontrol.c
  * Description        : 遥控器处理任务
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "tasks_remotecontrol.h"
#include "drivers_uartrc_user.h"
#include "drivers_uartrc_low.h"
#include "utilities_debug.h"
#include "stdint.h"
#include "stddef.h"
#include "drivers_ramp.h"
#include "pid_regulator.h"
#include "tasks_timed.h"
#include "usart.h"
#include "peripheral_define.h"
#include "pwm_server_motor.h"
#include "drivers_uartjudge_low.h"
#include "tasks_motor.h"
#include "iwdg.h"
//**//
#include "utilities_minmax.h"
#include "math.h"
#include <stdlib.h>
#include <stdbool.h>
#include "tasks_platemotor.h"
#include "drivers_uartupper_user.h"
#include "tasks_arm.h"
#include "peripheral_laser.h"
#include "peripheral_sov.h"
#include "tasks_hero.h"

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

//uint8_t going = 0;
extern ChassisSpeed_Ref_t ChassisSpeedRef;
extern ArmSpeed_Ref_t ArmSpeedRef;
extern Gimbal_Ref_t GimbalRef;
extern FrictionWheelState_e g_friction_wheel_state ;
extern GMMode_e GMMode;
extern Chassis_Mode_e FrontWheel_Mode , Last_FrontWheel_Mode ,BehindWheel_Mode , Last_BehindWheel_Mode ;

extern float AM1RAngleTarget,AM1LAngleTarget,AM3RAngleTarget;
extern float AM1RRealAngle,AM1LRealAngle,AM3RRealAngle;

extern float CMFRAngleTarget,CMFLAngleTarget,CMBRAngleTarget,CMBLAngleTarget;
extern float CMFRRealAngle,CMFLRealAngle,CMBRRealAngle,CMBLRealAngle;

extern uint16_t load_cnt;
extern int ad1,ad2,ad3,ad4,ad5;

RemoteSwitch_t g_switch1;   

extern RampGen_t frictionRamp ;  //摩擦轮斜坡
extern RampGen_t LRSpeedRamp ;   //键盘速度斜坡
extern RampGen_t FBSpeedRamp  ; 
extern RampGen_t RotSpeedRamp ;

extern RC_Ctl_t RC_CtrlData; 
extern xSemaphoreHandle xSemaphore_rcuart;
extern float yawAngleTarget, pitchAngleTarget;
extern float rotateSpeed;
extern uint8_t g_isGYRO_Rested ;

extern WorkState_e g_workState;//张雁大符
extern InputMode_e inputmode;
extern Get_Bullet_e GetBulletState;

extern int twist_state ;//扭腰

void RControlTask(void const * argument){
	uint8_t data[18];
	static TickType_t lastcount_rc;
	static TickType_t thiscount_rc;
	static uint8_t first_frame = 0;
	while(1){
		if(first_frame == 0)
		{
			MX_IWDG_Init();
		}
		//一旦遥控信号中断，此进程就会被一直阻塞，就会引起看门狗复位
		HAL_IWDG_Refresh(&hiwdg);
		/*等待串口接收中断回调函数释放信号量*/
		xSemaphoreTake(xSemaphore_rcuart, osWaitForever);
		//fw_printfln("RC is running");
		/*获取两帧时间间隔，正常14ms，大于16ms认为错误*/
		thiscount_rc = xTaskGetTickCount();

		if( ((thiscount_rc - lastcount_rc) <= 16) && (first_frame == 1))//第一帧认为错误
		{
			/*从IOPool读数据到数组*/
			IOPool_getNextWrite(rcUartIOPool);
			if(IOPool_hasNextRead(rcUartIOPool, 0))
			{
				IOPool_getNextRead(rcUartIOPool, 0);
				uint8_t *pData = IOPool_pGetReadData(rcUartIOPool, 0)->ch;
				for(uint8_t i = 0; i != 18; ++i)
				{
					data[i] = pData[i];
				}

				/*处理数据*/
				RemoteDataProcess(data);	//process raw data then execute new order
				/*扔掉多余数据，重新开启接收中断*/
				vTaskDelay(2 / portTICK_RATE_MS);
				HAL_UART_AbortReceive(&RC_UART);
				HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18);
/*
//				if(countwhile >= 300){
//					countwhile = 0;
//			    fw_printf("ch0 = %d | ", RC_CtrlData.rc.ch0);
//				fw_printf("ch1 = %d | ", RC_CtrlData.rc.ch1);
//				fw_printf("ch2 = %d | ", RC_CtrlData.rc.ch2);
//				fw_printf("ch3 = %d \r\n", RC_CtrlData.rc.ch3);
//				
//				fw_printf("s1 = %d | ", RC_CtrlData.rc.s1);
//				fw_printf("s2 = %d \r\n", RC_CtrlData.rc.s2);
//				
//				fw_printf("x = %d | ", RC_CtrlData.mouse.x);
//				fw_printf("y = %d | ", RC_CtrlData.mouse.y);
//				fw_printf("z = %d | ", RC_CtrlData.mouse.z);
//				fw_printf("l = %d | ", RC_CtrlData.mouse.press_l);
//				fw_printf("r = %d \r\n", RC_CtrlData.mouse.press_r);
//				
//				fw_printf("key = %d \r\n", RC_CtrlData.key.v);
//				fw_printf("===========\r\n");
//				}else{
//					countwhile++;
//				}*/
	    }
		}
		else{
			/*错误帧等待2ms后清空缓存，开启中断*/
			//fw_printfln("RC discarded");
			first_frame = 1;
			vTaskDelay(2 / portTICK_RATE_MS);
			HAL_UART_AbortReceive(&RC_UART);
			HAL_UART_Receive_DMA(&RC_UART, IOPool_pGetWriteData(rcUartIOPool)->ch, 18);
		}
		lastcount_rc = thiscount_rc;
	}
}

bool g_switchRead = 0;
//接收机会一次性把遥控器本身、鼠标、键盘的数据全部接收，我们需要根据输入模式来有选择地使用这些数据
void RemoteDataProcess(uint8_t *pData)
{
	if(pData == NULL)
	{
			return;
	}
	//遥控器 11*4 + 2*2 = 48，需要 6 Bytes
	//16位，只看低11位
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
											 ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	//16位，只看最低两位
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

	//鼠标需要 8 Bytes
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	
	//键盘需要 2 Bytes = 16 bits ，每一位对应一个键
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);//16 bits correspond to 16 keys
	
	SetInputMode(&RC_CtrlData.rc);
	
	/*左上角拨杆状态获取*/	//用于遥控器发射控制
	GetRemoteSwitchAction(&g_switch1, RC_CtrlData.rc.s1);
	g_switchRead = 1;
	

	switch(GetInputMode())
	{
		case REMOTE_INPUT:
		{
			if(GetWorkState() == NORMAL_STATE)
			{ 
				RemoteControlProcess(&(RC_CtrlData.rc));//遥控器模式
			}
		}break;
		case KEY_MOUSE_INPUT:
		{
			if(GetWorkState() == NORMAL_STATE)
			{
					MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);//键鼠模式
			}
		}break;
		case GETBULLET_INPUT:
		{
			 GetBulletControlprocess(&(RC_CtrlData.rc),&RC_CtrlData.mouse,&RC_CtrlData.key);//取弹模式
		}break;
	}
}

/////////////////////////遥控器模式//////////////////////////
float forward_kp = 1.0 ;
extern float yawMotorAngle;
void RemoteControlProcess(Remote *rc)
{
	static float AngleTarget_temp = 0;
	if(GetWorkState() == NORMAL_STATE)
	{
		ChassisSpeedRef.forward_back_ref = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 0.4;
		ChassisSpeedRef.left_right_ref   = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 0.4; 
		
 		pitchAngleTarget += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
		//yawAngleTarget   -= (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT; 
		if(fabs(yawMotorAngle) <= 15)
		{
				yawAngleTarget   -= (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;
		}
			
		AngleTarget_temp = yawAngleTarget;
			
		if(fabs(yawMotorAngle) > 15 )
		{
				AngleTarget_temp   -= (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;
				if(fabs(AngleTarget_temp)<fabs(yawAngleTarget))
					yawAngleTarget = AngleTarget_temp;
		}
		//ChassisSpeedRef.rotate_ref   = -(rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) *STICK_TO_CHASSIS_SPEED_REF_FACT*0.2 ;
	 
	}
	 RemoteShootControl(&g_switch1, rc->s1);
}


extern uint8_t JUDGE_State;
uint16_t forward_back_speed = 0;
uint16_t left_right_speed = 0;
uint16_t rotate_speed=0;
uint16_t lastKey;
///////////////////////////键鼠模式//////////////////////////
//调整鼠标灵敏度
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.025f * 2
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.025f * 2

int keyDebug;
uint8_t detect,going;
//遥控器模式下机器人无级变速  键鼠模式下机器人速度为固定档位
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	static float AngleTarget_temp = 0;
	keyDebug = key ->v;
	if(GetWorkState() == NORMAL_STATE)
	{
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 
	
		pitchAngleTarget -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT; 
//		if(key->v == 0x0400) GMMode = LOCK;    //锁定云台  G
//		if(key->v == 0x0420) GMMode = UNLOCK;  //解锁云台  G + Ctrl		
		if(key->v == 0x8000)//b
		{
			HERO_Order = HERO_SHOOT_LOAD;
		}				
//		if(GMMode == UNLOCK) 
//		{
//			yawAngleTarget    -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;
//		}
		if(fabs(yawMotorAngle) <= 15)
		{
				yawAngleTarget   -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;
		}
			
		AngleTarget_temp = yawAngleTarget;
			
		if(fabs(yawMotorAngle) > 15 )
		{
				AngleTarget_temp   -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;
				if(fabs(AngleTarget_temp)<fabs(yawAngleTarget))
				yawAngleTarget = AngleTarget_temp;
		}
		
		forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
		left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		rotate_speed = NORMAL_ROTATE_SPEED;
		//speed mode: normal speed/high speed 
		if(key->v & 0x10)//Shift
		{
			forward_back_speed =  LOW_FORWARD_BACK_SPEED;
			left_right_speed = LOW_LEFT_RIGHT_SPEED;
			rotate_speed = LOW_ROTATE_SPEED;
		}
		if(key->v == 0x20)//Ctrl
			
		{	
			forward_back_speed =  LOW_FORWARD_BACK_SPEED;
			left_right_speed = LOW_LEFT_RIGHT_SPEED;
			rotate_speed = LOW_ROTATE_SPEED;
			if(PIR_R_Ready && PIR_L_Ready)
			{
				HERO_Order = HERO_MANUL_PREPARE;
			}
		}
		//movement process
		if(key->v & 0x01)  // key: w
		{
			ChassisSpeedRef.forward_back_ref = forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
			twist_state = 0;
		}
		else if(key->v & 0x02) //key: s
		{
			ChassisSpeedRef.forward_back_ref = -forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
			twist_state = 0;
		}
		else
		{
			ChassisSpeedRef.forward_back_ref = 0;
			FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		}
		if(key->v & 0x04)  // key: d
		{
			ChassisSpeedRef.left_right_ref = -left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
			twist_state = 0;
		}
		else if(key->v & 0x08) //key: a
		{
			ChassisSpeedRef.left_right_ref = left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
			twist_state = 0;
		}
		else
		{
			ChassisSpeedRef.left_right_ref = 0;
			LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		}
		if(key->v & 0x80)	//key:e  检测第8位是不是1
		{
			ChassisSpeedRef.rotate_ref=-rotate_speed*RotSpeedRamp.Calc(&RotSpeedRamp);
		}
		else if(key->v & 0x40)	//key:q  
		{
			ChassisSpeedRef.rotate_ref=rotate_speed*RotSpeedRamp.Calc(&RotSpeedRamp);
		}
		else 
		{
			ChassisSpeedRef.rotate_ref = 0;
			RotSpeedRamp.ResetCounter(&RotSpeedRamp);
		}
		if(lastKey == 0x0000 && key->v & 0x1000)//x
		{
			HERO_Order = HERO_AUTO_GET3BOX;
		}
		if(key->v == 0x2000)//c
		{
			HERO_Order = HERO_AUTO_GETBOX;
		}
		if(key->v & 0x4000)//v
		{
			HERO_Order = HERO_MANUL_FETCH;
		}
		if(key->v == 0x0800)//z lastKey == 0x0000 && 
		{
			HERO_Order = HERO_MANUL_RECOVER;
		}

		if(key->v == 256)  // key: r
		{
			twist_state = 1;
		}
//		else
//		{
//			twist_state = 0;
//		}
		
		//mouse x y control
		if(GMMode == LOCK)
		{
			ChassisSpeedRef.rotate_ref -= mouse->x/27.0*3000;
			yawAngleTarget = -ChassisSpeedRef.rotate_ref * forward_kp / 2000;
		}
		/*裁判系统离线时的功率限制方式*/
		if(JUDGE_State == OFFLINE)
		{
			if(abs(ChassisSpeedRef.forward_back_ref) + abs(ChassisSpeedRef.left_right_ref) > 500)
			{
				if(ChassisSpeedRef.forward_back_ref > 325)
				{
				ChassisSpeedRef.forward_back_ref =  325 +  (ChassisSpeedRef.forward_back_ref - 325) * 0.15f;
				}
				else if(ChassisSpeedRef.forward_back_ref < -325)
				{
				ChassisSpeedRef.forward_back_ref =  -325 +  (ChassisSpeedRef.forward_back_ref + 325) * 0.15f;
				}
				if(ChassisSpeedRef.left_right_ref > 300)
				{
				ChassisSpeedRef.left_right_ref =  300 +  (ChassisSpeedRef.left_right_ref - 300) * 0.15f;
				}
				else if(ChassisSpeedRef.left_right_ref < -300)
				{
				ChassisSpeedRef.left_right_ref =  -300 +  (ChassisSpeedRef.left_right_ref + 300) * 0.15f;
				}
			}

			if ((mouse->x < -2.6) || (mouse->x > 2.6))
			{
				if(abs(ChassisSpeedRef.forward_back_ref) + abs(ChassisSpeedRef.left_right_ref) > 400)
				{
					if(ChassisSpeedRef.forward_back_ref > 250){
					 ChassisSpeedRef.forward_back_ref =  250 +  (ChassisSpeedRef.forward_back_ref - 250) * 0.15f;
					}
					else if(ChassisSpeedRef.forward_back_ref < -250)
					{
						ChassisSpeedRef.forward_back_ref =  -250 +  (ChassisSpeedRef.forward_back_ref + 250) * 0.15f;
					}
					if(ChassisSpeedRef.left_right_ref > 250)
					{
					 ChassisSpeedRef.left_right_ref =  250 +  (ChassisSpeedRef.left_right_ref - 250) * 0.15f;
					}
					else if(ChassisSpeedRef.left_right_ref < -250)
					{
						ChassisSpeedRef.left_right_ref =  -250 +  (ChassisSpeedRef.left_right_ref + 250) * 0.15f;
					}
				}
			}
		}
		
		MouseShootControl(mouse);
	}
	
	lastKey = key->v;
}

/////////////////////////取弹模式/////////////////////////////
uint8_t MaybePrepare =0;
uint8_t isAM1Init = 0;
void GetBulletControlprocess(Remote *rc,Mouse *mouse, Key *key)
{
	if(GetWorkState() == NORMAL_STATE)
	{
		//鼠标控制pitch&yaw
		pitchAngleTarget += mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT; 
		if(key->v == 0x0400) GMMode = LOCK;    //锁定云台  G
		if(key->v == 0x0420) GMMode = UNLOCK;  //解锁云台  G + Ctrl			
		if(GMMode == UNLOCK) 
		{
			yawAngleTarget    -= mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;
		}
		if(key->v & 0x01)  // key: w
		{
			ChassisSpeedRef.forward_back_ref = forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
			
		}
		else if(key->v & 0x02) //key: s
		{
			ChassisSpeedRef.forward_back_ref = -forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
			
		}
		else
		{
			ChassisSpeedRef.forward_back_ref = 0;
			FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		}
		if(key->v & 0x04)  // key: d
		{
			ChassisSpeedRef.left_right_ref = -left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
			
		}
		else if(key->v & 0x08) //key: a
		{
			ChassisSpeedRef.left_right_ref = left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
			
		}
		else
		{
			ChassisSpeedRef.left_right_ref = 0;
			LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		}
		if(key->v & 0x80)	//key:e  检测第8位是不是1
		{
			ChassisSpeedRef.rotate_ref=-rotate_speed*RotSpeedRamp.Calc(&RotSpeedRamp);
		}
		else if(key->v & 0x40)	//key:q  
		{
			ChassisSpeedRef.rotate_ref=rotate_speed*RotSpeedRamp.Calc(&RotSpeedRamp);
		}
		else 
		{
			ChassisSpeedRef.rotate_ref = 0;
			RotSpeedRamp.ResetCounter(&RotSpeedRamp);
		}
		if(GMMode == LOCK)
		{
			ChassisSpeedRef.rotate_ref -= mouse->x/27.0*3000;
			yawAngleTarget = -ChassisSpeedRef.rotate_ref * forward_kp / 2000;
		}
		
		AM1RAngleTarget +=(rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_ARM_SPEED_REF_FACT; //右侧电机 

		AM1LAngleTarget =-AM1RAngleTarget; //左侧电机
		
		if(isAM1Init)
		{
			if(AM1RAngleTarget>0) AM1RAngleTarget=0;
			if(AM1RAngleTarget<-190) AM1RAngleTarget=-190;
		
			if(AM1LAngleTarget<0) AM1LAngleTarget=0;
			if(AM1LAngleTarget>190) AM1LAngleTarget=190;
		}
			//prepare
			if(lastKey == 0x0000 && key->v == 0x0800)//z				
			{
				MaybePrepare = 1;
					//HERO_Order = HERO_MANUL_PREPARE;
			}
			if(MaybePrepare == 1)
			{
				if(lastKey == 0x0800 && key->v == 0x0000)//z				
				{
					MaybePrepare = 0;
					HERO_Order = HERO_MANUL_PREPARE;
				}
			}
			//recover
		 if(key->v == 0x0810)//shift+z
			{
				MaybePrepare = 0;
				HERO_Order = HERO_MANUL_RECOVER;
			}
			//load
			else if(key->v & 0x4000)//v
			{
				HERO_Order=HERO_MANUL_LOAD;
			}
			//grip
			 if(lastKey == 0x0000 && key->v & 0x1000)//x
			{
				GRIP_SOV_ON();
			}
			//release
			if(key->v == 0x1010)//x+shift
			{
				GRIP_SOV_OFF();
			}
			//up
			else if(lastKey == 0x0000 && key->v == 0x2000)//c
			{
				HERO_Order = HERO_AUTO_GET3BOX;
				CMFRRealAngle = 0.0;
				CMFLRealAngle = 0.0;
				CMBRRealAngle = 0.0;
				CMBLRealAngle = 0.0;
				CMFRAngleTarget = 0;
				CMFLAngleTarget = 0;
				CMBRAngleTarget = 0;
				CMBLAngleTarget = 0;
			}
			if(key->v == 0x8000)//b
			{//待标定
				if(PIR_R_Ready && PIR_L_Ready)
				{
					HERO_Order = HERO_MANUL_PREPARE;
				}
			}
			lastKey = key->v;
		
//		if(inputmode==GETBULLET_INPUT)
//		{
//			if(GetBulletState == NO_GETBULLET)
//			{
//					
//			}
//			else if(GetBulletState == GEBULLET_PREPARE)
//			{
//				HERO_Order = HERO_MANUL_PREPARE;
//			}
//			else if(GetBulletState == MANUAL_GETBULLET)
//			{
//				
//			}
//			else if(GetBulletState == AUTO_GETBULLET)
//			{
//				
//			}
//	  }
			
	RemoteGetBulletControl(&g_switch1, rc->s1);
}

}


