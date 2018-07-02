#include "tasks_hero.h"
#include "stdint.h"
#include "tasks_motor.h"
#include "cmsis_os.h"
#include "pid_regulator.h"
#include "stdlib.h"
#include "utilities_debug.h"
#include "drivers_uartupper_user.h"
#include "drivers_canmotor_user.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartrc_user.h"
#include "tasks_arm.h"
#include "peripheral_sov.h"
#include "tasks_motor.h"

#include <stdlib.h>
#include <math.h>
//参考工程车，增加英雄取弹模块
uint8_t HERO_task_on=0;
extern Get_Bullet_e GetBulletState;
extern float AM1RAngleTarget ;
extern float AM2RAngleTarget ;
extern float AM3RAngleTarget ;
extern float AM1LAngleTarget ;
extern float AM2LAngleTarget ;
extern float AM1LRealAngle ;
extern float AM1RRealAngle ;
extern float AM2LRealAngle ;
extern float AM2RRealAngle ;
extern float AM3RRealAngle ;
extern float LastAM1LAngleTarget;
extern float LastAM1RAngleTarget;
extern float LastAM2LAngleTarget;
extern float LastAM2RAngleTarget;
extern float LastAM3RAngleTarget;

extern float CMFRAngleTarget ;
extern float CMBRAngleTarget ;
extern float CMFLAngleTarget ;
extern float CMBLAngleTarget ;
extern float CMFLRealAngle ;
extern float CMFRRealAngle ;
extern float CMBLRealAngle ;
extern float CMBRRealAngle ;

extern float PM2AngleTarget,PM2RealAngle;
uint8_t PM2RotateEnable = 1;
uint16_t PM2RotateCounter = 0;

//红外ADC配置
extern uint32_t ADC_Value[60];
//状态及命令枚举
HERO_Order_t HERO_Order=HERO_MANUL_RECOVER;
HERO_Order_t Last_HERO_Order=HERO_STANDBY;
extern FrictionWheelState_e g_friction_wheel_state; 
 Chassis_Mode_e FrontWheel_Mode = CHASSIS_NORMAL, Last_FrontWheel_Mode = CHASSIS_NORMAL,BehindWheel_Mode = CHASSIS_NORMAL, Last_BehindWheel_Mode = CHASSIS_NORMAL;

void HeroTask(void const * argument)
{
	while(1)
	{
	
		switch(HERO_Order)
			{
				case HERO_MANUL_PREPARE:
				{
					HERO_manul_prepare();
				}break;
				case HERO_MANUL_FETCH:
				{
					
				}break;
				case HERO_MANUL_LOAD:
				{
           HERO_manul_load();
				}break;
				case HERO_MANUL_RECOVER:
				{
					HERO_manul_recover();
				}break;
				case HERO_STANDBY:
				{
					HERO_manul_standby();
				}break;
				case HERO_AUTO_GET3BOX:
				{
					HERO_auto_getThreeBox();
					//HERO_auto_getOneBox();
					HERO_Order = HERO_MANUL_FETCH;
				}break;
				case HERO_SHOOT_LOAD:
				{
					shootLoad();
					HERO_Order = HERO_MANUL_FETCH;
				}break;
				case HERO_STEADY_ROTATE:
				{
					if(g_friction_wheel_state == FRICTION_WHEEL_ON)
					{
						//正常一直转
						if(PM2RotateEnable == 1)
						{
				
							PM2RotateCounter++;
							if(PM2RotateCounter==2000)
							{
								PM2RotateCounter = 0;
								PM2AngleTarget += 150;
								PM2RotateEnable = 0;
							}
							else PM2AngleTarget-=2;
						}
						//堵转回90度
						else if(PM2RotateEnable == 2)
						{
								PM2AngleTarget+=90;
								PM2RotateEnable = 0;
						}
						//回转到位
						else if(fabs(PM2AngleTarget-PM2RealAngle)<5)
						{
								PM2RotateEnable = 1;
						}
						//堵转检测
						if(fabs(PM2AngleTarget-PM2RealAngle)>200 && PM2RotateEnable == 1)
						{
							PM2AngleTarget=PM2RealAngle;
							PM2RotateEnable = 2;
							PM2RotateCounter = 0;
						}
					}
					else
					{
						HERO_Order = HERO_MANUL_FETCH;
						PM2AngleTarget=PM2RealAngle;
					}
				}break;
				default:
				  fw_Error_Handler();
					
			}
			osDelay(2);

	}
}

void HERO_prepare(void)
{
	HERO_step(190,0,0,20);
}

void HERO_standby(void)
{
	HERO_step(150,0,0,2);
}

void HERO_load(void)
{
	HERO_step(160,0,0,15);
	HERO_step(30,0,0,15);
	osDelay(500);
	HERO_step(160,0,0,20);
}


void HERO_recover()
{
	HERO_step(10,0,0,20);
}

void HERO_manul_prepare(void)
{
	HERO_prepare();
	HERO_Order = HERO_MANUL_FETCH;
}

void HERO_manul_standby(void)
{
	HERO_standby();
	HERO_Order = HERO_MANUL_FETCH;
}

void HERO_manul_load(void)
{
	HERO_load();
	HERO_Order = HERO_MANUL_FETCH;
}


void HERO_manul_recover()
{
	HERO_recover();
	HERO_Order = HERO_MANUL_FETCH;
}


void HERO_auto_getOneBox()
{
		HERO_prepare();
		osDelay(500);
		GRIP_SOV_ON();
		osDelay(500);
		HERO_load();
		osDelay(500);
		GRIP_SOV_OFF();
		osDelay(500);
		HERO_standby();
}

//For Demo
void HERO_auto_getThreeBox()
{
		CMFRAngleTarget = 0;
		CMFLAngleTarget = 0;
		CMBRAngleTarget = 0;
		CMBLAngleTarget = 0;
		CMFRRealAngle = 0.0;
		CMFLRealAngle = 0.0;
		CMBRRealAngle = 0.0;
		CMBLRealAngle = 0.0;	
		HERO_auto_getOneBox();
		HERO_getbullet_moveleft(0,247,42);
		HERO_auto_getOneBox();
		HERO_getbullet_moveleft(0,260,42);
		HERO_auto_getOneBox();
}

void HERO_getbullet_moveleft(uint8_t round,float angle,uint16_t step)
{
	float step_length;
	uint16_t cnt = 0;
	step_length = (angle - 0)/step;
	
	for(uint8_t r=0;r<round;r++){
		for(uint16_t i=0;i<360;i++)
		{
			CMFRAngleTarget += 1;
			CMFLAngleTarget += 1;
			CMBRAngleTarget -= 1;
			CMBLAngleTarget -= 1;
			if(r<3) osDelay(10);
			else osDelay(2);
		}
		CMFRAngleTarget = 360;
		CMFLAngleTarget = 360;
		CMBRAngleTarget = -360;
		CMBLAngleTarget = -360;
		osDelay(100);
		CMFRAngleTarget = 0;
		CMFLAngleTarget = 0;
		CMBRAngleTarget = 0;
		CMBLAngleTarget = 0;
		CMFRRealAngle = 0.0;
		CMFLRealAngle = 0.0;
		CMBRRealAngle = 0.0;
		CMBLRealAngle = 0.0;
	}
	
  for(uint16_t i=0;i<step;i++)
	{
			CMFRAngleTarget += step_length;
			CMFLAngleTarget += step_length;
			CMBRAngleTarget -= step_length;
			CMBLAngleTarget -= step_length;
			osDelay(20);
	}
		CMFRAngleTarget = angle;
		CMFLAngleTarget = angle;
		CMBRAngleTarget = -angle;
		CMBLAngleTarget = -angle;
		osDelay(100);
		CMFRAngleTarget = 0;
		CMFLAngleTarget = 0;
		CMBRAngleTarget = 0;
		CMBLAngleTarget = 0;
		CMFRRealAngle = 0.0;
		CMFLRealAngle = 0.0;
		CMBRRealAngle = 0.0;
		CMBLRealAngle = 0.0;
	while((fabs(CMFLAngleTarget-CMFLRealAngle)>5 || fabs(CMFRAngleTarget-CMFRRealAngle)>5) && cnt < 1500)
	{
			cnt++;
			osDelay(1);
	}
}

//给值时，AM1R,AM2L为正，AM3R为负
void HERO_step(float angle1,float angle2,float angle3,uint16_t step)
{
	float step_length1;
	uint16_t cnt = 0;
	//uint16_t step = 20;
	step_length1 = (angle1 - AM1LRealAngle)/step;
  for(uint16_t i=0;i<step;i++)
				{
					AM1RAngleTarget -= step_length1;
					AM1LAngleTarget += step_length1;
					osDelay(30);
				}
				AM1RAngleTarget = -angle1;
				AM1LAngleTarget = angle1;
	while((fabs(AM1LAngleTarget-AM1LRealAngle)>3 || fabs(AM1RAngleTarget-AM1RRealAngle)>3) && cnt < 1000)
	{
			cnt++;
			osDelay(1);
	}
			
}

//void HERO_step_slow(float angle1,float angle2,float angle3)
//{
//	float step_length1,step_length2,step_length3;
//	uint16_t step = 40;
//	step_length1 = (angle1 - AM1RRealAngle)/step;
//	step_length2 = (angle2 - AM2LRealAngle)/step;
//	step_length3 = (angle3 - AM3RRealAngle)/step;
//  for(uint32_t i=0;i<step;i++)
//				{
//					AM1RAngleTarget += step_length1;
//					AM1LAngleTarget -= step_length1;
//					AM2LAngleTarget += step_length2;
//					AM2RAngleTarget -= step_length2;
//					AM3RAngleTarget += step_length3;
//					osDelay(50);
//				}
//				AM1RAngleTarget = angle1;
//				AM1LAngleTarget = -angle1;
//				AM2LAngleTarget = angle2;
//				AM2RAngleTarget = -angle2;
//				AM3RAngleTarget = angle3;
//			
//}
