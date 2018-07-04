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

extern float AM3RAngleTarget ;
extern float AM1LAngleTarget ;

extern float AM1LRealAngle ;
extern float AM1RRealAngle ;


extern float AM3RRealAngle ;
extern float LastAM1LAngleTarget;
extern float LastAM1RAngleTarget;


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
HERO_Order_t HERO_Order= HERO_INIT;
HERO_Order_t Last_HERO_Order=HERO_STANDBY;
extern FrictionWheelState_e g_friction_wheel_state; 
 Chassis_Mode_e FrontWheel_Mode = CHASSIS_NORMAL, Last_FrontWheel_Mode = CHASSIS_NORMAL,BehindWheel_Mode = CHASSIS_NORMAL, Last_BehindWheel_Mode = CHASSIS_NORMAL;

void HeroTask(void const * argument)
{
	while(1)
		
	{
		switch(HERO_Order)
			{
				case HERO_INIT:
				{
					HERO_init();
				}break;
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
				case HERO_AUTO_GETBOX:
				{
					HERO_auto_getOneBox();
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

extern uint8_t isAM1Init;
void HERO_init(void)
{
	uint8_t AMstack = 0;
	uint16_t cnt;
	while(!AMstack)
	{
		if((fabs(AM1RRealAngle-AM1RAngleTarget)>16 || fabs(AM1LRealAngle-AM1LAngleTarget)>16) && cnt<30)
		{
			cnt++;
		}
		else if(cnt==30)
		{
			cnt = 0;
			AMstack = 1;
		}
		else
		{
			AM1RAngleTarget -= 1;
		  AM1LAngleTarget += 1;
			cnt = 0;
		}
		osDelay(12);
	}
	if(AMstack)
	{
		AM1RRealAngle = -190;
		AM1LRealAngle = 190;
		AM1RAngleTarget = -190;
		AM1LAngleTarget = 190;
		HERO_step(0,0,0,80);
		isAM1Init = 1;
	}
	else
	{
		
	}
	HERO_Order = HERO_MANUL_FETCH;
}

void HERO_prepare(void)
{
	HERO_step(150,0,0,40);
}

void HERO_standby(void)
{
	HERO_step(125,0,0,4);
}

void HERO_load(void)
{
	HERO_step(135,0,0,30);
	HERO_step(5,0,0,30);
	osDelay(500);
	HERO_step(135,0,0,40);
}


void HERO_recover()
{
	HERO_step(0,0,0,20);
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
		if(HERO_Order == HERO_MANUL_FETCH) return;
		HERO_prepare();
		osDelay(500);
		if(HERO_Order == HERO_MANUL_FETCH) return;
		GRIP_SOV_ON();
		osDelay(500);
	  if(HERO_Order == HERO_MANUL_FETCH) return;
		HERO_load();
		osDelay(500);
		PM2AngleTarget = PM2AngleTarget - 150;
	  if(HERO_Order == HERO_MANUL_FETCH) return;
		GRIP_SOV_OFF();
		osDelay(500);
		PM2AngleTarget = PM2AngleTarget - 200;
	  if(HERO_Order == HERO_MANUL_FETCH) return;
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
		  if(HERO_Order == HERO_MANUL_FETCH) return;
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
	
}

//给值时，AM1R为正，AM3R为负
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
					osDelay(15);
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
