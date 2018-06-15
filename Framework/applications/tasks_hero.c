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

extern float PM2AngleTarget,PM2RealAngle;
uint8_t PM2RotateEnable = 1;
uint8_t PM2RotateCounter = 0;

//红外ADC配置
extern uint32_t ADC_Value[60];
//状态及命令枚举
HERO_Order_t HERO_Order=HERO_STANDBY;
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
					HERO_prepare();
				}break;
				case HERO_MANUL_FETCH:
				{
					
				}break;
				case HERO_MANUL_LOAD:
				{
           HERO_load();
				}break;
				case HERO_MANUL_RECOVER:
				{
					HERO_recover();
				}break;
				case HERO_STANDBY:
				{
					
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
						PM2RotateCounter++;
						if(PM2RotateCounter == 80)
						{
							PM2AngleTarget -=90;
							PM2RotateEnable=0;
							PM2RotateCounter=0;
						}
							
						if(PM2RotateEnable == 1)
						{
							PM2AngleTarget+=20;
							osDelay(20);
						}
						//堵转回90度
						else if(PM2RotateEnable == 2)
						{
								PM2AngleTarget-=90;
								PM2RotateEnable = 0;
						}
						//回转到位
						else if(fabs(PM2AngleTarget-PM2RealAngle)<5)
						{
								PM2RotateEnable = 1;
						}
						//堵转检测
						if((PM2AngleTarget-PM2RealAngle)>200)
						{
							PM2AngleTarget=PM2RealAngle;
							PM2RotateEnable = 2;
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
	HERO_step(180,0,0);
	HERO_Order = HERO_MANUL_FETCH;
}

void HERO_load(void)
{
	HERO_step(90,0,0);
	osDelay(1000);
	HERO_step(180,0,0);
	HERO_Order = HERO_MANUL_FETCH;
}


void HERO_recover()
{
	HERO_step(10,0,0);
	HERO_Order = HERO_MANUL_FETCH;
}

//给值时，AM1R,AM2L为正，AM3R为负
void HERO_step(float angle1,float angle2,float angle3)
{
	float step_length1;
	uint16_t step = 20;
	step_length1 = (angle1 - AM1RRealAngle)/step;
  for(uint32_t i=0;i<step;i++)
				{
					AM1RAngleTarget += step_length1;
					AM1LAngleTarget -= step_length1;
					osDelay(30);
				}
				AM1RAngleTarget = angle1;
				AM1LAngleTarget = -angle1;
			
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
