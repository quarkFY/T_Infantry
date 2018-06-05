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
//�ο����̳�������Ӣ��ȡ��ģ��
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

//����ADC����
extern uint32_t ADC_Value[60];
//״̬������ö��
HERO_Order_t HERO_Order=HERO_STANDBY;
HERO_Order_t Last_HERO_Order=HERO_STANDBY;

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

//��ֵʱ��AM1R,AM2LΪ����AM3RΪ��
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
