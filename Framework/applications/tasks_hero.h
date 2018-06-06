#ifndef __TASKS_HERO_H
#define __TASKS_HERO_H

#include "stdint.h"
void HeroTask(void const * argument);
void HERO_prepare(void);
void HERO_recover(void);
void HERO_step(float angle1,float angle2,float angle3);
//void HERO_step_slow(float angle1,float angle2,float angle3);
void HERO_load(void);
uint8_t gapOK(float AngleTarget,float RealAngle);


typedef enum
{
	//手动操作命令
	HERO_MANUL_PREPARE = 0,
	HERO_MANUL_LOAD= 1,
	//空命令,除底盘云台外不存在其他功能
	HERO_STANDBY = 2,
	HERO_MANUL_RECOVER = 3,
	HERO_MANUL_FETCH = 4,
	HERO_SHOOT_LOAD = 5,
	
	
}HERO_Order_t;

extern HERO_Order_t HERO_Order;
#endif
