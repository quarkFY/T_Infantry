#ifndef __TASKS_HERO_H
#define __TASKS_HERO_H

#include "stdint.h"
void HeroTask(void const * argument);

void HERO_prepare(void);
void HERO_recover(void);
void HERO_standby(void);
void HERO_load(void);

void HERO_init(void);
void HERO_manul_prepare(void);
void HERO_manul_recover(void);
void HERO_manul_standby(void);
void HERO_step(float angle1,float angle2,float angle3,uint16_t step);
void HERO_getbullet_moveleft(uint8_t round,float angle1,uint16_t step);
//void HERO_step_slow(float angle1,float angle2,float angle3);
void HERO_manul_load(void);

void HERO_auto_getOneBox(void);
void HERO_auto_getThreeBox(void);

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
	HERO_STEADY_ROTATE = 6,
	HERO_AUTO_GET3BOX = 7,
	HERO_INIT = 8,
	HERO_AUTO_GETBOX =9,
	
	
}HERO_Order_t;

extern HERO_Order_t HERO_Order;
#endif
