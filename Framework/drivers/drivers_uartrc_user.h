/**
  ******************************************************************************
  * File Name          : drivers_uartrc_user.h
  * Description        : 遥控器串口
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 遥控器用户函数
  ******************************************************************************
  */
#ifndef DRIVERS_UARTRC_USER_H
#define DRIVERS_UARTRC_USER_H

#include "utilities_iopool.h"

/*没用到
typedef struct{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int8_t s1;
	int8_t s2;
}Remote_t;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse_t;	

typedef struct{
	uint16_t v;
}Key_t;
*/

/*没用到
typedef struct{
	Remote_t rc;
	Mouse_t mouse;
	Key_t key;
}RC_CtrlData_t; 
*/

IOPoolDeclare(rcUartIOPool, struct{uint8_t ch[18];});



typedef enum
{
	NORMAL = 0,
	EMERGENCY = 1,
}Emergency_Flag;

typedef enum
{
	LOW_s = 0,
	NORMAL_s = 1,
	HIGH_s = 2,
}Move_Speed_e;

typedef enum
{
	NO_GETBULLET = 0,
	AUTO_GETBULLET = 1,
	MANUL_GETBULLET = 2,
	HERO_STANDBY=3,
}Get_Bullet_e;

typedef enum
{
	NO_SHOOT = 0,
	AUTO_SHOOT = 1,
	MANUL_SHOOT_ONE = 2,
	MANUL_SHOOT_FOUR = 3,
}Shoot_State_e;

typedef enum
{
	LOCK,
	UNLOCK,
}GMMode_e;

typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 3,
	GETBULLET_INPUT = 2,
}InputMode_e;

typedef enum
{
	FRICTION_WHEEL_OFF = 0,
	FRICTION_WHEEL_START_TURNNING = 1,
	FRICTION_WHEEL_ON = 2,
}FrictionWheelState_e;

typedef enum
{
	NO_GETGOLF = 0,
	MANUL_GETGOLF = 1,
  AUTO_GETGOLF = 2,
}GetGolf_State_e;

/*没用到
typedef enum
{
	OPEN = 0,
	CLOSE = 1,
}Slab_Mode_e;
*/

//Shoot_State_e GetShootMode(void);
//void SetShootMode(Shoot_Mode_e v);
Emergency_Flag GetEmergencyFlag(void);
void SetEmergencyFlag(Emergency_Flag v);
Move_Speed_e GetMoveSpeed(void);
void SetMoveSpeed(Move_Speed_e v);

InputMode_e GetInputMode(void);
void SetShootState(Shoot_State_e v);
Shoot_State_e GetShootState(void);
void SetFrictionState(FrictionWheelState_e v);
FrictionWheelState_e GetFrictionState(void);

Get_Bullet_e GetGetBulletState(void);
void SetGetBulletState(Get_Bullet_e v);


#endif
