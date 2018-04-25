#ifndef __TASKS_HERO_H
#define __TASKS_HERO_H

#include "stdint.h"
void HeroTask(void const * argument);
void HERO_Manul_Discard(void);
void HERO_prepare(void);
void HERO_recover(void);
void HERO_step(float angle1,float angle2,float angle3);
void HERO_step_slow(float angle1,float angle2,float angle3);
void HERO_Load(void);
void RaiseControlProcess(void);
uint8_t gapOK(float AngleTarget,float RealAngle);
//void HERO_Adjustdistance();


typedef enum
{
	//手动操作命令
	HERO_MANUL_PREPARE = 0,
	HERO_MANUL_LOAD= 1,
	HERO_MANUL_FETCH= 2,
	HERO_MANUL_READY = 3,
	HERO_MANUL_GRIP = 4,
	HERO_MANUL_DISCARD = 5,
	//空命令,除底盘云台外不存在其他功能
	HERO_STANDBY = 6,
	HERO_MANUL_RECOVER = 7,
	
	
}HERO_Order_t;

extern HERO_Order_t HERO_Order;

//typedef enum
//{
//	HERO_NO_MOVE = 0,
//	
//	HERO_ADJUSTING = 2,
//	HERO_DISTANCE_OK = 3,
//	HERO_DISTANCE_AND_ROTATION_OK = 4,
//	HERO_PLACING = 5,
//	HERO_FETCHING = 6,
//	HERO_PLACING_COMPLETE =7,
//	HERO_FETCHING_COMPLETE =8,
//	HERO_DISCASDING =9,
//	HERO_REPLACING =10,
//	HERO_GRABING =11,//抬高，伸出，抓住的状态（没有考虑是否抓住）
//	
//	HERO_MANUAL_PLACING=12,//手动控制抓取时的运动过程中的状态，运动完成后的状态为ENGINEER_GRABING
//	HERO_MANUAL_FETCHING=13,//手动控制抓取的运动过程中
//	HERO_MANUAL_STRETCHED=14,//手动抓取运动结束的状态
//	
////	HERO_BELT_MOVING=15,//皮带轮手动控制
//	
//	HERO_RECOVERING=16,
//}HERO_State_t;


//extern HERO_State_t HERO_State;

//extern uint8_t engineering_task_on;
//外部调用
//uint8_t HERO_grab_somthing();
//void StartNewLoadTask();
//void StartNewFetchTask();
//void StopHEROTask();

//void OrderReplace();
//void HEROPrintState();
//void SetFetchHeight(uint8_t HeightIndex);


//内部函数声明

//void calibrationStep();
//uint8_t HERO_Grip_and_Load();
//void HERO_Adjustdistance();
//void HERO_AdjustRotationAndDistance();
//void HERO_Recover();
//void HERO_Discard_Stuff();
//uint8_t HERO_Replacing();
//uint8_t HERO_manual_fetch();
//uint8_t HERO_manual_load();
//uint8_t HERO_grab_somthing();
//uint8_t HERO_belt_back();
//uint8_t HERO_belt_forward();
#endif

