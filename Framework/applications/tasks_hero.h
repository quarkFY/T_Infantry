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
	//�ֶ���������
	HERO_MANUL_PREPARE = 0,
	HERO_MANUL_LOAD= 1,
	HERO_MANUL_FETCH= 2,
	HERO_MANUL_READY = 3,
	HERO_MANUL_GRIP = 4,
	HERO_MANUL_DISCARD = 5,
	//������,��������̨�ⲻ������������
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
//	HERO_GRABING =11,//̧�ߣ������ץס��״̬��û�п����Ƿ�ץס��
//	
//	HERO_MANUAL_PLACING=12,//�ֶ�����ץȡʱ���˶������е�״̬���˶���ɺ��״̬ΪENGINEER_GRABING
//	HERO_MANUAL_FETCHING=13,//�ֶ�����ץȡ���˶�������
//	HERO_MANUAL_STRETCHED=14,//�ֶ�ץȡ�˶�������״̬
//	
////	HERO_BELT_MOVING=15,//Ƥ�����ֶ�����
//	
//	HERO_RECOVERING=16,
//}HERO_State_t;


//extern HERO_State_t HERO_State;

//extern uint8_t engineering_task_on;
//�ⲿ����
//uint8_t HERO_grab_somthing();
//void StartNewLoadTask();
//void StartNewFetchTask();
//void StopHEROTask();

//void OrderReplace();
//void HEROPrintState();
//void SetFetchHeight(uint8_t HeightIndex);


//�ڲ���������

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

