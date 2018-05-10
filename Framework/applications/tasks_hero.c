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

//红外ADC配置
extern uint32_t ADC_Value[60];
//Engineering_Step_t Engineering_Step = ENGINEERING_STOP;
//Engineering_Step_t Engineering_Last_Step = ENGINEERING_STOP;
//状态及命令枚举
HERO_Order_t HERO_Order=HERO_STANDBY;
HERO_Order_t Last_HERO_Order=HERO_STANDBY;

 Chassis_Mode_e FrontWheel_Mode = CHASSIS_NORMAL, Last_FrontWheel_Mode = CHASSIS_NORMAL,BehindWheel_Mode = CHASSIS_NORMAL, Last_BehindWheel_Mode = CHASSIS_NORMAL;
//HERO_State_t HERO_State=HERO_NO_MOVE;

//int32_t fetch_height=-11500;//-1000第二种

//extern uint32_t ADC_Value[60];
//uint32_t targetValue2=1300;

//uint32_t ad2_bias=0;
//uint32_t ad3_bias=0;//以ad1为基准

//uint32_t ad6_bias=0;//以ad5为基准
//uint32_t ad7_bias=0;//以ad4为基准
////假设这个pid的输出和遥控器是同一个单位
//fw_PID_Regulator_t HERO_ForwardBackPID = fw_PID_INIT(0.5, 0.0, 0.0, 150, 150, 150, 50);
//fw_PID_Regulator_t HERO_LeftRightPID = fw_PID_INIT(0.5, 0.0, 0.0, 150, 150, 150, 50);
//fw_PID_Regulator_t HERO_RotatePID = fw_PID_INIT(0.5, 0.0, 0.0, 150, 150, 150, 50);

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
				case HERO_MANUL_READY:
				{

				}break;
				case HERO_MANUL_GRIP:
				{

				}break;
				case HERO_MANUL_LOAD:
				{
           HERO_load();
				}break;
				case HERO_MANUL_DISCARD:
				{
			
				}break;
				case HERO_MANUL_RECOVER:
				{
					HERO_recover();
				}break;
				case HERO_STANDBY:
				{
					RaiseControlProcess();
				}break;
				default:
				  fw_Error_Handler();
					
			}
			Last_HERO_Order = HERO_Order;
			osDelay(2);

	}
}
	
void HERO_prepare(void)
{
	HERO_step(45,45,0);
	osDelay(300);
	HERO_step(45,45,-90);
	osDelay(300);
	HERO_step(50,0,-130);	
	osDelay(600);
	HERO_step(65,-5,-180);
	HERO_Order = HERO_MANUL_FETCH;
	//70，0，-180
}

void HERO_load(void)
{
	armUpleft();
	osDelay(100);
	armUpleft();
	osDelay(200);
//	HERO_step(100,145,-225);
//	osDelay(1500);
//	HERO_step(100,20,-180);
	HERO_Order = HERO_MANUL_FETCH;
}


void HERO_recover()
{
	HERO_step(45,45,0);
	osDelay(800);
	HERO_step(0,0,0);
	HERO_Order = HERO_STANDBY;
}

//给值时，AM1R,AM2L为正，AM3R为负
void HERO_step(float angle1,float angle2,float angle3)
{
	float step_length1,step_length2,step_length3;
	uint16_t step = 20;
	step_length1 = (angle1 - AM1RRealAngle)/step;
	step_length2 = (angle2 - AM2LRealAngle)/step;
	step_length3 = (angle3 - AM3RRealAngle)/step;
  for(uint32_t i=0;i<step;i++)
				{
					AM1RAngleTarget += step_length1;
					AM1LAngleTarget -= step_length1;
					AM2LAngleTarget += step_length2;
					AM2RAngleTarget -= step_length2;
					AM3RAngleTarget += step_length3;
			//		if(HERO_Order==HERO_STOP)
			//		{	
			//			fw_printfln("stop called when strech!");
			//			return 0;
			//		}
					osDelay(30);
				}
				AM1RAngleTarget = angle1;
				AM1LAngleTarget = -angle1;
				AM2LAngleTarget = angle2;
				AM2RAngleTarget = -angle2;
				AM3RAngleTarget = angle3;
			
}

void HERO_step_slow(float angle1,float angle2,float angle3)
{
	float step_length1,step_length2,step_length3;
	uint16_t step = 40;
	step_length1 = (angle1 - AM1RRealAngle)/step;
	step_length2 = (angle2 - AM2LRealAngle)/step;
	step_length3 = (angle3 - AM3RRealAngle)/step;
  for(uint32_t i=0;i<step;i++)
				{
					AM1RAngleTarget += step_length1;
					AM1LAngleTarget -= step_length1;
					AM2LAngleTarget += step_length2;
					AM2RAngleTarget -= step_length2;
					AM3RAngleTarget += step_length3;
			//		if(HERO_Order==HERO_STOP)
			//		{	
			//			fw_printfln("stop called when strech!");
			//			return 0;
			//		}
					osDelay(50);
				}
				AM1RAngleTarget = angle1;
				AM1LAngleTarget = -angle1;
				AM2LAngleTarget = angle2;
				AM2RAngleTarget = -angle2;
				AM3RAngleTarget = angle3;
			
}


//
extern uint8_t CMF,CMB;
void RaiseControlProcess()
{
	if (CMF == 1)
	{
		if (FrontWheel_Mode == CHASSIS_LOW)
		{
			FRONT_SOV2_OFF();
			FrontWheel_Mode = CHASSIS_NORMAL;
		}
		else if(FrontWheel_Mode == CHASSIS_NORMAL)
		{
			FRONT_SOV1_ON();
			FrontWheel_Mode = CHASSIS_HIGH;
		}
	}
	if (CMF == 2)
	{
		if (FrontWheel_Mode == CHASSIS_HIGH)
		{
			FRONT_SOV1_OFF();
			FrontWheel_Mode = CHASSIS_NORMAL;
		}
		else if(FrontWheel_Mode == CHASSIS_NORMAL)
		{
			FRONT_SOV2_ON();
			FrontWheel_Mode = CHASSIS_LOW;
		}
	}
	if (CMB == 1)
	{
		if (BehindWheel_Mode == CHASSIS_LOW)
		{
			BEHIND_SOV2_OFF();
			BehindWheel_Mode = CHASSIS_NORMAL;
		}
		else if(FrontWheel_Mode == CHASSIS_NORMAL)
		{
			BEHIND_SOV1_ON();
			BehindWheel_Mode = CHASSIS_HIGH;
		}
	}
	if (CMB == 2)
	{
		if (BehindWheel_Mode == CHASSIS_HIGH)
		{
			BEHIND_SOV1_OFF();
			BehindWheel_Mode = CHASSIS_NORMAL;
		}
		else if(BehindWheel_Mode == CHASSIS_NORMAL)
		{
			BEHIND_SOV2_ON();
			BehindWheel_Mode = CHASSIS_LOW;
		}
	}
	CMF = 0;
	//HIGH TO LOW|LOW TO HIGH????????
//	if(Last_FrontWheel_Mode == CHASSIS_NORMAL && FrontWheel_Mode == CHASSIS_HIGH)
//	{
//		FRONT_SOV1_ON();
//	}
//	else if(Last_FrontWheel_Mode == CHASSIS_NORMAL && FrontWheel_Mode == CHASSIS_LOW)
//	{
//		FRONT_SOV2_ON();
//	}
//	else if(Last_FrontWheel_Mode == CHASSIS_HIGH && FrontWheel_Mode == CHASSIS_NORMAL)
//	{
//		FRONT_SOV1_OFF();
//	}
//	else if(Last_FrontWheel_Mode == CHASSIS_HIGH && FrontWheel_Mode == CHASSIS_LOW)
//	{
//		FRONT_SOV1_OFF();
//		osDelay(1000);
//		FRONT_SOV2_ON();
//	}
//	else if(Last_FrontWheel_Mode == CHASSIS_LOW && FrontWheel_Mode == CHASSIS_NORMAL)
//	{
//		FRONT_SOV2_OFF();
//	}
//	else if(Last_FrontWheel_Mode == CHASSIS_LOW && FrontWheel_Mode == CHASSIS_HIGH)
//	{
//		FRONT_SOV2_OFF();
//		osDelay(1000);
//		FRONT_SOV1_ON();
//	}
//	Last_FrontWheel_Mode = FrontWheel_Mode;
//	
//	if(Last_BehindWheel_Mode == CHASSIS_NORMAL && BehindWheel_Mode == CHASSIS_HIGH)
//	{
//		BEHIND_SOV1_ON();
//	}
//	else if(Last_BehindWheel_Mode == CHASSIS_NORMAL && BehindWheel_Mode == CHASSIS_LOW)
//	{
//		BEHIND_SOV2_ON();
//	}
//	else if(Last_BehindWheel_Mode == CHASSIS_HIGH && BehindWheel_Mode == CHASSIS_NORMAL)
//	{
//		BEHIND_SOV1_OFF();
//	}
//	else if(Last_BehindWheel_Mode == CHASSIS_HIGH && BehindWheel_Mode == CHASSIS_LOW)
//	{
//		BEHIND_SOV1_OFF();
//		osDelay(1000);
//		BEHIND_SOV2_ON();
//	}
//	else if(Last_BehindWheel_Mode == CHASSIS_LOW && BehindWheel_Mode == CHASSIS_NORMAL)
//	{
//		BEHIND_SOV2_OFF();
//	}
//	else if(Last_BehindWheel_Mode == CHASSIS_LOW && BehindWheel_Mode == CHASSIS_HIGH)
//	{
//		BEHIND_SOV2_OFF();
//		osDelay(1000);
//		BEHIND_SOV1_ON();
//	}
//	Last_BehindWheel_Mode = BehindWheel_Mode;
}


//uint8_t gapOK(float AngleTarget,float RealAngle)
//{
//	if((AngleTarget - RealAngle)> -7 && (AngleTarget - RealAngle)< 7)
//		return 1;
//	else
//		return 0;
//}

//int32_t ad1=0,ad2=0,ad3=0;
////工程车调整距离任务
//void HERO_Adjustdistance()
//{
////		static int32_t closecnt=0;
//		//fw_printfln("1");
//		//fw_printf("engineering_adjusting!\r\n");
//		////////////注意！
//		////ad1 右边，ad2，中间，ad3，左边
//		
//		for(uint16_t i=0;i<100;i++)
//		{
//			if(i%5==0)ad1+=ADC_Value[i];
//			if(i%5==1)ad2+=ADC_Value[i];
//			if(i%5==2)ad3+=ADC_Value[i];
//		}
//		ad1/=20;
//		ad2/=20;
//		ad3/=20;
//	}
//		ad2-=ad2_bias;
//		ad3-=ad3_bias;
//		ad5/=20;
//		if(ad5>1000)
//		{
//			//上面有方块
//			Engineering_Order=ENGINEER_ADJUSTDISTANCE_AND_ROTATION;
//			return;
//		}
//		if(ad2<400&&ad1<400&&ad3<400)
//		{
//			osDelay(10);
//			HERO_Order=HERO_STANDBY;
//			return;
//		}
//		if(abs(ad2-targetValue2)<30 && abs(ad1-ad3)<40)
//		{
//			closecnt++;
//		}
//		else
//		{
//			closecnt--;
//		}
//		if(closecnt>60)
//			closecnt=60;
//		else if(closecnt<0)
//			closecnt=0;
//		if(closecnt>40)
//		{
//			fw_printf("close! cnt:%d\r\n",closecnt);
//			fw_printf("%d  %d  %d",ad1,ad2,ad3);
//			HERO_State=HERO_DISTANCE_OK;
//			HERO_Order=HERO;
//			closecnt=0;
//			osDelay(20);
//			return;
//		}
//		HERO_State=HERO_ADJUSTING;
//		//PID
//		HERO_ForwardBackPID.target=targetValue2;
//		HERO_ForwardBackPID.feedback=ad2;
//		HERO_ForwardBackPID.Calc(&HERO_ForwardBackPID);
//		
//		HERO_LeftRightPID.target=0;
//		HERO_LeftRightPID.feedback= ad3-ad1;
//		HERO_LeftRightPID.Calc(&HERO_LeftRightPID);
//		
//		ChassisSpeedRef.forward_back_ref=HERO_ForwardBackPID.output/66.0*3500;
//		ChassisSpeedRef.left_right_ref=HERO_LeftRightPID.output/66.0*3500;
//}
////工程车调整角度
//uint32_t targetValue5=1200;
//void HERO_AdjustRotationAndDistance()
//{
//		static int32_t rotate_closecnt=0;
//		static int32_t left_right_closecnt=0;
//		//fw_printfln("2");
//		fw_printfln("en");
//		//ad4 右边，ad5，ad6中间，ad7，左边
//		int32_t ad1=0,ad2=0,ad3=0,ad4=0,ad5=0,ad6=0,ad7=0;
//		for(uint16_t i=0;i<100;)
//		{
//			ad1+=ADC_Value[i++];
//			ad2+=ADC_Value[i++];
//			ad3+=ADC_Value[i++];
//			ad4+=ADC_Value[i++];
//			ad5+=ADC_Value[i++];
//		}
//		ad1/=20;
//		ad2/=20;
//		ad3/=20;
//		ad4/=20;
//		ad5/=20;
//		ad3-=ad3_bias;
//		if(ad5<400&&ad6<400)
//		{
//			osDelay(10);
//			return;
//		}
//		if(abs(ad4-ad5)<30 && abs(ad5-targetValue5)<40 && abs(ad1-ad3)<40)
//		{
//			rotate_closecnt++;
//		}
//		else
//		{
//			rotate_closecnt--;
//		}
//		//fw_printfln("%d",rotate_closecnt);
//		if(rotate_closecnt>60)
//			rotate_closecnt=60;
//		else if(rotate_closecnt<0)
//			rotate_closecnt=0;
//		if(rotate_closecnt>40)
//		{
//			fw_printf("close! cnt:%d\r\n",rotate_closecnt);
//			fw_printf("%d  %d  %d  %d",ad4,ad5,ad6,ad7);
//			HERO_State=HERO_DISTANCE_AND_ROTATION_OK;
//			HERO_Order=HERO;
//			rotate_closecnt=0;
//			osDelay(20);
//			return;
//		}

//		HERO_State=HERO_ADJUSTING;
//		//PID
//		HERO_RotatePID.target=0;

//		HERO_RotatePID.feedback=ad5-ad4;
//		HERO_RotatePID.Calc(&HERO_RotatePID);
//		
//		HERO_ForwardBackPID.target=targetValue5;
//		HERO_ForwardBackPID.feedback=ad5;
//		HERO_ForwardBackPID.Calc(&HERO_ForwardBackPID);
//		
//		HERO_LeftRightPID.target=0;
//		HERO_LeftRightPID.feedback=ad3-ad1;
//		HERO_LeftRightPID.Calc(&HERO_LeftRightPID);
//		
//		ChassisSpeedRef.forward_back_ref=HERO_ForwardBackPID.output/66.0*3000;
//		ChassisSpeedRef.rotate_ref=HERO_RotatePID.output/66.0*2500;
//		ChassisSpeedRef.left_right_ref=HERO_LeftRightPID.output/66.0*3500;
//		
//}
//英雄车抓取装弹任务
//调用函数
/*
uint8_t HERO_Lift(float value, uint32_t time_milis);
uint8_t HERO_Stretch(float value, uint32_t time_milis);
uint8_t HERO_Grab(float value, uint32_t time_milis);
uint8_t taskDelay(uint32_t time_milis);
uint8_t Hero_Grip_and_Load()
{
//		ChassisSpeedRef.forward_back_ref=0;
//		ChassisSpeedRef.left_right_ref=0;
//		ChassisSpeedRef.rotate_ref=0;
		if(GetBulletState == MANUAL_GETBULLET)
		{
			HERO_Order=HERO_STANDBY;
			osDelay(2);
			return 1;
		}
		if(HERO_State==HERO_DISTANCE_OK)
		{
			fw_printfln("Place");
			HERO_State=HERO_PLACING;
			//先抬升,爪子伸开
			if(!HERO_Lift(-5000,100)) return 0;
			if(!HERO_Grab(grab_limit,100)) return 0;
			//带传送，送障碍快
			aux4_targetSpeed=35000;
			if(!taskDelay(1500))
			{
				aux4_targetSpeed=0;
				return 0;
			}
			aux4_targetSpeed=0;
			//再下降，抓取
			if(!HERO_Lift(-600,100)) return 0;
			//if(!taskDelay(100)) return 0;
			if(!HERO_Grab(0,200)) return 0;
			if(!taskDelay(100)) return 0;
			//抬升
			if(!HERO_Lift(-32000,300)) return 0;
			//伸出
			if(!HERO_Stretch(aux2_limit,1500)) return 0;
			//if(!taskDelay(500)) return 0;
			//放下
			if(!HERO_Lift(-28000,100)) return 0;
			//松开
			if(!HERO_Grab(grab_limit,100)) return 0;
//			//抬升
//			if(!HERO_Lift(-20000,300)) return 0;
			//回位
			if(!HERO_Stretch(0,800)) return 0;
			//if(!HERO_Lift(-800,250)) return 0;
			//状态
			HERO_State=HERO_PLACING_COMPLETE;
		}
		else if(HERO_State==HERO_DISTANCE_AND_ROTATION_OK)
		{
			ChassisSpeedRef.forward_back_ref=0;
			ChassisSpeedRef.left_right_ref=0;
			ChassisSpeedRef.rotate_ref=0;
			fw_printfln("fetch");
			HERO_State=HERO_FETCHING;
			//先抬升
			if(!HERO_Lift(-20000,500)) return 0;
			if(!HERO_Grab(grab_limit,50)) return 0;
			//伸出
			if(!HERO_Stretch(aux2_limit,1000)) return 0;
			//抓
			if(!HERO_Lift(fetch_height,400)) return 0;
			//if(!taskDelay(200)) return 0;
			if(!HERO_Grab(0,200)) return 0;
			if(!taskDelay(100)) return 0;
			//抬升
			if(!HERO_Lift(-32000,400)) return 0;
			
			HERO_State=HERO_GRABING;
		}
		osDelay(2);
		HERO_Order=HERO_STANDBY;
		return 1;
}

uint8_t HERO_Replacing()
{
	if(HERO_State==HERO_GRABING|| HERO_State==HERO_REPLACING)
	{
		HERO_State=HERO_REPLACING;
		//带传送，送障碍快
		aux4_targetSpeed=-35000;
		//回位
		if(!HERO_Stretch(0,1500)) return 0;
		if(!taskDelay(700))
		{
			aux4_targetSpeed=0;
			return 0;
		}
//		aux4_targetSpeed=0;
		if(!HERO_Lift(-5000,700)) return 0;//-24000保护前挡板高度,-10000折中高度
		if(!HERO_Grab(grab_limit,200)) return 0;
		HERO_State=HERO_FETCHING_COMPLETE;
		osDelay(600);
		aux4_targetSpeed=0;
	}
	HERO_Order=HERO_STANDBY;
	return 1;
}

uint8_t HERO_Lift(float value, uint32_t time_milis)
{
	float original=aux_motor1_position_target;
	float tmp=(value-original)/time_milis;
	for(uint32_t i=0;i<time_milis+1;i++)
	{
		aux_motor1_position_target=original + i*tmp;
		if(HERO_Order==HERO_STOP)
		{	
			fw_printfln("stop called when lift!");
			return 0;
		}
		osDelay(1);
	}
	uint16_t cnt=0;
	while((GetAuxMotorRealAngle(1)-value) >2000 || (GetAuxMotorRealAngle(1)-value)<-2000)
	{
		if(cnt++>800)
		{
			HERO_Order=HERO_PAUSE;
			fw_printfln("lift not reach! target %f  real %f", value, GetAuxMotorRealAngle(1));
			return 0;
		}
		if(HERO_Order==HERO_STOP)
		{	
			fw_printfln("stop called when lift!");
			return 0;
		}
		osDelay(1);
	}
	return 1;
}
uint8_t HERO_Stretch(float value, uint32_t time_milis)
{
	float original=aux_motor2_position_target;
	//使用5次插值
	//float tmp=(value-original)/time_milis;
	for(uint32_t i=1;i<time_milis+1;i++)
	{
		float a_b=original-value;
		float tmp=((float)i/time_milis);
		float tmp3=tmp*tmp*tmp;
		float tmp4=tmp3*tmp;
		float tmp5=tmp4*tmp;
		aux_motor2_position_target=-6*a_b*tmp5+15*a_b*tmp4-10*a_b*tmp3+original;
		if(HERO_Order==HERO_STOP)
		{	
			fw_printfln("stop called when strech!");
			return 0;
		}
		//fw_printfln("%f",aux_motor2_position_target);
		osDelay(1);
	}
	uint16_t cnt=0;
	while((GetAuxMotorRealAngle(2)-value) >1000 || (GetAuxMotorRealAngle(2)-value)<-1000)
	{
		if(cnt++>500)
		{
			fw_printfln("stretch not reach!target %f  real %f", value, GetAuxMotorRealAngle(2));
			HERO_Order=HERO_PAUSE;
			return 0;
		}
		if(HERO_Order==HERO_STOP)
		{	
			fw_printfln("stop called when strech! ");
			return 0;
		}
		osDelay(1);
	}
	return 1;
}

uint8_t HERO_Grab(float value, uint32_t time_milis)
{
	float original=aux_motor3_position_target;
	float tmp=(value-original)/time_milis;
	for(uint32_t i=0;i<time_milis+1;i++)
	{
		aux_motor3_position_target=original + i*tmp;
		if(HERO_Order==HERO_STOP)
		{	
			return 0;
		}
		osDelay(1);
	}
	return 1;
}

uint8_t taskDelay(uint32_t time_milis)
{
	for(int i=0;i<time_milis;i++)
	{
		if(HERO_Order==HERO_STOP)
		{	
			return 0;
		}
		osDelay(1);
	}
	return 1;
}

void HERO_Recover()
{
	//没有抓取, 不是no move，不是HERO complete
//	if(!HERO_grab_somthing() && HERO_State!=HERO_NO_MOVE && HERO_State!=HERO_COMPLETE)
	if(!HERO_grab_somthing())
	{
		HERO_State=HERO_RECOVERING;
		fw_printfln("Not grabing, Now Recover!");
		//恢复动作
		aux4_targetSpeed=0;
		aux_motor1_position_target=-20000;
		osDelay(1000);
		aux_motor2_position_target=0;
		aux_motor3_position_target=grab_limit;
		osDelay(500);
		aux_motor1_position_target=0;
		osDelay(500);
		//状态
		HERO_State=HERO_NO_MOVE;
	}
	else
	{
		fw_printfln("Grabing. Now recover");
		//
		aux4_targetSpeed=0;
		HERO_Lift(-32000,500);
		HERO_Stretch(aux2_limit,800);
		HERO_Grab(0,100);
		HERO_State=HERO_GRABING;
	}
	HERO_Order=HERO_STANDBY;
}


//void StartNewHEROTask()
//{
//	if(HERO_Order==HERO_STOP)
//	{
//		fw_printfln("Start new task!");
//		HERO_Order=HERO_PREPARE;
////		HERO_State=HERO_NO_MOVE;
//	}
//}



void StartNewFetchTask()
{
	if(HERO_State==HERO_GRABING)
	{
		fw_printfln("Fetch:Grabing Something");
		//爪子有抓取,放回！
		HERO_Order=HERO_REPLACE_STUFF;
	}
	else if(aux_motor1_position_target>-2000 && aux_motor2_position_target>-2000)
	{
		fw_printfln("Fetch: Go to Adjusting");
		//几乎为零位，可以进行作业
		HERO_Order=HERO_ADJUSTDISTANCE_AND_ROTATION;
	}
	else
	{
		fw_printfln("Fetch:Go to PAUSE");
		//其他异常情况
		HERO_Order=HERO_PAUSE;
	}
}

void StopHEROTask()
{
	if(HERO_State!=HERO_GRABING)
	{
		HERO_Order=HERO_STOP;
		HERO_State=HERO_NO_MOVE;
	}
//	HERO_Order=HERO_STOP; 
//		HERO_State=HERO_NO_MOVE;
}

void StartNewPlaceTask()
{
//	if(aux_motor1_position_target<-10000 && aux_motor2_position_target<-20000 && aux_motor3_position_target<1200)
	if(HERO_State==HERO_GRABING)
	{
		fw_printfln("Place:Grabing Something");
		//爪子有抓取
		HERO_Order=HERO_DISCARD_STUFF;
	}
	else if(aux_motor1_position_target>-8000 && aux_motor2_position_target>-2000)
	{
		fw_printfln("Place: Go to Adjusting");
		//几乎为零位，可以进行作业
		HERO_Order=HERO_ADJUSTDISTANCE;
	}
	else
	{
		fw_printfln("Place:Go to PAUSE");
		//其他异常情况
		HERO_Order=HERO_PAUSE;
	}
}

void HERO_Discard_Stuff()
{
	if(HERO_State==HERO_GRABING)
	{
		HERO_State=HERO_DISCASDING;
//		aux_motor1_position_target=-32000;
//		osDelay(500);
//		aux_motor2_position_target=-45000;
//		osDelay(500);
		aux_motor3_position_target=1350;
		osDelay(500);
		aux_motor2_position_target=0;
		aux_motor3_position_target=1350;
		osDelay(800);
		aux_motor1_position_target=-800;
		osDelay(500);
		HERO_State=HERO_NO_MOVE;
	}
	HERO_Order=HERO_STANDBY;
}

//void OrderReplace()
//{
//	if(HERO_Order!=HERO_REPLACE_STUFF)
//		HERO_Order=HERO_REPLACE_STUFF;
//}

void SetFetchHeight(uint8_t HeightIndex)
{
	if(HeightIndex==1)
	{
		if(fetch_height!=-10500)
			fetch_height=-10500;
	}
	else if(HeightIndex==2)
	{
		if(fetch_height!=-1200)
			fetch_height=-1200;
	}
}
///DEBUG////
void HEROPrintState()
{
	switch(HERO_State)
	{
		case HERO_NO_MOVE:
		{
			fw_printfln("no move");
			break;
		}
		case HERO_ADJUSTING:
		{
			fw_printfln("Adjusting");
			break;
		}
		case HERO_DISTANCE_OK:
		{
			fw_printfln("distance ok");
			break;
		}
		case HERO_DISTANCE_AND_ROTATION_OK:
		{
			fw_printfln("distance and rotation ok");
			break;
		}
		case HERO_FETCHING:
		{
			fw_printfln("HERO fetching");
			break;
		}
		case HERO_PLACING:
		{
			fw_printfln("HERO placing");
			break;
		}
		case HERO_FETCHING_COMPLETE:
		{
			fw_printfln("HERO fetch complete");
			break;
		}
		case HERO_PLACING_COMPLETE:
		{
			fw_printfln("HERO place complete");
			break;
		}
		case HERO_DISCASDING:
		{
			fw_printfln("HERO discarding");
			break;
		}
		case HERO_REPLACING:
		{
			fw_printfln("HERO replacing");
			break;
		}
		case HERO_GRABING:
		{
			fw_printfln("grabing");
			break;
		}
		case HERO_MANUAL_PLACING:
		{
			fw_printfln("Manual placing");
			break;
		}
		case HERO_MANUAL_FETCHING:
		{
			fw_printfln("Manual fetching");
			break;
		}
		case HERO_MANUAL_STRETCHED:
		{
			fw_printfln("Manual stretched");
			break;
		}
	}
}
////手动操作程序
//手动放
uint8_t HERO_manual_load()
{
//	ChassisSpeedRef.forward_back_ref=0;
//	ChassisSpeedRef.left_right_ref=0;
//	ChassisSpeedRef.rotate_ref=0;
	//手动模式可以包含HERO_complete
	if(HERO_State==HERO_MANUAL_FETCH)
	{
		fw_printfln("load");
		HERO_State=HERO_MANUAL_LOAD;
		//先抬升,爪子伸开
		if(!HERO_retract(-5000,100)) return 0;
		if(!HERO_Grab(grab_limit,100)) return 0;
		//带传送，送障碍快
		aux4_targetSpeed=35000;
		if(!taskDelay(1500))
		{
			aux4_targetSpeed=0;
			return 0;
		}
		aux4_targetSpeed=0;
		//再下降，抓取
		if(!HERO_Lift(-600,100)) return 0;
		//if(!taskDelay(100)) return 0;
		if(!HERO_Grab(0,200)) return 0;
		if(!taskDelay(100)) return 0;
		//抬升
		if(!HERO_Lift(-32000,300)) return 0;
		//伸出
		if(!HERO_Stretch(aux2_limit,1100)) return 0;
		//if(!taskDelay(500)) return 0;
		//放下
		if(!HERO_Lift(-30000,50)) return 0;
		//状态
		HERO_State=HERO_GRABING;
		HERO_Order=HERO_STANDBY;
	}
	else if(HERO_State==HERO_GRABING)
	{
		fw_printfln("discard!");
		HERO_Order=HERO_DISCARD_STUFF;
	}
	else
	{
		HERO_Order=HERO_STANDBY;
	}
	return 1;
}
//手动取
uint8_t HERO_manual_fetch()
{
	if(HERO_State==HERO_NO_MOVE || HERO_State==HERO_FETCHING_COMPLETE|| HERO_State==HERO_PLACING_COMPLETE)
	{
		fw_printfln("fetch");
		HERO_State=HERO_MANUAL_FETCHING;
		//先抬升
		if(!HERO_Lift(-25000,300)) return 0;
		if(!HERO_Grab(grab_limit,50)) return 0;
		//伸出
		if(!HERO_Stretch(aux2_limit,400)) return 0;
		if(!HERO_Lift(fetch_height-4000,300)) return 0;
		HERO_State=HERO_MANUAL_STRETCHED;
		HERO_Order=HERO_STANDBY;
	}
	else if(HERO_State==HERO_MANUAL_STRETCHED)
	{
		fw_printfln("fetched,now grab!");
		if(!HERO_Lift(fetch_height,300)) return 0;
		if(!taskDelay(200)) return 0;
		if(!HERO_Grab(0,100)) return 0;
		if(!taskDelay(100)) return 0;
		//抬升
		if(!HERO_Lift(-32000,400)) return 0;
		HERO_State=HERO_GRABING;
		HERO_Order=HERO_STANDBY;
	}
	else
	{
		HERO_Order=HERO_STANDBY;
	}
	return 1;
}
//返回现在工程车有没有抓取到什么物体
uint8_t HERO_grab_somthing()
{
	double tmp=GetAuxMotorRealAngle(3);
	if(aux_motor3_position_target<0.01&&aux_motor3_position_target>-0.01)
	{
//		fw_printfln("grab target:%f",aux_motor3_position_target);
		if(tmp>300)
			return 1;
	}
	return 0;
}

uint8_t HERO_belt_back()
{
	HERO_State=HERO_BELT_MOVING;
	aux4_targetSpeed=-35000;
	if(!(taskDelay(500)))
	{
		aux3_targetSpeed=0;
		HERO_Order=HERO_STANDBY;
		HERO_State=HERO_NO_MOVE;
		return 0;
	}
	aux4_targetSpeed=0;
	HERO_Order=HERO_STANDBY;
	HERO_State=HERO_NO_MOVE;
	return 1;
}

uint8_t HERO_belt_forward()
{
	HERO_State=HERO_BELT_MOVING;
	aux4_targetSpeed=35000;
	if(!(taskDelay(500)))
	{
		aux3_targetSpeed=0;
		HERO_Order=HERO_STANDBY;
		HERO_State=HERO_NO_MOVE;
		return 0;
	}
	aux4_targetSpeed=0;
	HERO_Order=HERO_STANDBY;
	HERO_State=HERO_NO_MOVE;
	return 1;
}
	
*/
