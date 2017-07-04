#include "rtos_init.h"
#include "drivers_platemotor.h"
#include "application_motorcontrol.h"
#include "utilities_debug.h"
#include "drivers_canmotor_low.h"
//#include "drivers_mpu6050_low.h"
#include "peripheral_tim.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartrc_low.h"
#include "tasks_cmcontrol.h"
#include "drivers_imu_low.h"
#include "utilities_tim.h"
#include "tim.h"

#include "drivers_buzzer_low.h"
#include "drivers_uartjudge_low.h"

uint8_t isInited = 0;
void rtos_init(){

	playMusicWhenInit();
	fw_userTimeEnable();
	MPU6500_Init();
	IST8310_Init();
	ctrlUartInit();
	RemoteTaskInit();
	UserTimerInit();
	CMControtLoopTaskInit();
	rcInit();
	motorInit();
	plateMotorInit();
	judgeUartInit();
//	mpu6050Init();
//	Init_Quaternion();
	fw_printfln("init success");
}


