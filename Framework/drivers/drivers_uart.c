/**
  ******************************************************************************
  * File Name          : drivers_uart.c
  * Description        : 电机串口驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 串口接收终端回调函数：遥控器
	* 妙算通讯
	* 裁判系统读取
  ******************************************************************************
  */
#include <usart.h>
#include "drivers_uart.h"
#include "drivers_uartrc_low.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartjudge_low.h"
#include "peripheral_define.h"
#include "utilities_debug.h"
#include "drivers_uartupper_low.h"
#include "drivers_uartjudge_low.h"
#include "drivers_uartgyro_low.h"

/********************所有串口接收中断****************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &RC_UART){
		//遥控器
		rcUartRxCpltCallback();
		
	}else if(UartHandle == &huart3){
		//妙算通信串口
		//自定义协议
//		fw_printfln("manifold get!!!");
//		ctrlUartRxCpltCallback();
		
		manifoldUartRxCpltCallback();
	}
	else if(UartHandle == &JUDGE_UART){
		//裁判系统读取采用单字节阻塞接收方式
		//比赛剩余时间
		//血量
		//底盘电压、电流
		//能量槽*****重要，超功率掉血
		judgeUartRxCpltCallback();
	}
	else if(UartHandle == &GYRO_UART){
		gyroUartRxCpltCallback();
	}
}   
