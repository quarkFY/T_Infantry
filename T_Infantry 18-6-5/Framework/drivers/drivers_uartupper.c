/**
  ******************************************************************************
  * File Name          : drivers_uartupper.c
  * Description        : 妙算通信
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * 串口初始化
	* 接收回调函数
	* 自定义组帧协议
  ******************************************************************************
  */
#include "drivers_uartupper_low.h"
#include "drivers_uartupper_user.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "stdint.h"
#include "peripheral_define.h"
//#include "tasks_upper.h"
#include "utilities_debug.h"
#include "usart.h"
#include "rtos_semaphore.h"
#include "tasks_platemotor.h"

#include "tasks_timed.h"
#include "tasks_motor.h"//zy

NaiveIOPoolDefine(ctrlUartIOPool, {0});
uint8_t data;
uint8_t buf[REC_LEN];
uint16_t RX_STA=0;
void zykReceiveData(uint8_t data);
void ctrlUartRxCpltCallback(){
	//zyk
	static portBASE_TYPE xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;

	zykReceiveData(data);
//	HAL_UART_Receive_IT(&CTRL_UART, &data, 1);
	//	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET))  
//	{
//		__HAL_UART_CLEAR_IDLEFLAG(&huart3);  
//    HAL_UART_DMAStop(&huart3);
//		uint32_t temp = huart3.hdmarx->Instance->NDTR;  
//		uint32_t rx_len =  REC_LEN - temp;
//		buf[rx_len]='\0';
//		RX_STA=0x8000;
//		zykProcessData();
//	}
//	HAL_UART_Receive_DMA(&CTRL_UART, buf, REC_LEN);
  HAL_UART_AbortReceive((&MANIFOLD_UART));
	if(HAL_UART_Receive_DMA(&MANIFOLD_UART, &data, 1) != HAL_OK)
	{
		Error_Handler();
		printf( "ManifoldUart error" );
	} 
	if( xHigherPriorityTaskWoken == pdTRUE )
	{
	 portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}


void ctrlUartInit(){
	//zyk 一次接收1字节
	//if(HAL_UART_Receive_IT(&CTRL_UART, &data, 1) != HAL_OK){
	//		Error_Handler();
	//}
	if(HAL_UART_Receive_DMA(&MANIFOLD_UART, &data, 1) != HAL_OK){
		Error_Handler();
		printf( "InitManifoldUart error" );
	} 
//	
//		if(HAL_UART_Receive_DMA(&CTRL_UART, buf, REC_LEN) != HAL_OK){
//			Error_Handler();
//	}
//  	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); 
}





void zykReceiveData(uint8_t data)
{
		if((RX_STA&0x8000)==0)//½ÓÊÕÎ´Íê³É
		{
			if(RX_STA&0x4000)//½ÓÊÕµ½ÁË0x0d
			{
				if(data!=0x0a)
				{
						RX_STA=0;//½ÓÊÕ´íÎó,ÖØÐÂ¿ªÊ¼
				}
				else 
				{
					RX_STA|=0x8000;	//½ÓÊÕÍê³ÉÁË
					buf[RX_LEN]='\0';    //½«Ä©Î²¸³ÖµÎª½áÊø·û
				}
			}
			else //»¹Ã»ÊÕµ½0X0D
			{	
				if(data==0x0d)RX_STA|=0x4000;
				else
				{
					buf[RX_STA&0X3FFF]=data ;
					RX_STA++;
					if(RX_STA>(REC_LEN-1))RX_STA=0;//½ÓÊÕÊý¾Ý´íÎó,ÖØÐÂ¿ªÊ¼½ÓÊÕ	  
				}		 
			}
		}
}








//void vInsert( uint8_t a[ ], uint8_t i, uint8_t n, uint8_t number){
//    for (int j=n;j>i;j--){
//        a[j]=a[j-1];
//        }
//        a[i]=number;
//    if (i==n)
//        a[i]=number;
//}

//void vCheck( uint8_t a[] ){
//	for(uint8_t i = 1; i <= size_frame - 5; i++)
//	{switch ( a[i] ){
//		case byte_SOF    : vInsert( a, i, size_frame, byte_ESCAPE);
//		                 a[i+1] = 0X00; 
//		//fw_printfln("inchange");
//		break;
//		case byte_EOF    : vInsert( a, i, size_frame, byte_ESCAPE);
//		                 a[i+1] = 0X01; //fw_printfln("inchange2");
//		break;		
//		case byte_ESCAPE : vInsert( a, i, size_frame, byte_ESCAPE);
//		                 a[i+1] = 0X02; //fw_printfln("inchange3");
//		break;
//	}}
//}

//void vSendUart(xdata_ctrlUart data){
//	uint8_t tempdata[size_frame] = {0};
//	tempdata[0] = byte_SOF;
//	tempdata[1] = data.dev_yaw >> 8;
//	tempdata[2] = data.dev_yaw & 0x00ff;
//  tempdata[3] = data.dev_pitch >> 8;
//	tempdata[4] = data.dev_pitch & 0x00ff;	
//	tempdata[5] = data.rune;
//	tempdata[6] = data.rune_locate;
//	tempdata[7] = data.target_dis >> 8;
//	tempdata[8] = data.target_dis & 0x00ff;
//	tempdata[9] = data.DLC;
//	tempdata[10] = byte_EOF;
//  vCheck( tempdata );
//	for( uint8_t i = 0; i <= size_frame - 1; i++){
//		fw_printf("%x",tempdata[i]);
//	}
//	printf("\r\n");
//}

//void vDeleteEscape(uint8_t *pData, uint8_t a){
//   while ( a <= size_frame - 2 ){
//	 *(pData + a) = *(pData + a + 1);
//		 a ++;
//	 }
//}

//void vCheckEscape(uint8_t *pData){
//	uint8_t a = 1;
//	while( *pData != byte_EOF && a <= size_frame -1){
//  if( *(pData + a) == byte_ESCAPE ) {
//	//	fw_printfln("in escapecheck");
//		switch ( *(pData + a + 1) ){
//			case 0x00 : *(pData + a + 1) = byte_SOF; break; 
//			case 0x01 : *(pData + a + 1) = byte_EOF; break;
//			case 0x02 : *(pData + a + 1) = byte_ESCAPE; break;
//		}
//		vDeleteEscape( pData, a );
//	}
//	a++;
//  }
//}

//xdata_ctrlUart xUartprocess(uint8_t *pData){
//	xdata_ctrlUart To_return;
//	To_return.Success = 0;
//	uint8_t a = 0; 
//		if ( *pData != byte_SOF ) {
//      To_return.Success = 0;
//			return To_return;
//		}
//		vCheckEscape( pData );
//		for(; a <= size_frame - 1; a++){
//			if(*(pData + a) == byte_EOF) break;
//		}
//		if( *(pData + a ) != byte_EOF || *(pData + a - 1) != a - 2) {
//    To_return.Success = 0; 
//    return To_return;
//		}
//		else To_return.Success = 1;
//		To_return.dev_yaw    = (*(pData + 1) << 8) + *(pData + 2);
//		To_return.dev_pitch  = (*(pData + 3) << 8) + *(pData + 4);
//		To_return.rune       = *(pData + 5);
//		To_return.rune_locate= *(pData + 6);
//		To_return.target_dis = (*(pData + 7) << 8) + *(pData + 8);
//		To_return.DLC = *(pData + a - 1);
//		return To_return;
//}

//Locate_State_e LocateState = Locating;

//void SetLocateState(Locate_State_e v){
//	LocateState = v;
//}
//Locate_State_e GetLocateState(void){
//	return LocateState;
//}
//Rune_State_e RuneState = WAITING;

//void SetRuneState(Rune_State_e v){
//	RuneState = v;
//}
//Rune_State_e GetRuneState(void){
//	return RuneState;
//}

//float dis_yaw = 9.7;
//float dis_pitch = 15;//5.33;
//float location_center_yaw = 0;
//float location_center_pitch = 0;



