#include "drivers_uartupper_low.h"
#include "drivers_uartupper_user.h"
#include "utilities_debug.h"
#include "peripheral_define.h"
#include "tasks_upper.h"
#include "usart.h"

NaiveIOPoolDefine(ctrlUartIOPool, {0});
uint8_t data;
uint8_t buf[REC_LEN];
uint16_t RX_STA=0;
void zykReceiveData(uint8_t data);
void ctrlUartRxCpltCallback(){
	//zyk

	zykReceiveData(data);
	HAL_UART_Receive_IT(&CTRL_UART, &data, 1);
	
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
}

void ctrlUartInit(){
	//zyk 一次接收1字节
	if(HAL_UART_Receive_IT(&CTRL_UART, &data, 1) != HAL_OK){
			Error_Handler();
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