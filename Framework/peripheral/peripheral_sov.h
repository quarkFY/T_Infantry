#ifndef _SOV_H_
#define _SOV_H_
#include "gpio.h"
//ǰ�ֵ�ŷ�
#define FRONT_SOV1_ON()  HAL_GPIO_WritePin(FRONT_SOV1_GPIO_Port, FRONT_SOV1_Pin,GPIO_PIN_SET);\
										
#define FRONT_SOV1_OFF()  HAL_GPIO_WritePin(FRONT_SOV1_GPIO_Port, FRONT_SOV1_Pin,GPIO_PIN_RESET);\
//���ֵ�ŷ�
#define FRONT_SOV2_ON()  HAL_GPIO_WritePin(FRONT_SOV2_GPIO_Port, FRONT_SOV2_Pin,GPIO_PIN_SET);\
										
#define FRONT_SOV2_OFF()  HAL_GPIO_WritePin(FRONT_SOV2_GPIO_Port, FRONT_SOV2_Pin,GPIO_PIN_RESET);\
//ȡ��
#define GRIP_SOV_ON()  HAL_GPIO_WritePin(GRIP_SOV_GPIO_Port, GRIP_SOV_Pin,GPIO_PIN_SET);\
										
#define GRIP_SOV_OFF()  HAL_GPIO_WritePin(GRIP_SOV_GPIO_Port, GRIP_SOV_Pin,GPIO_PIN_RESET);\
											
#endif

