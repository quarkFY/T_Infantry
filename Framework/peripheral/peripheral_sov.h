#ifndef _SOV_H_
#define _SOV_H_
#include "gpio.h"
//ȡ��
#define SOV1_ON()  HAL_GPIO_WritePin(SOV1_GPIO_Port, SOV1_Pin,GPIO_PIN_SET);\
										
#define SOV1_OFF()  HAL_GPIO_WritePin(SOV1_GPIO_Port, SOV1_Pin,GPIO_PIN_RESET);\
//ǰ�ֵ�ŷ�
#define SOV2_ON()  HAL_GPIO_WritePin(SOV2_GPIO_Port, SOV2_Pin,GPIO_PIN_SET);\
										
#define SOV2_OFF()  HAL_GPIO_WritePin(SOV2_GPIO_Port, SOV2_Pin,GPIO_PIN_RESET);\
//���ֵ�ŷ�
#define SOV3_ON()  HAL_GPIO_WritePin(SOV3_GPIO_Port, SOV3_Pin,GPIO_PIN_SET);\
										
#define SOV3_OFF()  HAL_GPIO_WritePin(SOV3_GPIO_Port, SOV3_Pin,GPIO_PIN_RESET);\
											
#endif

