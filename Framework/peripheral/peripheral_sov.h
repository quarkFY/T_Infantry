#ifndef _SOV_H_
#define _SOV_H_
#include "gpio.h"
//取弹
#define SOV1_ON()  HAL_GPIO_WritePin(SOV1_GPIO_Port, SOV1_Pin,GPIO_PIN_SET);\
										
#define SOV1_OFF()  HAL_GPIO_WritePin(SOV1_GPIO_Port, SOV1_Pin,GPIO_PIN_RESET);\
//前轮电磁阀
#define SOV2_ON()  HAL_GPIO_WritePin(SOV2_GPIO_Port, SOV2_Pin,GPIO_PIN_SET);\
										
#define SOV2_OFF()  HAL_GPIO_WritePin(SOV2_GPIO_Port, SOV2_Pin,GPIO_PIN_RESET);\
//后轮电磁阀
#define SOV3_ON()  HAL_GPIO_WritePin(SOV3_GPIO_Port, SOV3_Pin,GPIO_PIN_SET);\
										
#define SOV3_OFF()  HAL_GPIO_WritePin(SOV3_GPIO_Port, SOV3_Pin,GPIO_PIN_RESET);\
											
#endif

