#ifndef _SOV_H_
#define _SOV_H_
#include "gpio.h"
//前轮1电磁阀
//控制方式，sov1 on、sov2 off最高；sov1 off 、sov2 off中间；sov1 off、sov2 on最低；both on冲突
#define FRONT_SOV1_ON()  HAL_GPIO_WritePin(FRONT_SOV1_GPIO_Port, FRONT_SOV1_Pin,GPIO_PIN_SET);\
										
#define FRONT_SOV1_OFF()  HAL_GPIO_WritePin(FRONT_SOV1_GPIO_Port, FRONT_SOV1_Pin,GPIO_PIN_RESET);\
//前轮2电磁阀
#define FRONT_SOV2_ON()  HAL_GPIO_WritePin(FRONT_SOV2_GPIO_Port, FRONT_SOV2_Pin,GPIO_PIN_SET);\
										
#define FRONT_SOV2_OFF()  HAL_GPIO_WritePin(FRONT_SOV2_GPIO_Port, FRONT_SOV2_Pin,GPIO_PIN_RESET);\
//后轮1电磁阀
#define BEHIND_SOV1_ON()  HAL_GPIO_WritePin(BEHIND_SOV1_GPIO_Port, BEHIND_SOV1_Pin,GPIO_PIN_SET);\
										
#define BEHIND_SOV1_OFF()  HAL_GPIO_WritePin(BEHIND_SOV1_GPIO_Port, BEHIND_SOV1_Pin,GPIO_PIN_RESET);\
//后轮2电磁阀
#define BEHIND_SOV2_ON()  HAL_GPIO_WritePin(BEHIND_SOV2_GPIO_Port, BEHIND_SOV2_Pin,GPIO_PIN_SET);\
										
#define BEHIND_SOV2_OFF()  HAL_GPIO_WritePin(BEHIND_SOV2_GPIO_Port, BEHIND_SOV2_Pin,GPIO_PIN_RESET);\
//取弹
#define GRIP_SOV_ON()  HAL_GPIO_WritePin(GRIP_SOV_GPIO_Port, GRIP_SOV_Pin,GPIO_PIN_SET);\
										
#define GRIP_SOV_OFF()  HAL_GPIO_WritePin(GRIP_SOV_GPIO_Port, GRIP_SOV_Pin,GPIO_PIN_RESET);\

											
#endif

