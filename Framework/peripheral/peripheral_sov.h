#ifndef _SOV_H_
#define _SOV_H_
#include "gpio.h"
//控制方式，sov1 on、sov2 off最高；sov1 off 、sov2 off中间；sov1 off、sov2 on最低；both on冲突
///////////////////////////////////////////////////////////////////////////
#define FR1_ON()  HAL_GPIO_WritePin(FR1_GPIO_Port, FR1_Pin,GPIO_PIN_SET);\
										
#define FR1_OFF()  HAL_GPIO_WritePin(FR1_GPIO_Port, FR1_Pin,GPIO_PIN_RESET);\

#define FR2_ON()  HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin,GPIO_PIN_SET);\
										
#define FR2_OFF()  HAL_GPIO_WritePin(FR2_GPIO_Port, FR2_Pin,GPIO_PIN_RESET);\
/////////////////////////////////////////////////////////////////////////////////
#define FL1_ON()  HAL_GPIO_WritePin(FL1_GPIO_Port, FL1_Pin,GPIO_PIN_SET);\
										
#define FL1_OFF()  HAL_GPIO_WritePin(FL1_GPIO_Port, FL1_Pin,GPIO_PIN_RESET);\

#define FL2_ON()  HAL_GPIO_WritePin(FL2_GPIO_Port, FL2_Pin,GPIO_PIN_SET);\
										
#define FL2_OFF()  HAL_GPIO_WritePin(FL2_GPIO_Port, FL2_Pin,GPIO_PIN_RESET);\
/////////////////////////////////////////////////////////////////////////////////
#define BR1_ON()  HAL_GPIO_WritePin(BR1_GPIO_Port, BR1_Pin,GPIO_PIN_SET);\
										
#define BR1_OFF()  HAL_GPIO_WritePin(BR1_GPIO_Port, BR1_Pin,GPIO_PIN_RESET);\

#define BR2_ON()  HAL_GPIO_WritePin(BR2_GPIO_Port, BR2_Pin,GPIO_PIN_SET);\
										
#define BR2_OFF()  HAL_GPIO_WritePin(BR2_GPIO_Port, BR2_Pin,GPIO_PIN_RESET);\
/////////////////////////////////////////////////////////////////////////////////
#define BL1_ON()  HAL_GPIO_WritePin(BL1_GPIO_Port, BL1_Pin,GPIO_PIN_SET);\
										
#define BL1_OFF()  HAL_GPIO_WritePin(BL1_GPIO_Port, BL1_Pin,GPIO_PIN_RESET);\

#define BL2_ON()  HAL_GPIO_WritePin(BL2_GPIO_Port, BL2_Pin,GPIO_PIN_SET);\
										
#define BL2_OFF()  HAL_GPIO_WritePin(BL2_GPIO_Port, BL2_Pin,GPIO_PIN_RESET);\

											
#endif

