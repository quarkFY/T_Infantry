#ifndef _SOV_H_
#define _SOV_H_
#include "gpio.h"
//ǰ��1��ŷ�
//���Ʒ�ʽ��sov1 on��sov2 off��ߣ�sov1 off ��sov2 off�м䣻sov1 off��sov2 on��ͣ�both on��ͻ
#define FRONT_SOV1_ON()  HAL_GPIO_WritePin(FRONT_SOV1_GPIO_Port, FRONT_SOV1_Pin,GPIO_PIN_SET);\
										
#define FRONT_SOV1_OFF()  HAL_GPIO_WritePin(FRONT_SOV1_GPIO_Port, FRONT_SOV1_Pin,GPIO_PIN_RESET);\
//ǰ��2��ŷ�
#define FRONT_SOV2_ON()  HAL_GPIO_WritePin(FRONT_SOV2_GPIO_Port, FRONT_SOV2_Pin,GPIO_PIN_SET);\
										
#define FRONT_SOV2_OFF()  HAL_GPIO_WritePin(FRONT_SOV2_GPIO_Port, FRONT_SOV2_Pin,GPIO_PIN_RESET);\
//����1��ŷ�
#define BEHIND_SOV1_ON()  HAL_GPIO_WritePin(BEHIND_SOV1_GPIO_Port, BEHIND_SOV1_Pin,GPIO_PIN_SET);\
										
#define BEHIND_SOV1_OFF()  HAL_GPIO_WritePin(BEHIND_SOV1_GPIO_Port, BEHIND_SOV1_Pin,GPIO_PIN_RESET);\
//����2��ŷ�
#define BEHIND_SOV2_ON()  HAL_GPIO_WritePin(BEHIND_SOV2_GPIO_Port, BEHIND_SOV2_Pin,GPIO_PIN_SET);\
										
#define BEHIND_SOV2_OFF()  HAL_GPIO_WritePin(BEHIND_SOV2_GPIO_Port, BEHIND_SOV2_Pin,GPIO_PIN_RESET);\
//ȡ��
#define GRIP_SOV_ON()  HAL_GPIO_WritePin(GRIP_SOV_GPIO_Port, GRIP_SOV_Pin,GPIO_PIN_SET);\
										
#define GRIP_SOV_OFF()  HAL_GPIO_WritePin(GRIP_SOV_GPIO_Port, GRIP_SOV_Pin,GPIO_PIN_RESET);\

											
#endif

