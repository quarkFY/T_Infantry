/**
  ******************************************************************************
  * File Name          : drivers_canmotor_low.h
  * Description        : 电机CAN总线驱动函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * CAN总线底层函数
  ******************************************************************************
  */
#ifndef DRIVERS_CANMOTOR_LOW_H
#define DRIVERS_CANMOTOR_LOW_H

#include <can.h>
#include <stdint.h>
#include <cmsis_os.h>

void InitCanReception(void);

void CMGMCanTransmitTask(void const * argument);
void ZGYROCanTransmitTask(void const * argument);

#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;											//�ٶ�
	float ecd_angle;											//�Ƕ�
}Encoder;



#endif
