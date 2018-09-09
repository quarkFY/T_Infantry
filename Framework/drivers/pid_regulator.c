/**
  ******************************************************************************
  * File Name          : pid_regulator.c
  * Description        : PID函数
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * C语言PID函数实现
  ******************************************************************************
  */
#include "pid_regulator.h"

#define MINMAX(value, min, max) value = (value < min) ? min : (value > max ? max : value)
void fw_PID_Reset(fw_PID_Regulator_t *pid){
	
}

void fw_PID_Calc(fw_PID_Regulator_t *pid){
	pid->errorCurr = pid->target - pid->feedback;
	if(pid->SumCount <=500){
		pid->errorSum += pid->target - pid->feedback;
		pid->SumCount++;
		}
	else {
	  pid->errorSum = 0;
		pid->SumCount = 0;
	}
	pid->componentKp = pid->kp * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = pid->ki * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}

//////////////带i，d的pid
void complete_PID_Reset(complete_PID_Regulator_t *pid){
	
}

void complete_PID_Calc(complete_PID_Regulator_t *pid){
	pid->errorCurr = pid->target - pid->feedback;
//	if(pid->SumCount <=500){
//		pid->errorSum += pid->target - pid->feedback;
//		pid->SumCount++;
//		}
//	else {
//	  pid->errorSum = 0;
//		pid->SumCount = 0;
//	}
	//pid->errorSum += pid->target - pid->feedback;
	//MINMAX(pid->errorSum, -1000, 1000);
	pid->error[pid->addr_error] = pid->errorCurr;
	pid->errorSum += pid->errorCurr;
	pid->addr_error = (pid->addr_error + 1)%80;
	pid->errorSum -= pid->error[pid->addr_error];
	pid->componentKp = pid->kp * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = pid->ki * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}

////////////////云台的pid并没有改
extern complete_PID_Regulator_t pitchPositionPID;
extern complete_PID_Regulator_t yawPositionPID;
extern complete_PID_Regulator_t pitchSpeedPID;
extern complete_PID_Regulator_t yawSpeedPID;

int16_t ProcessYawPID(float target, float position_feedback, float velocity_feedback)
{
	return PID_PROCESS_Double(&yawPositionPID,&yawSpeedPID,target,position_feedback,velocity_feedback);
}
int16_t ProcessPitchPID(float target, float position_feedback, float velocity_feedback)
{
	return PID_PROCESS_Double(&pitchPositionPID,&pitchSpeedPID,target,position_feedback,velocity_feedback);
}
int16_t PID_PROCESS_Double(complete_PID_Regulator_t *pid_position,complete_PID_Regulator_t *pid_speed,float target, float position_feedback, float velocity_feedback)
{
	//position		
	pid_position->target = target;
	pid_position->feedback = position_feedback;
	pid_position->Calc(pid_position);
	//speed
	pid_speed->target = pid_position->output;
	pid_speed->feedback = velocity_feedback;
	pid_speed->Calc(pid_speed);
	return pid_speed->output;
}
