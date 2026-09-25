#ifndef __PID_H
#define __PID_H

#include "board.h"
#include "driverlib.h"
#include "math.h"

typedef struct
{
	float Sv;//����ֵ
	float Pv;//ʵ��ֵ
	float Ek_1;
	float Ek_2;
	float Ek;
	float OUT_Single;//��������
  float OUT;//����
	float Pout;
	float Iout; 
	float Dout;
}PID_Data;
typedef struct
{
	float T;//��������
	float Ti;//���ֳ���
	float Td;//΢�ֳ���
	float Kp_P;//ϵ��
	float Kp_I;//ϵ��
	float Kp_D;//ϵ��
  float ADD_LIMIT_UP;//������������
  float ADD_LIMIT_DOWN;//������������
  float OUTPUT_LIMIT_UP;//��������
  float OUTPUT_LIMIT_DOWN;//��������
}PID_Parameter;

float PID_calculate(float NOW,float HOPE,PID_Data* pid_data,PID_Parameter* pid_parameter,uint16_t mode);

void init_PID_Data(void);

void init_PID_Parameter(void);


#endif


