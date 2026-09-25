#include "pid.h"


PID_Data pid_InputId_Data;
PID_Data pid_InputIq_Data;
PID_Parameter pid_InputI_Parameter;

PID_Data pid_UinPLL_Data;
PID_Parameter pid_UinPLL_Parameter;

/*
���ܣ�����ʽPID
������
NOW ��ǰֵ
HOPE ����ֵ
pid_data pid����ֵ
pid_parameter pid����
mode ����ģʽ��0��������1����
*/
float PID_calculate(float NOW,float HOPE,PID_Data* pid_data,PID_Parameter* pid_parameter,uint16_t mode)//
{

  if(mode==1)
  {
   pid_data->Pv=NOW;
    pid_data->Sv=HOPE;
    pid_data->Ek=pid_data->Sv-pid_data->Pv;
    pid_data->Pout=pid_parameter->Kp_P*(pid_data->Ek-pid_data->Ek_1);  
    pid_data->Iout=pid_parameter->Kp_I*pid_data->Ek*pid_parameter->T/pid_parameter->Ti;      
    pid_data->Dout=pid_parameter->Kp_D*pid_parameter->Td*(pid_data->Ek-pid_data->Ek_1-pid_data->Ek_1+pid_data->Ek_2)/pid_parameter->T;
    pid_data->OUT_Single=pid_data->Pout+pid_data->Iout+pid_data->Dout;
  
    if(pid_data->OUT_Single>pid_parameter->ADD_LIMIT_UP)
      pid_data->OUT_Single=pid_parameter->ADD_LIMIT_UP;
    else if(pid_data->OUT_Single<pid_parameter->ADD_LIMIT_DOWN)
      pid_data->OUT_Single=pid_parameter->ADD_LIMIT_DOWN;
    
    pid_data->OUT+=pid_data->OUT_Single;
	
   
    pid_data->Ek_2=pid_data->Ek_1;
    pid_data->Ek_1=pid_data->Ek;
  }
  else
  {
    pid_data->OUT=0;
  }
  
	 if(pid_data->OUT>pid_parameter->OUTPUT_LIMIT_UP)
     pid_data->OUT=pid_parameter->OUTPUT_LIMIT_UP;
    else if(pid_data->OUT<pid_parameter->OUTPUT_LIMIT_DOWN)
      pid_data->OUT=pid_parameter->OUTPUT_LIMIT_DOWN;
  
	return  pid_data->OUT;
}

void init_PID_Data(void)//pid���ݳ�ʼ��
{
  pid_InputId_Data.Sv=0;
	pid_InputId_Data.Pv=0;
	pid_InputId_Data.Ek_2=0;
	pid_InputId_Data.Ek_1=0;
	pid_InputId_Data.Ek=0;
  pid_InputId_Data.OUT=0.015707;//
  
  pid_InputIq_Data.Sv=0;
	pid_InputIq_Data.Pv=0;
	pid_InputIq_Data.Ek_2=0;
	pid_InputIq_Data.Ek_1=0;
	pid_InputIq_Data.Ek=0;
  pid_InputIq_Data.OUT=0.015707;//
  
  pid_UinPLL_Data.Sv=0;
	pid_UinPLL_Data.Pv=0;
	pid_UinPLL_Data.Ek_2=0;
	pid_UinPLL_Data.Ek_1=0;
	pid_UinPLL_Data.Ek=0;
  pid_UinPLL_Data.OUT=2.0f*M_PI*50.0f/25000.0f;//50hz
}

void init_PID_Parameter(void)//pid������ʼ��
{
  pid_InputI_Parameter.Kp_P=0.4f;
	pid_InputI_Parameter.Kp_I=0.10000009f;
	pid_InputI_Parameter.Kp_D=0;
	pid_InputI_Parameter.T=1;//Ms
	pid_InputI_Parameter.Td=2;
	pid_InputI_Parameter.Ti=100;
  pid_InputI_Parameter.ADD_LIMIT_DOWN=-6.0f;
  pid_InputI_Parameter.ADD_LIMIT_UP=6.1f;
  pid_InputI_Parameter.OUTPUT_LIMIT_DOWN=-40.0F;
  pid_InputI_Parameter.OUTPUT_LIMIT_UP=40.0F;
  
  pid_UinPLL_Parameter.Kp_P=-0.000600000028f;
	pid_UinPLL_Parameter.Kp_I=-0.00039999999f;
	pid_UinPLL_Parameter.Kp_D=0;
	pid_UinPLL_Parameter.T=1;//Ms
	pid_UinPLL_Parameter.Td=2;
	pid_UinPLL_Parameter.Ti=100;
  pid_UinPLL_Parameter.ADD_LIMIT_DOWN=-0.001f;
  pid_UinPLL_Parameter.ADD_LIMIT_UP=0.001f;
  pid_UinPLL_Parameter.OUTPUT_LIMIT_DOWN=2.0f*M_PI*40.0f/25000.0f;//40hz//0.015707-<50hz
  pid_UinPLL_Parameter.OUTPUT_LIMIT_UP=2.0f*M_PI*60.0f/25000.0f;//60hz
}

