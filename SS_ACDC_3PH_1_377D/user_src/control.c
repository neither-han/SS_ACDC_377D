#include "control.h"

extern float inputU_SinCos[4],inputU_angle;
extern PID_Data pid_InputId_Data;
extern PID_Data pid_InputIq_Data;
extern PID_Parameter pid_InputI_Parameter;
extern PID_Data pid_UinPLL_Data;
extern PID_Parameter pid_UinPLL_Parameter;

uint16_t ADC_DATA[10];
float Uin_BC,Uin_AB,Uin_CA,Uin_A,Uin_B,Uin_C,Uin_d,Uin_q;
float Iin_A,Iin_B,Iin_C,Iin_d,Iin_q,Iin_d_hope,Iin_q_hope;
float PID_Forward_Dout,PID_Forward_Qout;
float Vdd_forNormalization=600;
float Forward_D_set,Forward_Q_set,Forward_Dset_normalization,Forward_Qset_normalization;
float U,V,W;
uint16_t U_16bit,V_16bit,W_16bit;


volatile float buffer_d[3][30];
int count_d=0,count_c=0,fla=0;

uint16_t  BingWang;

void User_IRQhander(void)
{
GPIO_writePin(36,1);
    ADC_DATA[0] = ADC_readResult(myADC0_RESULT_BASE,myADC0_SOC2);//A2
    ADC_DATA[1] = ADC_readResult(myADC0_RESULT_BASE,myADC0_SOC3);//A3
    ADC_DATA[2] = ADC_readResult(myADC0_RESULT_BASE,myADC0_SOC4);//A4
    ADC_DATA[3] = ADC_readResult(myADC0_RESULT_BASE,myADC0_SOC5);//A5
    ADC_DATA[4] = ADC_readResult(myADC1_RESULT_BASE,myADC1_SOC0);//B2
    ADC_DATA[5] = ADC_readResult(myADC1_RESULT_BASE,myADC1_SOC1);//B3
    ADC_DATA[6] = ADC_readResult(myADC2_RESULT_BASE,myADC2_SOC0);//C2
    ADC_DATA[7] = ADC_readResult(myADC2_RESULT_BASE,myADC2_SOC1);//C3
    
    Uin_CA=ADC_DATA[1]*0.276f-558.0f;//A3
    Uin_AB=ADC_DATA[3]*0.276f-558.0f;//A5
    Uin_BC=ADC_DATA[7]*0.276f-558.0f;//C3

    Uin_A=(Uin_AB-Uin_CA)/3.0f;
    Uin_B=(Uin_BC-Uin_AB)/3.0f;
    Uin_C=(Uin_CA-Uin_BC)/3.0f;
  
/*
    Iin_A=ADC_DATA[]*44.0f-200.0f;
    Iin_B=ADC_DATA[]*44.0f-200.0f;
    Iin_C=ADC_DATA[]*44.0f-200.0f;
    */
    
      //更新SIN和COS
    Sin_Cos_Update();
    //park
    parkTest_TOW(Uin_A,Uin_B,Uin_C,inputU_SinCos,&Uin_d,&Uin_q);
    parkTest_TOW(Iin_A,Iin_B,Iin_C,inputU_SinCos,&Iin_d,&Iin_q);
    //Uin pll
    pll_test();
    //current loop
    I_pid();
    //FP_park
    F_parkTest_TOW(&U,&V,&W,inputU_SinCos,Forward_Dset_normalization,Forward_Qset_normalization);
    U_16bit=(uint16_t)((U+1.0f)*Epwm_count_Period/2);
    V_16bit=(uint16_t)((V+1.0f)*Epwm_count_Period/2);
    W_16bit=(uint16_t)((W+1.0f)*Epwm_count_Period/2);
    //PWM
    /*User_Waveform_Generation(U_16bit,1000,V,myEPWM2_BASE,myEPWM1_BASE);
    User_Waveform_Generation(V_16bit,1000,W,myEPWM4_BASE,myEPWM3_BASE);
    User_Waveform_Generation(W_16bit,1000,U,myEPWM7_BASE,myEPWM6_BASE);*/
    User_Waveform_Generation(W_16bit,1000,-1,myEPWM2_BASE,myEPWM1_BASE);
    User_Waveform_Generation(V_16bit,1000,-1,myEPWM4_BASE,myEPWM3_BASE);
    User_Waveform_Generation(U_16bit,1000,-1,myEPWM7_BASE,myEPWM6_BASE);
    EPWM_setGlobalLoadOneShotLatch(myEPWM1_BASE);
GPIO_writePin(36,0);
  if(count_c<30)
  {
    count_d++;
  if(count_d>=50)
  {
    buffer_d[0][count_c]=Uin_d;
    buffer_d[1][count_c]=Uin_q;
    buffer_d[2][count_c]=inputU_angle;
    count_d=0;
    count_c++;
  }
  }
}
void I_pid(void)//78Khz
{
   
    if(BingWang)
    {
    //PID
        PID_Forward_Dout=PID_calculate(Iin_d,Iin_d_hope,&pid_InputId_Data,&pid_InputI_Parameter,BingWang);
        PID_Forward_Qout=PID_calculate(Iin_q,Iin_q_hope,&pid_InputIq_Data,&pid_InputI_Parameter,BingWang);
    //feedforward
        Forward_D_set=PID_Forward_Dout+Uin_d-Iin_q*M_PI*2.0f*50.0f*L_grid;
        Forward_Q_set=PID_Forward_Qout+Uin_q+Iin_d*M_PI*2.0f*50.0f*L_grid;
    }
    else 
    {
        Forward_D_set=500;
        Forward_Q_set=0;
    }
    //normalization
    Forward_Dset_normalization=Forward_D_set/Vdd_forNormalization;
    Forward_Qset_normalization=Forward_Q_set/Vdd_forNormalization;
    //limit
    Max_min(&Forward_Dset_normalization,0.97);
    Max_min(&Forward_Qset_normalization,0.97);
}

void  Max_min(float *a,float b)
{
  if(*a>b)
  {
    *a=b;
  }
  else
  {
    if(*a<-b)
      *a=-b;
  }
}


/*
Epwm_base_00ang01
          ^^
          ||
          a b
a->Forward
b->Backward
后级持续低电平：Backward_dutycycle=0
后级持续高电平：Backward_dutycycle>Epwm_count_Period

前级持续高电平：Forward_dutycycle=0；
前级持续高电平：Forward_dutycycle=Epwm_count_Period
*/
void User_Waveform_Generation(uint16_t Forward_dutycycle,
uint16_t Backward_dutycycle,
float Current_direction,
uint32_t Epwm_base_01and11,
uint32_t Epwm_base_00and10)
{
    uint16_t ComValue_A_01and11,ComValue_B_01and11,ComValue_A_00and10,ComValue_B_00and10;
//负电流，电流自电网流向电感
//forword的下降沿与backword的下降沿对齐
    if(Current_direction<0){
        if(Forward_dutycycle>Backward_dutycycle){
            if(Forward_dutycycle<Backward_dutycycle+Epwm_count_DEADtime*2){
                Forward_dutycycle=Backward_dutycycle+Epwm_count_DEADtime*2;
            }
            ComValue_A_01and11=0;
            ComValue_B_01and11=Backward_dutycycle;
            ComValue_A_00and10=Backward_dutycycle;
            ComValue_B_00and10=Backward_dutycycle+(Epwm_count_Period-Forward_dutycycle);
        }
        else {
            if(Forward_dutycycle>Backward_dutycycle-Epwm_count_DEADtime*2){
                Forward_dutycycle=Backward_dutycycle-Epwm_count_DEADtime*2;
            }
            ComValue_A_01and11=Backward_dutycycle-Forward_dutycycle;
            ComValue_B_01and11=Backward_dutycycle;
            ComValue_A_00and10=Backward_dutycycle;
            ComValue_B_00and10=Epwm_count_Free;
            //ComValue_B_00and10=Epwm_count_Period;
        }
        //output set
        //Epwm_base_00and10 OUTPUT_A ——>10
        EPWM_setCounterCompareValue(Epwm_base_00and10, EPWM_COUNTER_COMPARE_A, ComValue_B_00and10);
        //Epwm_base_00and10 OUTPUT_B ——>00
        EPWM_setCounterCompareValue(Epwm_base_00and10, EPWM_COUNTER_COMPARE_B, ComValue_A_00and10);
        //Epwm_base_01and11 OUTPUT_A ——>11
        EPWM_setActionQualifierAction(Epwm_base_01and11, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        EPWM_setActionQualifierAction(Epwm_base_01and11, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
        //Epwm_base_01and11 OUTPUT_B ——>01
        EPWM_setActionQualifierAction(Epwm_base_01and11, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        EPWM_setActionQualifierAction(Epwm_base_01and11, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    }
//正电流，电流自电感流向电网
//forword的上升沿与backword的下降沿对齐
    else {
        if(Forward_dutycycle+Backward_dutycycle>Epwm_count_Period){
            if(Forward_dutycycle+Backward_dutycycle<Epwm_count_Period+Epwm_count_DEADtime*2){
                Forward_dutycycle=Epwm_count_Period-Backward_dutycycle+Epwm_count_DEADtime*2;
            }          
            ComValue_A_01and11=Backward_dutycycle-(Epwm_count_Period-Forward_dutycycle);
            ComValue_B_01and11=Backward_dutycycle;
            ComValue_A_00and10=Backward_dutycycle;
            ComValue_B_00and10=Epwm_count_Free;
        }
        else{
            if(Forward_dutycycle>Epwm_count_Period-Backward_dutycycle-Epwm_count_DEADtime*2){
                Forward_dutycycle=Epwm_count_Period-Backward_dutycycle-Epwm_count_DEADtime*2;
            }
            ComValue_A_01and11=0;
            ComValue_B_01and11=Backward_dutycycle;
            ComValue_A_00and10=Backward_dutycycle;
            ComValue_B_00and10=Forward_dutycycle+Backward_dutycycle;
        }
        //output set
        //Epwm_base_00and10 OUTPUT_A ——>10
        EPWM_setCounterCompareValue(Epwm_base_00and10, EPWM_COUNTER_COMPARE_A, ComValue_A_00and10);
        //Epwm_base_00and10 OUTPUT_B ——>00
        EPWM_setCounterCompareValue(Epwm_base_00and10, EPWM_COUNTER_COMPARE_B, ComValue_B_00and10);
        //Epwm_base_01and11 OUTPUT_A ——>11
        EPWM_setActionQualifierAction(Epwm_base_01and11, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        EPWM_setActionQualifierAction(Epwm_base_01and11, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
        //Epwm_base_01and11 OUTPUT_B ——>01
        EPWM_setActionQualifierAction(Epwm_base_01and11, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);	
        EPWM_setActionQualifierAction(Epwm_base_01and11, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);	
    }
    EPWM_setCounterCompareValue(Epwm_base_01and11, EPWM_COUNTER_COMPARE_A, ComValue_A_01and11);
    EPWM_setCounterCompareValue(Epwm_base_01and11, EPWM_COUNTER_COMPARE_B, ComValue_B_01and11);
}

