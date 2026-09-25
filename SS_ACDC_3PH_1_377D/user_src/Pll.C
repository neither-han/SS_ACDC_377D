#include "Pll.h"
#include "pid.h"
#include "Park.h"

extern float inputU_SinCos[4],inputU_angle;
extern PID_Data pid_UinPLL_Data;
extern PID_Parameter pid_UinPLL_Parameter;
extern float Uin_A,Uin_B,Uin_C,Uin_d,Uin_q;

void pll_test(void)//
{
   float U_d_abs,Uin_sig;
  U_d_abs=fabsf(Uin_d);
  if(U_d_abs<10.0f)
  {
    U_d_abs=10.0f;
  }
  
Uin_sig=Uin_q/U_d_abs;
if(Uin_sig>1.0f)
{
  Uin_sig=1.0f;
}
if(Uin_sig<-1.0f)
{
  Uin_sig=-1.0f;
}
  

  //inputU_angle+=PID_calculate(Uin_sig,0,&pid_UinPLL_Data,&pid_UinPLL_Parameter,1);
  inputU_angle+=2.0f*M_PI*50.0f/25000.0f;
  if(inputU_angle>2*M_PI)
  {
    inputU_angle-=2*M_PI;
  }
}




