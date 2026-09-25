#include "Park.h"
#include "stdio.h"
#include "string.h"

#define Ts 5e-5  //��ɢʱ��
#define w 100.0f*M_PI  //����Ƶ��
#define k 1.0f  //ϵ��
//inputU_SinCos[0]->sin;inputU_SinCos[1]->cos;
//inputU_SinCos[2]->cos-2*M_PI/3;inputU_SinCos[3]->cos-4*M_PI/3;
volatile float inputU_SinCos[4],inputU_angle;

void Sin_Cos_Update(void)   //
{
  inputU_SinCos[0]=sinf(inputU_angle);
  inputU_SinCos[1]=cosf(inputU_angle);
  inputU_SinCos[2]=cosf(inputU_angle-2*M_PI/3);
  inputU_SinCos[3]=cosf(inputU_angle-4*M_PI/3);
}

void SOGI_V(float Vin,float *v,float *vq)
{
  static float s1=0,s2=0;
  s1+=(w*((Vin-s1)*k-s2*w))*Ts;
  s2+=s1*Ts;
  *vq=s2*w;
  *v=s1;
}
void SOGI_I(float Iin,float *i,float *iq)
{
  static float s1=0,s2=0;
  s1+=(w*((Iin-s1)*k-s2*w))*Ts;
  s2+=s1*Ts;
  *iq=s2*w;
  *i=s1;
}

void parkTest_TOW(float U,float V,float W,float *sincos,float *Xd,float *Xq)

{
  float Xa,Xb;
  //UVW>ab;
  Xa=(2.0F*U-V-W)/3.0F;
  Xb=(V-W)*0.57735f;
  //ab>dq
  *Xd=Xa*sincos[1]+Xb*sincos[0];//0sin//1cos
  *Xq=-Xa*sincos[0]+Xb*sincos[1];

}
void F_parkTest_TOW(float *U,float *V,float *W,float *sincos,float Xd,float Xq)
{
  float Xa,Xb;
  //dq>ab
  Xa=Xd*sincos[1]-Xq*sincos[0];
  Xb=Xd*sincos[0]+Xq*sincos[1];
  //ab>UVW
  *U=Xa;
  *V=(-Xa+1.73205F*Xb)/2;
  *W=(-Xa-1.73205F*Xb)/2;
}
void F_crakTest_TOW(float *Alpha,float *Beta,float *sincos,float Xd,float Xq)
{
  //dq>ab
  *Alpha=Xd*sincos[1]-Xq*sincos[0];
  *Beta=Xd*sincos[0]+Xq*sincos[1];
}
void crakTest_TOW(float Alpha,float Beta,float *sincos,float *Xd,float *Xq)
{
   //ab>dq
  *Xd=Alpha*sincos[1]+Beta*sincos[0];
  *Xq=-Alpha*sincos[0]+Beta*sincos[1];
}
