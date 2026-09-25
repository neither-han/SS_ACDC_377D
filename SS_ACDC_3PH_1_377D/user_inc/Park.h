#ifndef __PARK_H
#define __PARK_H

#include "board.h"
#include "driverlib.h"
#include "math.h"

void parkTest_TOW(float U,float V,float W,float *sincos,float *Xd,float *Xq);
void F_parkTest_TOW(float *U,float *V,float *W,float *sincos,float Xd,float Xq);
void F_crakTest_TOW(float *Alpha,float *Beta,float *sincos,float Xd,float Xq);
  
void SOGI_V(float Vin,float *v,float *vq);
void SOGI_I(float Iin,float *i,float *iq);
void Sin_Cos_Update(void);
#endif 

