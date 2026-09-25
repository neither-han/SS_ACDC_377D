#ifndef _CONTRUL_H
#define _CONTRUL_H
//system include
#include "board.h"
#include "driverlib.h"
#include "math.h"
//user include
#include "Pll.h"
#include "pid.h"
#include "Park.h"

#define Epwm_count_DEADtime 10
#define Epwm_count_Period   2000
#define Epwm_count_Free     2100

#define L_grid 290e-6

void User_IRQhander(void);
void User_Waveform_Generation(uint16_t Forward_dutycycle,
uint16_t Backward_dutycycle,
float Current_direction,
uint32_t Epwm_base_01and11,
uint32_t Epwm_base_00ang10);
void I_pid(void);
void  Max_min(float *a,float b);

#endif
