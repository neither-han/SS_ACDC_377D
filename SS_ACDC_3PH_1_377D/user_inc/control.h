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
#define M_PI_f32 3.14159f
#define kk M_PI*2.0f*50.0f*L_grid

typedef struct
{
    uint32_t ADC_Sum[10];
	float ADC_Calibration[10];
    uint16_t Sum_number;
    uint16_t adc_number;
}ADC_calibration_parameters;

void User_IRQhander(void);

void User_Waveform_Generation(uint16_t Forward_dutycycle,
uint16_t Backward_dutycycle,
float Current_direction,
int* current_num,
uint32_t Epwm_base_01and11,
uint32_t Epwm_base_00ang10);

void I_pid(void);

void  Max_min(float *a,float b);

void control_state_machine(void);

uint16_t ADC_calibration_function(uint16_t* ADC_Data,ADC_calibration_parameters* parameters);
void ADC_calibration_init(void);
#endif
