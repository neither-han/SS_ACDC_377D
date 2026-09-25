//#############################################################################
//
// FILE:   empty_sysconfig_main.c
//
// TITLE:  Empty Pinmux Example
//
// Empty SysCfg & Driverlib Example
//
// This example is an empty project setup for SysConfig and Driverlib 
// development.
//
//#############################################################################
//
// 
// $Copyright:
// Copyright (C) 2013-2026 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################


//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "board.h"

//
// User Included Files
//
#include "control.h"

//
// Main
//
volatile uint16_t pDMADest[10];
const void *adc1_DATA_dma=(const void *)pDMADest;

volatile uint16_t pDMA[10]={1,1,1,1,1,1,1,1,1,1};
const void *datafrom=(const void *)pDMA;
void main(void)
{
    // Initializes device clock and peripherals
    //
    Device_init();
    //
    // Configures the GPIO pin as a push-pull output
    //
    Device_initGPIO();
    //
    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();
    //
    // Initializes the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //


    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    Board_init();
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
//    Device_enableAllPeripherals();

    EPWM_clearEventTriggerInterruptFlag(myEPWM1_BASE);
    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM
    GPIO_writePin(23,1);
    GPIO_writePin(89,1);
    init_PID_Parameter();
    init_PID_Data();

	    while(1)
    {
        GPIO_writePin(24,1);
        GPIO_writePin(25,0);
        DEVICE_DELAY_US(1000000);
        GPIO_writePin(24,0);
        GPIO_writePin(25,1);
        DEVICE_DELAY_US(1000000);

    }
    //this is a test for pushing to github.
}


__interrupt void INT_myEPWM1_ISR(void)
{
     User_IRQhander();
    // 结束中断函数
    //
    // Clear INT flag for this timer
    //
    EPWM_clearEventTriggerInterruptFlag(myEPWM1_BASE);
    //
    // Acknowledge this interrupt to receive more interrupts from group 3
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

//
// End of File
//
