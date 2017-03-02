//*****************************************************************************
// Copyright (c) 2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
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
//
// This file was automatically generated by the Tiva C Series PinMux Utility
// Version: 1.0.4
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "piconfig.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"

//*****************************************************************************
void
PortFunctionInit(void)
{
    //
    // Enable Peripheral Clocks 
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

    //
    // Enable pin PK6 for EPHY0 EN0LED1
    //
    MAP_GPIOPinConfigure(GPIO_PK6_EN0LED1);
    MAP_GPIOPinTypeEthernetLED(GPIO_PORTK_BASE, GPIO_PIN_6);

    //
    // Enable pin PK4 for EPHY0 EN0LED0
    //
    MAP_GPIOPinConfigure(GPIO_PK4_EN0LED0);
    MAP_GPIOPinTypeEthernetLED(GPIO_PORTK_BASE, GPIO_PIN_4);

    //
    // Enable pin PA4 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);

    //
    // Enable pin PA5 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);

    //
    // Enable pin PA0 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_0);

    //
    // Enable pin PA1 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_1);

    //
    // Enable pin PA2 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);

    //
    // Enable pin PA3 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);

    //
    // Enable pin PF0 for GPIOOutput
    //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);

    //
    // Enable pin PD0 for I2C7 I2C7SCL
    //
    MAP_GPIOPinConfigure(GPIO_PD0_I2C7SCL);
    MAP_GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);

    //
    // Enable pin PD1 for I2C7 I2C7SDA
    //
    MAP_GPIOPinConfigure(GPIO_PD1_I2C7SDA);
    MAP_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Enable pin PK0 for UART4 U4RX
    //
    MAP_GPIOPinConfigure(GPIO_PK0_U4RX);
    MAP_GPIOPinTypeUART(GPIO_PORTK_BASE, GPIO_PIN_0);

    //
    // Enable pin PK1 for UART4 U4TX
    //
    MAP_GPIOPinConfigure(GPIO_PK1_U4TX);
    MAP_GPIOPinTypeUART(GPIO_PORTK_BASE, GPIO_PIN_1);
}
