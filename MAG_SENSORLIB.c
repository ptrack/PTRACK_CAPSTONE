#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "piconfig.h"			//setting up pins
#include "inc/hw_i2c.h"			//used in i2c
#include "inc/hw_timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"		//used in i2c
#include "sensorlib/i2cm_drv.c"	//used in i2c
#include "sensorlib/i2cm_drv.h"	//used in i2c
#include "sensorlib/lsm303dlhc_mag.c"//used in i2c
#include "sensorlib/lsm303dlhc_mag.h"//used in i2c
#include "sensorlib/hw_lsm303dlhc.h"//used in i2c

typedef struct
{
	float latitude;				//ignore this
	float longitude;
} position;
//prototype functions
void pinconfig (void);		//calls function that configures pins
//void UartInit (void);
//void GpsConfigTX (void);
//position GpsRXData (void);
//float convert_decimal_degrees(float num);
//double azi (float lat1, float lon1, float lat2, float lon2);		//ignore these
//void TimerSetup (void);
//void delay(uint32_t);
//void speedprofile(float,float,float, double);

tI2CMInstance sI2CInst;
volatile bool g_bLSM303DLHCMagDone;

void inthandler (void); //created this function to be called by the interrupt handler

void LSM303DLHCMagCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
	// See if an error occurred.
	if(ui8Status != I2CM_STATUS_SUCCESS)
	{
		// An error occurred, so handle it here if required.				//this is from sensor library
	}
	// Indicate that the LSM303DLHCMag transaction has completed.
	g_bLSM303DLHCMagDone = true;
}


int main(void)
{
	//set clock frequency to 120MHz
	uint32_t freq = SysCtlClockFreqSet(SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_XTAL_25MHZ | SYSCTL_CFG_VCO_480,120000000);
	//variables from sensor library
    float fMag[3];
	tLSM303DLHCMag sLSM303DLHCMag;
	//configure pins for SDA+SCL (done with all oter pins in the pinconfig function
	pinconfig();
	//enable system peripheral and wait until it is ready before moving on
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C7))
		{
		}
	//configure I2C peripheral true for fast data transfers
	I2CMasterInitExpClk(I2C7_BASE,120000000,true);
	//Clear Master Interrupts
	I2CMasterIntClear(I2C7_BASE);
	//declare function pointer to the i2c master interrupt handler
	void (*Handler)(void);
	Handler = inthandler;
	//Did not Enable I2C Interrupts

	//Register an Interrupt handler for I2C Interrupts that are enabled
	I2CIntRegister(I2C7_BASE, (*Handler));
	//Initialize Magnetometer (possibility that param #2 is base address of I2C)
	int maginit = LSM303DLHCMagInit(&sLSM303DLHCMag, &sI2CInst,0x3D,LSM303DLHCMagCallback,0);
	//read continuously
	while (1)
	{
		g_bLSM303DLHCMagDone = false;
		LSM303DLHCMagDataRead(&sLSM303DLHCMag, LSM303DLHCMagCallback, 0);
		while(!g_bLSM303DLHCMagDone)	//gets stuck here
		{
		}
		//
		// Get the new magnetometer readings.
		//
		LSM303DLHCMagDataMagnetoGetFloat(&sLSM303DLHCMag, &fMag[0], &fMag[1], &fMag[2]);
	}

}
void inthandler (void)
{
	I2CMIntHandler(&sI2CInst);
}
