#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/fpu.h"
#include "driverlib/uart.h"
#include "piconfig.h"
#define PI 3.1415926535897931
#define PI_Degrees 180
#define RAD2DEG 57.2957795130823209
// define structure to pass lat/long arrays
typedef struct
{
	float latitude;
	float longitude;
} position;
//prototype functions
void pinconfig (void);
void UartInit (void);
void GpsConfigTX (void);
position GpsRXData (void);
float convert_decimal_degrees(float num);
double azi (float lat1, float lon1, float lat2, float lon2);
void TimerSetup (void);
void delay(uint32_t);
void speedprofile(float,float,float, double);

int main(void)
{
	// configure pins for alternate uses
	pinconfig();
	// Initialize/Configure uc UART module
	UartInit();
	// send configure messages to GPS
	GpsConfigTX();
	// enable floating point unit for use with lat/long
	FPUEnable();
	//read,parse and save lat/long from gps to structure
	position base = GpsRXData();
	double angle = azi(base.latitude,base.longitude,30.4435167,-95.334383);
//	double hardangle = 78;
	while(1)
	{
		float fastbase = 10;
		float fastv = 150;
		float fastacc = 15;
		TimerSetup ();

		speedprofile(fastbase, fastv, fastacc, angle);
	}
}


////-----------------------------------------------------------------------------------------------
void pinconfig(void)
{
	    // Enable Peripheral Clocks
	    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
	    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);
	    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
	    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

	    //
	    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_4);
	    //
	    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_6);
	    // Enable pin PA4 for GPIOOutput
	    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
	    // Enable pin PA5 for GPIOOutput
	    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
	    // Enable pin PA0 for GPIOOutput
	    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_0);
	    // Enable pin PA1 for GPIOOutput
	    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_1);
	    // Enable pin PA2 for GPIOOutput
	    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);
	    // Enable pin PA3 for GPIOOutput
	    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	    // Enable pin PF0 for GPIOOutput
	    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
	    // Enable pin PD0 for I2C7 I2C7SCL
	    MAP_GPIOPinConfigure(GPIO_PD0_I2C7SCL);
	    MAP_GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
	    // Enable pin PD1 for I2C7 I2C7SDA
	    MAP_GPIOPinConfigure(GPIO_PD1_I2C7SDA);
	    MAP_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);
	    // Enable pin PK0 for UART4 U4RX
	    MAP_GPIOPinConfigure(GPIO_PK0_U4RX);
	    MAP_GPIOPinTypeUART(GPIO_PORTK_BASE, GPIO_PIN_0);
	    // Enable pin PK1 for UART4 U4TX
	    MAP_GPIOPinConfigure(GPIO_PK1_U4TX);
	    MAP_GPIOPinTypeUART(GPIO_PORTK_BASE, GPIO_PIN_1);
		//congifure system frequency to use 25 MHz external oscillator and PLL to set system frequency to 120 MHz
		uint32_t freq = SysCtlClockFreqSet(SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_XTAL_25MHZ | SYSCTL_CFG_VCO_480,120000000);
}
////-----------------------------------------------------------------------------------------------
void UartInit (void)
{
	//congifure system frequency to use 25 MHz external oscillator and PLL to set system frequency to 120 MHz
	uint32_t freq = SysCtlClockFreqSet(SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_XTAL_25MHZ | SYSCTL_CFG_VCO_480,120000000);
	//enable system peripheral uart 4
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART4))
	{
	}
	// Set the clock that uart uses to the 120 MHz System CLock
	UARTClockSourceSet(UART4_BASE, UART_CLOCK_SYSTEM);
	//Set Config of UART
	UARTConfigSetExpClk(UART4_BASE, 120000000, 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}
////-----------------------------------------------------------------------------------------------
void GpsConfigTX (void)
{
	char nmeaconfig[16] = {0xA0,0xA1,0x00,0x09,0x08,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x0D,0x0A};
	uint8_t count;
	for(count = 0 ; count <16 ; count++)
	{
		UARTCharPut(UART4_BASE,nmeaconfig[count]);
		while(!UARTSpaceAvail(UART4_BASE))
		{
		}
	}
	SysCtlDelay(120000000);
}
////-----------------------------------------------------------------------------------------------
position GpsRXData (void)
{
	char rec[160];
	char lat[9];
	char lon[9];
	float latavg [5];
	float longavg[5];
	float latitude;
	float longitude;
	uint8_t i,y,x;
	// clear RX FIFO OE Errors before reading
	UARTRxErrorClear(UART4_BASE);
	//read uart and save into array
	y=0;
	for (y=0; y<5; y++)
	{
		//read in one byte at a time
		for(i = 0; i < 160 ; i++)
			{
				rec[i] = UARTCharGet(UART4_BASE);
			}
		i=0;
		x=0;
		// scan for beggining of message
		while(i<80)
			{
		if(rec[i] == '$' && i<=80)
		{
			if(rec[i+1]=='G')
			{
				if(rec[i+2] =='P')
				{
					if(rec[i+3]=='G')
					{
						if (rec[i+4]=='G')
						{
							if(rec[i+5] =='A')
							{
								x =i;
								uint8_t count;
								// pull latitude and longitude from sentence
								for (count = 0; count<9; count++)
								{
									lat[count] = rec[x+18];
									lon[count] = rec[x+31];
									x++;
								}
								if (fabs((atof(lat)/100)) <1)
								{
									goto nogo;
								}
								if ((atof(lat)/ 100) >180)
								{
									goto nogo;
								}
								if (fabs((atof(lon)/100)) <1)
								{
									goto nogo;
								}
								if ((atof(lon)/ 100) >180)
								{
									goto nogo;
								}
								//convert to usable float number
								latavg[y] = atof(lat)/100;
								longavg[y] = atof(lon)/100;
								if (rec[i+29] == 'S')
								{
									latavg[y] = -latavg[y];
								}
								if(rec[i+41] == 'W')
								{
									longavg[y]= -longavg[y];
								}
								i=99;
							}
							else
									{
										nogo: i++;
									}

						}
						else
								{i++;}

					}
					else
							{i++;}

				}
				else
						{i++;}

			}
			else
					{i++;}
		}
		else
		{i++;}

			}
	}
		float sum =0;
		y=5;
		for (i=0; i<y; i++)
		{
			sum = sum + latavg[i];
		}
		latitude = sum/y;
		sum=0;
		for (i=0; i<y; i++)
		{
			sum = sum + longavg[i];
		}
		longitude = sum/y;
		position station = {latitude,longitude};
		return station;
	}
////-----------------------------------------------------------------------------------------------
float convert_decimal_degrees (float num)
{
	float num1 = fabs(num);
	int deg = num1;
	float mind = (((num1-deg)*100)/60);
	float decdeg = deg+mind;
	if (num <1)
	{
		decdeg=-decdeg;
	}
	decdeg= ((decdeg*PI)/PI_Degrees);
	return decdeg;
}
////-----------------------------------------------------------------------------------------------
double azi (float lat1, float lon1, float lat2, float lon2)
{
	float declat1 = convert_decimal_degrees (lat1);
	float declon1 = convert_decimal_degrees (lon1);
	float declat2 = convert_decimal_degrees (lat2);
	float declon2 = convert_decimal_degrees (lon2);

	double londif = declon2-declon1;
	float theta = atan2((sin(londif)*cos(declat2)),(cos(declat1)*sin(declat2)-(sin(declat1)*cos(declat2)*cos(londif))));
	theta = RAD2DEG*theta;
	return theta;
}
//-----------------------------------------------------------------------------------------------
void TimerSetup (void)
{
	//ENABLE TIMER PERIPHERAL
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	//WAIT FOR PERIPHERAL TO COME UP
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
	//SET CLOCK SOURCE FOR THE TIMER
	TimerClockSourceSet(TIMER0_BASE,TIMER_CLOCK_SYSTEM);
	//CONFIGURE TIMER FOR PERIODIC WITH LOAD COUNT OF 120000000
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE,TIMER_A, 120000000);
	TimerEnable(TIMER0_BASE, TIMER_A);

	return;
}
//-----------------------------------------------------------------------------------------------
void delay (uint32_t pvalue)
{
	uint32_t i = 0;
	//Load timer with 120e6
	TimerLoadSet(TIMER0_BASE,TIMER_A, 120000000);
	//get start time
	uint32_t starttime = TimerValueGet(TIMER0_BASE, TIMER_A);
	//delay based on pvalue clock cycles
	while (starttime-TimerValueGet(TIMER0_BASE, TIMER_A)< pvalue)
	{
		i++;
	}
	return;
}
//-----------------------------------------------------------------------------------------------
void speedprofile(float vo, float v, float a, double angle)
{
	//enable (active low)
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_0 ,0x00);
	//ms1
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1 ,0xFF);
	//ms2
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2 ,0x00);
	//ms3
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3 ,0x00);
	//set direction (0=CW 1=CCW)
	GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5 ,0x00);
	//enable floating point unit
	FPUEnable();
	//Set Timer Frequency (Count of Timer Ticks per second)
	float f = 120000000;
	// Calculate  acceleration/deceleration distance (iterations of formula before engaging slew speed) S= (v^2-vo^2)/(2*a)
	uint32_t S = ((v*v)-(vo*vo))/(2*a);
	uint32_t s = S;
	//initial step delay (timer ticks)
	float p1 = f/(sqrt((vo*vo)+(2*a)));
	//slew speed delay (timer ticks)
	float pS = f/v;
	//acc/dec multiplier
	float R = a/(f*f);
	//calculate slew steps based on desired angle change
	uint32_t slewsteps = ((angle/.094)-(2*S)+40)*2;
	//initial steps*
	int i = 10;
	while (i>0)
		{
			delay(p1/2);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 ,0xFF);
			delay(p1/2);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 ,0x00);
			i--;
		}
	//acceleration
	float pi = p1;
	while(S>0)
		{
			float p = pi*(1+(-R*pi*pi)+(-R*pi*pi)*(-R*pi*pi));
			delay(p/2);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 ,0xFF);
			delay(p/2);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 ,0x00);
			pi=p;
			S--;
		}
	//slew steps
	while(slewsteps>0)
		{
			delay(pS);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 ,0xFF);
			delay(pS);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 ,0x00);
			i--;
		}
	pi=pS;
	//deceleration
	while(s>0)
		{
			float p = pi*(1+(R*pi*pi)+(R*pi*pi)*(R*pi*pi));
			delay(p/2);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 ,0xFF);
			delay(p/2);
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 ,0x00);
			pi=p;
			s--;
		}
	return;
}







