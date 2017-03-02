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
