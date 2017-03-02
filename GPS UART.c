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