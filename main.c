 /*
 * port_main2.c
 *
 *  Created on: 15-Oct-2018
 *      Author: Hrishikesh-derive
 */






#include<msp430.h>
#include <math.h>
#include "port_ui.h"
#include "port_filter.h"





//*****************************************************************************
// Global Definitions for BPM Demo
//*****************************************************************************
#define FOSC 32000000/*Hz*/
//SFR_16BIT(ADC12MEM0);

#define BPM                 0
#define LastResult          1
#define CLOCK               2
#define MAX_STATE           3

#define menu_Start          0
#define menu_PumpAir        1
#define menu_Measuring      2
#define menu_ReleaseAir     3
#define menu_Result         4

#define Valve_OpenClose _LATE7
#define open    0                               //valve open
#define close   1                            //valve close
#define fast    1                             //pump speed fast
#define slow    2                             //pump speed slow
#define Gain_CP 0.09
#define Sample_Window_Interval     60         //100 //60//250// 250
#define pressure_buf_length         512
#define overPressure_limit          200         //in mmHg
#define pulse_peak_limit             800              //919 // 1350    //ADC decimal value
#define pressure_counting_pulse     50                //in mmHg
#define MAP_Sample_Window_Interval  4

//*****************************************************************************
// Global Variables for BPM Demo
//*****************************************************************************
uint16_t pressure_reading0, pressure_reading;
uint16_t ADC1_temp;
uint16_t ADC2_temp, last_reading, ADC2_temp_Max;
uint16_t ADC1_temp_Max;
unsigned int Sample_Window_Counter;
unsigned char overPressure;
unsigned char pulse_counter;
unsigned int Osc_Peak_Buf[pressure_buf_length];
unsigned int CP_Peak_Buf[pressure_buf_length];
unsigned int mx=0;
unsigned char PeakFound;
unsigned int x, xs, xd, xp, xSYS, xDIA;
int  Max_Peak;



unsigned int Systolic_Pressure, Systolic_Pressure0, Systolic_Pressure_last;
unsigned int Diastolic_Pressure, Diastolic_Pressure0, Diastolic_Pressure_last;
unsigned int Heart_Rate, Heart_Rate_last;
unsigned char TMR4_counter;
unsigned int PumpSpeed;
unsigned int MAP;
double ks, kd;
int test=0,test1=0,test2=0;

unsigned char Target_Pressure_Set;
unsigned int MAP_temp, MAP_temp_Max;
unsigned char MAP_Sample_Window_Counter;
unsigned int mx_temp;
uint16_t Target_Pressure, MAP_target;
unsigned char Error;
void BPMDemoMenu(void);



    char goto_sleep;
    char demo_description[60];
    char demo_title[60];
    char state_internal;
    int menu_state;
    char menu_state_max;
    char submenu_state=1;
    char submenu_state_max;

    unsigned char demo_description_delay;
    unsigned char demo_title_delay;

    unsigned int  Fosc;
  //  uint32_t refresh_rate_not_scrolling;
    void (*menu)(void);


 filterLowPass k;
menu_state=1;




/*****************************************************************************
 * Function Name: void pump_air(int spd)
 * Specification: Pumping air into cuff in fast or slow mode
 *****************************************************************************/
void pump_air(int spd)
{
    P2DIR|=BIT5+BIT4;
    P2OUT|=BIT5+BIT4;                    //pump start and valve closed

}

/*****************************************************************************
 * Function Name: void release_air(void)
 * Specification: Releasing air from cuff
 *****************************************************************************/
void release_air(void)
{
    P2OUT&= ~(BIT4+BIT5);         // Open release valve


}




/*****************************************************************************
 * Function Name: unsigned int Calibrate_Pressure (unsigned int value)
 * Specification: Calibrate pressure reading using specific lookup table.
 *                To use the settings in this lookup table the GP_CP must set to 0.06!
 *                This lookup table is checked using Additel 681 Digital Pressure Gauge. It needs to be fine tuned.
 *****************************************************************************/
unsigned int Calibrate_Pressure (unsigned int value)
{
    unsigned int CalibratedPressure;

    if (value <= 5)
            CalibratedPressure = value + 20;
    else if (value <= 25)
            CalibratedPressure = value + 15;



    else if (value <= 40)
            CalibratedPressure = value + 10;
    else if (value <= 70)
            CalibratedPressure = value + 5;
    else if (value <= 95)
            CalibratedPressure = value;
    else if (value <= 115)
            CalibratedPressure = value - 5;
    else if (value <= 135)
            CalibratedPressure = value - 10;
    else if (value <= 160)
            CalibratedPressure = value - 15;
    else if (value <= 185)
            CalibratedPressure = value - 20;
    else if (value <= 210)
            CalibratedPressure = value - 25;
    else if (value <= 235)
            CalibratedPressure = value - 30;
    else
            CalibratedPressure = value - 35;

    return(CalibratedPressure);
}
void initGPIO()
{


  //Button to initiate transfer
  P1DIR &= ~BIT1;                           // Set P1.1 to inpput direction
  P1REN |= BIT1;                            // Enable P1.1 internal resistance
  P1OUT |= BIT1;                            // Set P1.1 as pull-Up resistance
  P1IES |= BIT1;                            // P1.1 Hi/Lo edge
  P1IFG &= ~BIT1;                           // P1.1 IFG cleared
  P1IE |= BIT1;                             // P1.1 interrupt enabled
}
/*****************************************************************************
 * Function Name: void Calculate_BP(void)
 * Specification: Calculate blood pressure and heart rate.
 *****************************************************************************/
void Calculate_BP(void)
{


    float temp_meas1, temp_meas2;
    double Ad, As, tempD, tempD_diff, tempS, tempS_diff;

    // Find the Max of the pulse peaks
    for (x=0; x<mx; x++)
    {
        if (Osc_Peak_Buf[x] > Max_Peak)
        {
            Max_Peak = Osc_Peak_Buf[x];     // MAP (average pressure)
            xp = x;
        }
    }

    MAP = (int)(Max_Peak * Gain_CP);    //average pressure





//  Coefficient lookup table for blood pressure calculation
    if (MAP>200)                //increase ks to lower SYS reading
            ks = 0.93;
    else if (MAP<=200 && MAP>150)
            ks = 0.84;
    else if (MAP<=150 && MAP>135)
            ks = 0.82;
    else if (MAP<=135 && MAP>120)
            ks = 0.81;
    else if (MAP<=120 && MAP>110)
            ks = 0.81;
    else if (MAP<=110 && MAP>70)
            ks = 0.80;
    else if (MAP<=70)
            ks = 0.64;

    if (MAP>180)                //increase kd to increase DIA reading
            kd = 0.95;
    else if (MAP<=180 && MAP>140)
            kd = 0.93;
    else if (MAP<=140 && MAP>120)
            kd = 0.91;
    else if (MAP<=120 && MAP>60)
            kd = 0.91;
    else if (MAP<=60 && MAP>50)
            kd = 0.60;
    else if (MAP<=50)
            kd = 0.50;

    Ad = Max_Peak * kd;                            //dia_osc
    As = Max_Peak * ks;                            //sys_osc

    tempD = fabs(Ad - Osc_Peak_Buf[0]);     //starting comparaing
    xDIA = 0;
    for (xd=1; xd<xp; xd++)
    {
        tempD_diff = fabs(Ad - Osc_Peak_Buf[xd]);
        if (tempD_diff < tempD)
        {
            xDIA = xd;                                       //location of nearest value to Ad
            tempD = tempD_diff;
        }
    }

    tempS = fabs(As - Osc_Peak_Buf[xp+1]);
    xSYS = xp+1;
    for (xs=xp+2; xs<mx; xs++)
    {
        tempS_diff = fabs(As - Osc_Peak_Buf[xs]);
        if ( tempS_diff < tempS)
        {
            xSYS = xs;                                                //location of nearest value to As
            tempS = tempS_diff;
        }
    }


    Systolic_Pressure0 = (int)(CP_Peak_Buf[xSYS-1] * Gain_CP);
    Diastolic_Pressure0 = (int)(CP_Peak_Buf[xDIA-1] * Gain_CP);

    Systolic_Pressure = Calibrate_Pressure(Systolic_Pressure0);
    Diastolic_Pressure = Calibrate_Pressure(Diastolic_Pressure0);

    Systolic_Pressure_last = Systolic_Pressure;     //update last result
    Diastolic_Pressure_last = Diastolic_Pressure;   //update last result
                //update last result
}





/*****************************************************************************
 * Function Name: void BPMDemoMenu(void)
 * Specification: BPM Demo Main Menu:
 *****************************************************************************/


void BPMDemoMenu(void)
{



    switch(submenu_state)
            {


                case menu_PumpAir:
                    mx = 0;                                    // Reset buffer index
                    ADC2_temp_Max = 0;
                    PeakFound = 0;
                    overPressure = 0;
                    pressure_reading = 0;
                    ADC1_temp = 0;
                    ADC2_temp = 0;
                    pulse_counter = 0;
                    Systolic_Pressure = 0;
                    Diastolic_Pressure = 0;
                    Heart_Rate = 0;
                    Target_Pressure_Set = 0;

                    pump_air(fast);                              // Pumping air in fast mode

                    ADC12IE= ADC12IE1;                           // Enable ADC1 interrupt to start collecting ADC data
                    submenu_state = menu_Measuring;

                    break;

                case menu_Measuring:


                    if (PeakFound)
                    {
                        PeakFound = 0;

                    }
                    if (overPressure)                      //If cuff pressure >= 200mmHg then release air
                    {
                        ADC12IE&= ~ADC12IE1;                 //Disable ADC1 interrupt

                        submenu_state = menu_ReleaseAir;
                    }

                    break;

                case menu_ReleaseAir:
                    release_air();
                    ADC12IE&= ~ADC12IE1;                      //Disable ADC1 interrupt

                    submenu_state = menu_Result;
                    break;

                case menu_Result:
                    switch (Error)                                 //Checking to see if any errors occured
                    {
                        case 0:                                   //No error, display blood pressure result

                            break;
                        case 1:                                   //Error1 (over preset pressure limit) occured, display error message

                            break;
                    }
            }

}


/***************************************************************************
****************************************************************************

ADC interrupt subroutine


***************************************************************************
****************************************************************************/

#pragma vector=ADC12_VECTOR
__interrupt void adc_isr()
{

    {

       ADC1_temp = ADC12MEM0;         //Channel1(ADC1) = Cuff pressure
        ADC2_temp = ADC12MEM1;         //Channel2(ADC2) = Extracted oscillation signal


        pressure_reading0 = (int)(ADC1_temp * Gain_CP);                               // mmHg = ADC_value * (3.3V/4096) / 151 DC_gain / 0.0008 KPa * (760 mmHg / 101.325 KPa) = ADC * 0.05

        pressure_reading = Calibrate_Pressure(pressure_reading0);

         //Finding each pulse peak in oscillation signal
        if ( ADC1_temp > 840)
        {


            if (ADC2_temp > ADC2_temp_Max)                                           //Finding each pulse peak in oscillation signal
            {
                ADC2_temp_Max = ADC2_temp;
                ADC1_temp_Max = ADC1_temp;
                Sample_Window_Counter = Sample_Window_Interval;
            }
            else
            {
                Sample_Window_Counter--;                                                  //signal is now going down
                if (Sample_Window_Counter == 0)                                           //no more max peaks detected in the sampling window interval
                {
                    if (ADC2_temp_Max > pulse_peak_limit)                                 //peak value must be greater than specific value to be counted as a pulse
                    {
                        if (mx < pressure_buf_length)                                     //keep buffer from overflow
                        {
                            Osc_Peak_Buf[mx] = ADC2_temp_Max;
                            CP_Peak_Buf[mx] = ADC1_temp_Max;
                            mx++;
                            MAP_temp = ADC2_temp_Max;
                        }

                        ADC2_temp_Max = 0;
                        PeakFound = 1;

                        if (!Target_Pressure_Set)
                        {
                            if (MAP_temp > MAP_temp_Max)
                            {
                                MAP_temp_Max = MAP_temp;
                                mx_temp = mx-1;
                                MAP_Sample_Window_Counter = MAP_Sample_Window_Interval;          //number of pulse peaks needed after peak Max is found
                            }
                            else
                            {
                                MAP_Sample_Window_Counter--;
                                if (MAP_Sample_Window_Counter == 0)                                //found peak max
                                {
                                    MAP_target = (int)(Osc_Peak_Buf[mx_temp] * Gain_CP);
                                    Target_Pressure = MAP_target + 40;                                 //set the pressure limit more 40 mmhg from MAP target pressure

                                   // Target_Pressure = Calibrate_Pressure(Target_Pressure);
                                    Target_Pressure_Set = 1;
                                    MAP_temp_Max = 0;
                                }
                            }
                        }
                    }
                }
            }

            if (Target_Pressure_Set)
            {


                if (pressure_reading >= Target_Pressure)          //If cuff pressure >= Target_Pressure then go to calculate BP and release air
                {
                    overPressure = 1;
                    Error = 0;                                   //no error
                    Calculate_BP();
                }
            }
            else
            {
                if (pressure_reading >= overPressure_limit)    //If cuff pressure >= overPressure_Limit then go to release air and display error message
                {
                    overPressure = 1;
                    Error = 1;                                    //Err1=over preset pressure limit
                }
            }
        }

    }

}

/***************************************************************************
****************************************************************************

adc_init():this function initialize ADC,

***************************************************************************
****************************************************************************/

void adc_init()
{
ADC12CTL0=ADC12SHT0_5 | ADC12ON | ADC12MSC;
ADC12CTL1=ADC12SHP | ADC12CONSEQ_3 + ADC12DIV_0 + ADC12SSEL_1 ;

ADC12MCTL0=ADC12INCH_0;
ADC12MCTL1=ADC12INCH_1 | ADC12EOS;                //select channel 0 and channel 1
P6SEL = 0x0F;



}

/***************************************************************************
****************************************************************************
//port interrupt subroutine
//port1


****************************************************************************
****************************************************************************/


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Port_1 (void)
#else
#error Compiler not supported!
#endif
{

    ADC12IE&= ~ADC12IE1;           //disable ADC interrupt
 P1IFG &= ~BIT1;                   //clear port interupt flag
 P2OUT&=~(BIT4+BIT5);               //stop the pump and open the valve

}





/****************************************************************************
*****************************************************************************
*
* MAIN FUNCTION
*
*****************************************************************************
*****************************************************************************/
int main()
{
    WDTCTL = WDTPW+WDTHOLD;
    UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
    UCSCTL4 |= SELA_2;                        // Set ACLK = REFO
    UCSCTL0 = 0x0000;
    initGPIO();                                 // Set lowest possible DCOx, MODx

    test=2;
    //set the MCLK and SMCLK to 16Mhz
    do
     {
       UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
          test++;
       SFRIFG1 &= ~OFIFG;
     }while (SFRIFG1&OFIFG);
    test=1;
     __bis_SR_register(SCG0 +GIE);
     UCSCTL1 = DCORSEL_5;
     UCSCTL2 |= 487;


     __bic_SR_register(SCG0);


     __delay_cycles(250000);


     adc_init();
     ADC12CTL0 |= ADC12ENC;                    // Enable conversions
       ADC12CTL0 |= ADC12SC;

    //**********************************************************************
    // Initialize Digital Lowpass Filter
    //**********************************************************************
    filterLowPassInit(&k, 500, 5);    // Specify sampling frequency 500Hz and filter cutoff frequency 5Hz
   filterSingle(&k, 1.0);

    //**********************************************************************
    // Main Loop
    //**********************************************************************
    while(1)
    {

        BPMDemoMenu();







    }

    return 0;
}

void LogChar(char ch){}







