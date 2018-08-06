#ifndef TI
#define TI
#endif


#include "ext_config.h"
#include "watchtime.h"

/*******************************************************/
//@briefbattery volume detect

/******************************************************/
#ifdef BATCHECK
void volchecksub(void)
{
	volatile char voltage_tempad;

	   voltage_tempad=BatDetect();                           //

	   if (voltage_tempad >= BATTERY_MID)                 //
	   {
		   f_dispbatterylow =1;                  //
		   f_dispbatterymid =1;
		   f_dispbatteryhigh =1;
	   }
	   else if(voltage_tempad <= BATTERY_MID && voltage_tempad >= BATTERY_LOW)
	   {
		   f_dispbatterylow =1;                  //
		   f_dispbatterymid =1;
		   f_dispbatteryhigh =0;
	   }
	   else
	   {
		   f_dispbatterylow =1;                  //
		   f_dispbatterymid =0;
		   f_dispbatteryhigh =0;
	   }

}


//ADC monitor the battery,A7 input;  SINGLE CHANNEL SINGLE CONVERT;
//MCLK SELECT; ADC10OSC start convert
char BatDetect(void){

	volatile unsigned int battery_volume;

	  ADC10CTL0=SREF_1+ADC10SHT_1 + ADC10ON+MSC+REFON;// 1.5V ASREFERENCE VOLTAGE
	  _delay_cycles(150);             // refon will setup 30us
	  ADC10CTL1=INCH_7+ SHS_0+ ADC10DIV_0+ ADC10SSEL_2+ CONSEQ_0; // SINGLE CHANNEL SINGLE CONVERT
	  ADC10AE0|=BIT7;          // CHANNEL5 SELECT
	  P5DIR|=BIT1;              //P5.1 OUTPUT
	  P5OUT&=~BIT1;             // P5.1 RESET

	  ADC10CTL0 |=ENC+ADC10SC;  //start convert

	  while(!(ADC10CTL0&ADC10IFG)); //WAIT FOR THE ADC10MEM IS LOAD
      battery_volume=ADC10MEM;
   /*   ADC10CTL0&=~ADC10IFG;
      ADC10CTL0&=~ENC;
      ADC10CTL1=0x00;   //
      ADC10CTL0=0x00;    //
      ADC10AE0&=~BIT7;
     // P5DIR&=~BIT1;*/

      return battery_volume/10;
}

#endif
