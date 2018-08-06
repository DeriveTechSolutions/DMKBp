//#include <msp430.h>
#include "msp430f6638.h"
#include "conf.h"
#include "key.h"
#include "display.h"
#include "UCB0_I2C.h"
#include "watchtime.h"
#include "memoryMode.h"
#include "measureMode.h"
#include "UCA0_UART.h"


#define LOW_AIRIN         1200    // slow air in value of timerA0
#define HIGH_AIRIN        2500    //quick air in value for timerA0
#define MAX_PRE           3152    //equal to ???mmHg
#define LowToHigh_PRE     1083    // equal to 40mmHg
#define senserNoise       692     // reading without load
#define zreoPressure      319     // ADC reading when pressure is zero
#define GAIN              1.62     // gain of amplifier
//#define E2PROMTEST
#define UART
//#define UCS

#ifdef UCS
#define st(x)      do { x } while (__LINE__ == -1)
/* Select source for MCLK and SMCLK e.g. SELECT_MCLK_SMCLK(SELM__DCOCLK + SELS__DCOCLK) */
#define SELECT_MCLK_SMCLK(sources) st(UCSCTL4 = (UCSCTL4 & ~(SELM_7 + SELS_7)) | (sources);)
#endif

void MCUinit(void);
void varInit(void);
void OSCSet (void);
void Init_FLL_Settle(unsigned int fsystem, unsigned int ratio);
void Init_FLL(unsigned int fsystem, unsigned int ratio);
void lowPowerInit(void);

#ifdef E2PROMTEST
void E2PROMWrite(void);   // only for test
unsigned char test;
unsigned char StartAddress1[2];
#endif

char test123=0;
//-----------------------------------------------------------------------------
int _system_pre_init(void)
    {
    /* Insert your low-level initializations here */
    WDTCTL = WDTPW + WDTHOLD; // Stop Watchdog timer
    /*==================================*/
    /* Choose if segment initialization */
    /* should be done or not. */
    /* Return: 0 to omit initialization */
    /* 1 to run initialization */
    /*==================================*/
    return (1);
    }


void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
#ifdef UCS
    // Set system clock to max (25MHz)
    Init_FLL_Settle(25000, 762);
#else
    OSCSet();                   //Init OSC, MCLK, SMCLK =4MHZ,  ACLK =32K
#endif
    lowPowerInit();
    I2CInit();                  //init for E2prom
#ifdef E2PROMTEST
    E2PROMWrite();              //just  for test, write data to e2prom
#endif

    MCUinit();                  // Init RTC_B, LCD , button,and 15ms timer
    SET_LCD_OFF();
    varInit();
    displayDate(month,day);     // display RTC every time start the project
    displayTime(hour,minute);
    displaySetmem();

#ifdef UART
    sendVar_init();
    UARTInit();
#endif

    __bis_SR_register(LPM0_bits +GIE);
	
while(1) {

	   if(ms15Flag==1){                                     //scan key value every 15 ms
		  keyJudge();                                       // Judge value of key and change workMode/workStatus
		  ms15Flag=0;
	   }
  switch (workMode)     {
     case STANDBY:                                        // standby_mode---for RTC DISPLAY only
 	     displayStandby();
          __bis_SR_register(LPM0_bits +GIE);
 	    break;
     case DATESET:                                        // dateset_mode
 	     displayDateset();
          __bis_SR_register(LPM0_bits +GIE);
 	    break;
     case MEASURE:                                        // measure_mode
 	     displayMeasure();
 	    break;
     case MEMORY :                                        // memory_mode
 	     displayMemory();
         __bis_SR_register(LPM0_bits +GIE);              // can wake up from button and RTC
 	    break;
  }

 }   //close while

}

#ifdef UCS
void Init_FLL_Settle(unsigned int fsystem, unsigned int ratio)
{
  volatile unsigned int x = ratio * 32;

  Init_FLL(fsystem, ratio);

  while (x--) {
   __delay_cycles(30);
  }
}

void Init_FLL(unsigned int fsystem, unsigned int ratio)
{
  unsigned int d, dco_div_bits;
  unsigned int mode = 0;

  // Save actual state of FLL loop control, then disable it. This is needed to
  // prevent the FLL from acting as we are making fundamental modifications to
  // the clock setup.
  unsigned int srRegisterState = __get_SR_register() & SCG0;
  __bic_SR_register(SCG0);

  d = ratio;
  dco_div_bits = FLLD__2;        // Have at least a divider of 2

  if (fsystem > 16000) {
    d >>= 1 ;
    mode = 1;
  }
  else {
    fsystem <<= 1;               // fsystem = fsystem * 2
  }

  while (d > 512) {
    dco_div_bits = dco_div_bits + FLLD0;  // Set next higher div level
    d >>= 1;
  }

  UCSCTL0 = 0x0000;              // Set DCO to lowest Tap

  UCSCTL2 &= ~(0x03FF);          // Reset FN bits
  UCSCTL2 = dco_div_bits | (d - 1);

  if (fsystem <= 630)            //           fsystem < 0.63MHz
	UCSCTL1 = DCORSEL_0;
  else if (fsystem <  1250)      // 0.63MHz < fsystem < 1.25MHz
	UCSCTL1 = DCORSEL_1;
  else if (fsystem <  2500)      // 1.25MHz < fsystem <  2.5MHz
	UCSCTL1 = DCORSEL_2;
  else if (fsystem <  5000)      // 2.5MHz  < fsystem <    5MHz
	UCSCTL1 = DCORSEL_3;
  else if (fsystem <  10000)     // 5MHz    < fsystem <   10MHz
	UCSCTL1 = DCORSEL_4;
  else if (fsystem <  20000)     // 10MHz   < fsystem <   20MHz
	UCSCTL1 = DCORSEL_5;
  else if (fsystem <  40000)     // 20MHz   < fsystem <   40MHz
	UCSCTL1 = DCORSEL_6;
  else
	UCSCTL1 = DCORSEL_7;

  while (SFRIFG1 & OFIFG) {                               // Check OFIFG fault flag
    UCSCTL7 &= ~(DCOFFG+XT1LFOFFG+XT1HFOFFG+XT2OFFG);     // Clear OSC flaut Flags
    SFRIFG1 &= ~OFIFG;                                    // Clear OFIFG fault flag
  }

  if (mode == 1) {                              		  // fsystem > 16000
    SELECT_MCLK_SMCLK(SELM__DCOCLK + SELS__DCOCLK);       // Select DCOCLK
  }
  else {
    SELECT_MCLK_SMCLK(SELM__DCOCLKDIV + SELS__DCOCLKDIV); // Select DCODIVCLK
  }

  __bis_SR_register(srRegisterState);	                  // Restore previous SCG0
}

#endif

/************************************************************************
 * @brief  OSCSet : CONFIGURE CLOCK,32k AS CLK SOURCE ,FLL generate DCO =4M
 * @param  none
 * @return none
 *************************************************************************/
void OSCSet(void)
{
	   UCSCTL6 &= ~(XT1OFF);                     // XT1 On
	   UCSCTL6 |= XCAP_3;                        // Internal load cap

	   while(BAKCTL & LOCKBAK)                    // Unlock XT1 pins for operation
	      BAKCTL &= ~(LOCKBAK);
	   // Loop until XT1 fault flag is cleared
	   do {
	     UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
	                                             // Clear XT2,XT1,DCO fault flags
	     SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	   }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

	  // UCSCTL6 &= ~(XT1DRIVE_3);                 // Xtal is now stable, reduce drive strength

	   // Initialize DCO to 4MHz
	   __bis_SR_register(SCG0);                  // Disable the FLL control loop
	   UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
	   UCSCTL1 = DCORSEL_3;                      // Set RSELx for DCO = 4.9 MHz
	   UCSCTL2 = FLLD_1 + 123;                    // Set DCO Multiplier for 4MHz
	                                             // (N + 1) * FLLRef = Fdco
	                                             // (123 + 1) * 32768 = 4MHz
	   __bic_SR_register(SCG0);                  // Enable the FLL control loop

	   // Worst-case settling time for the DCO when the DCO range bits have been
	   // changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
	   // UG for optimization.
	   // 32 x 32 x 2.45 MHz / 32,768 Hz = 76563 = MCLK cycles for DCO to settle
	   __delay_cycles(56563);
	   // Loop until XT1,XT2 & DCO fault flag is cleared
	   do
	   {
	     UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + XT1HFOFFG + DCOFFG);
	                                             // Clear XT2,XT1,DCO fault flags
	     SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	   }while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

	   UCSCTL4 = SELA_0+SELS_3+SELM_3;          // ACLK = LFTX1 (by default),MCLK,SMCLK =DCOCLK
}
/**********************************************************************//**
 * @brief  lowPowerInit : init VUSB internal LDO and disable SVS
 * @param  none
 * @return none
 *************************************************************************/
void lowPowerInit(void)
{
  // Disable VUSB LDO and SLDO
  USBKEYPID   =     0x9628;                 // set USB KEYandPID to 0x9628
                                            // enable access to USB config reg
  USBPWRCTL &= ~(SLDOEN+VUSBEN);            // Disable the VUSB LDO and the SLDO
  USBKEYPID   =    0x9600;                  // disable access to USB config reg

  //set wake up time from low-power mode, full-performance
   PMMCTL0_H = PMMPW_H;                // PMM Password
   SVSMHCTL |=SVMHFP+SVSHFP;
   SVSMLCTL |=SVMLFP+SVSLFP;
}
/**********************************************************************//**
 * @brief  MCU_init : init LCD, SVS, BUTTON BASIC TIMER
 *                    Select charge pump as LCD voltage source
 *                    Configure basic timer as 1s timer interval--RTC/32khz as clock source
 *                    Configure SVS at 2.5V,  no POR,if voltage lower than 2.5V,SVSFG will be set
 *                    Configure  P1.6/P1.7-as BUTTON ---HIGH TO LOW transition INTERRUPT
 * @param  none
 * @return none
 *************************************************************************/
void MCUinit(void)
{
	// configure the LCD
	   LCDBCTL0&=~LCDON;
	   LCDBCTL0  |=LCDDIV_21+LCDPRE_1+LCDMX0+LCDMX1+LCDSON;
	   LCDBVCTL |=LCDCPEN;
	   LCDBVCTL |=VLCD_8;
	   LCDBPCTL0=0xffff;
	   LCDBPCTL1=0xffff;
	   LCDBPCTL2 |=LCDS32;
	   P5SEL |=BIT3+BIT4+BIT5;                                  //COM1--COM3 select,COM0 default
	   LCDBCTL0|=LCDON;
	// Configure RTC_B
	   RTCCTL01 = RTCRDYIE  + RTCHOLD +RTCBCD;  // RTC hold, enable RTC read ready interrupt
	   RTCYEAR = 0x2013;                         // Year = 0x2011
	   RTCMON = 0x10;                            // Month = 0x10 = October
	   month=10;
	   RTCDAY = 0x07;                            // Day = 0x07 = 7th
	   day=7;
	   RTCDOW = 0x05;                            // Day of week = 0x05 = Friday
	   RTCHOUR = 0x11;                           // Hour = 0x11
	   hour=11;
	   RTCMIN = 0x59;                            // Minute = 0x59
	   minute=59;
	   RTCSEC = 0x45;                            // Seconds = 0x45
	   RTCPS1CTL=RT1IP_5+RT1PSIE;               //2Hz ,0.5 s for flash ":"
	   RTCCTL23 =RTCCALF_1;
	   RTCCTL01 &= ~(RTCHOLD);                   // Start RTC calendar mode

	   P5DIR|=BIT7;
	   P5SEL|=BIT7;                             //RTC output on P5.7

	   //P1.2 AND P1.3 WORK AS GPIO,OUTPUT LOW
	   P1OUT&=~(BIT2+BIT3);
	   P1DIR|=BIT2+BIT3;
	   P1SEL&=~(BIT2+BIT3);
     // BATTERY DETECT,reserved

	// Configure P6.4~P6.7 AS BUTTON
	   P6DIR&=~(BIT4+BIT5+BIT6+BIT7);
	   P5OUT&=~BIT6;
	   P5DIR|=BIT6;          //P5.6 OUTPUT LOW LEVEL

    //configure timer2 to generate 15ms 4000/4M=1ms
	   TA2CCR0 = 480;                                //
	   TA2CTL = TASSEL_1 + TACLR+ MC_1;              // ACLK, up mode,
	   TA2CCTL0|=CCIE;                               //ENABLE CCR0 INTERRUPT

}

/*********************************************************************
* @ brief give default value to variables
* @param none
* @return none
**********************************************************************/
void varInit(void)
{
	workMode=STANDBY;
	setPosition=MONTH;

    AMPMmode=1;               //12-hour mode
    batterylow=0;
    batterymid=0;
    batteryhigh=0;
    ms15Flag=0;

    currentItemNum=0;
    memorySta=MEMSTACLOSE;
    memUpdateDisp=0;
    recordUpdate=0;

    measureSta=MEASURESTACLOSE;
    ADCDataCounter=0;
    highPreCounter=0;

    ADCStopFlag=0;

    MEMP7=0;
    MEMP8=0;
    mmhg=0;
    TIlogo=0;
    kpa=0;
    coupleDots =0;
}

/************************************************************************************************
* ADC interrupt service routine
* ADC samples and process
* two steps: 1, air in quickly, don't store data to e2prom
*            2, air in slowly, store data to e2prom,
*            when pressure>120mmhg, stop ADC
*************************************************************************************************/
#pragma vector=ADC12_VECTOR
__interrupt void ADC12_ISR(void)
{
    ADC12CTL0 &= ~ADC12ENC;                                   // ADC12 disabled
	ADC12CTL0 = 0;                                            // ADC12, Vref disabled completely

		ADCSampleCounter++;

		if(measureSta==MEASUREDISPAIRINQUICK){                // let air in quickly, don't store data to E2PROM
			  if(ADC12MEM0>=LowToHigh_PRE) {                  // 250,reach 36mmhg ,aerate slower
				  TA0CCR1 = LOW_AIRIN;                        // aerate airout slower
				  measureSta=MEASUREDISPAIRINSLOW;
			  }
			  else
			  if((ADCSampleCounter>=1200)&&(ADC12MEM0<=LowToHigh_PRE)) {    // 200- smaller than 25.7mmhg ,cuff is not ok
				  TA1CCTL0&=~CCIE;
				  TA0CCTL1 = OUTMOD_5;                            // stop PUMP
				  TA0CCTL2 = OUTMOD_5;                            // stop valve
				  TA0CCR1 = HIGH_AIRIN;
				  measureSta=MEASUREDISPERROR;
				  errorCode=CUFF_ERR ;
				  ADCSampleCounter=0;
			  }
		}

		if(measureSta==MEASUREDISPAIRINSLOW) {
				rawData[ADCDataCounter]=ADC12MEM0&0x0fff;
			    ADCDataCounter++;
			    if(ADCDataCounter>=3000){
			    	measureSta=MEASUREDISPERROR;    //raw data overflow
			    	errorCode=OVER_FLOW;
			      }
#ifdef UART
	ADC_array[ii]=(ADC12MEM0>>8)&0x0f;
	CheckSum+=ADC_array[ii];
	ADC_array[ii+1]=ADC12MEM0&0xff;
	CheckSum+=ADC_array[ii+1];
	ii=ii+2;

	if(ii==0x06)
	{
		ADC_array[ii]=CheckSum+CheckSum1;
		ii=4;
		UCA0IE |= UCTXIE;
		CheckSum=0;
	}
#endif
			 if(ADC12MEM0>=MAX_PRE){                             //645 reach 120mmhg
				 highPreCounter++;
				 if(highPreCounter>=5){                      //pressure over 120mmhg for more than 5 times
					 highPreCounter=0;
				     TA0CCTL1 = OUTMOD_5;                        // stop PUMP,reset
				     TA0CCTL2 = OUTMOD_5;                        // stop valve
				     TA0CCR1 = HIGH_AIRIN;
				     measureSta=MEASUREDISPAIROUT;               //air escape start
				     ADCSampleCounter=0;
				  }
			  }
			if((ADCSampleCounter>=1200)&&(ADC12MEM0<=LowToHigh_PRE)) { // - smaller than certain mmhg ,cuff is not ok or the air is leak from cuff
				  TA1CCTL0&=~CCIE;
				  TA0CCTL1 = OUTMOD_5;                        // stop PUMP,reset
				  TA0CCTL2 = OUTMOD_5;                        // stop valve
				  TA0CCR1 = HIGH_AIRIN;
				  measureSta=MEASUREDISPERROR;
				  errorCode=CUFF_ERR;
				  ADCSampleCounter=0;
			  }
		}

		if(measureSta==MEASUREDISPAIROUT) {
			if(ADC12MEM0<=senserNoise)  {                   //20mmHg
				TA1CCTL0&=~CCIE;               //stop ADC
				ADCStopFlag=1;                 //set stop adc flag for measure process judge
				P6OUT&=~BIT3;
				P6DIR|=BIT3;                   //STOP power to amplifier, save power consumption
			}
		}
	//Vout=0.054*P+0.12, P uint is kPa, Vout uint is V,
	// supose 200mmHg is the max value of the op circuit, then output of sensor is about 1.56V at 20mmHg
	//select ADC 2.5V as ref, then op gain is 2.5/1.56=1.6, ,100k and 61.9k as resistors
	//P(kPa)=(Vout-0.12)/0.054,  1kPa=7.5mmHg
	//P(mmHg)=(Vout-0.12)*7.50062/0.054=(Vout-0.12)*139
	//Vout*x=Vin                         x is the gain of hardware,100k 61.9k so gain is 1.619
	//Vin/2.5=ADCMEM/4096               2.5 is the reference voltage
    // p= (ADCMEM*2.5-0.12*4096*x)*139/(4096*x)

	if(ADC12MEM0<=zreoPressure)
		preValue=0;
	else
	preValue=0.0848*ADC12MEM0/GAIN-16.68;     //preValue unit is mmhg
}

/************************************************************************************************
 *  timer1, for ADC12 sample frequency control, 6V use Avcc as ref,
 *  3V system use 2.5V as reference voltage
*************************************************************************************************/
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_CCR0_ISR(void)
{
	  ADC12CTL0 &= ~ADC12ENC;                                   // ADC12 disabled
	  REFCTL0 &= ~REFMSTR;                         // Reset REFMSTR to hand over control to
	                                               // ADC12_A ref control registers
	  ADC12CTL0 =  ADC12SHT0_2 + ADC12REFON +  ADC12ON + ADC12REF2_5V;
	  ADC12CTL1 = ADC12SHP + ADC12SSEL_1;          // enable sample timer,aclk
	  ADC12MCTL0 = ADC12SREF_1 + ADC12INCH_0;      // ADC A0
	  __delay_cycles(60);                          // Delay for reference start-up
	  ADC12IE = ADC12IE0;                          // ADC_IFG upon conv result-ADCMEMO
	  ADC12CTL0 |= ADC12ENC+ADC12SC;                    //enable and start convertion
}

/************************************************************************************************
 * timer2 for 15ms generate
*************************************************************************************************/
#pragma vector=TIMER2_A0_VECTOR           //
__interrupt void TIMER2_CCR0_ISR(void)
{
	ms15Flag=1;
//	if (((memorySta==LAST3SEC)&&(workMode==MEMORY))||((workMode==MEASURE)&&(measureSta==MEASUREDISPBEGIN)))
//		__no_operation();
//	else
	_bic_SR_register_on_exit(LPM0_bits);           // exit LPM3 and display
}

/************************************************************************************************
* @brief  This function interrupts once every 0.5 second
*         freshFlag : 1:display ":"   ;  0: display blank at ":"
*         freshFlag is also used for other judgment
*************************************************************************************************/
#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void)
{
//------------------------------------------------------------------------------
	 while(BAKCTL & LOCKBAK)                    // Unlock backup system
	         BAKCTL &= ~(LOCKBAK);

	   switch(__even_in_range(RTCIV,14))
	   {
	   case  0: break;                           // Vector  0:  No interrupt
	   case  2:                                  // Vector  2:  RTCRDYIFG
		   if(workMode!=DATESET){
			     minute = (RTCMIN>>4)*10+(RTCMIN&0x0f);
			     hour=(RTCHOUR>>4)*10+(RTCHOUR&0x0f);
			     day =(RTCDAY>>4)*10+(RTCDAY&0x0f);
			     month = (RTCMON>>4)*10+(RTCMON&0x0f);
		   }
	     _bic_SR_register_on_exit(LPM0_bits);    // exit LPM3 and display

	     break;
	   case  4: break;                           // Vector  4:  RTCEVIFG
	   case  6: break;                           // Vector  6:  RTCAIFG
	   case  8: break;                           // Vector  8:  RT0PSIFG
	   case 10:                                  // Vector 10:  RT1PSIFG
		freshFlag^=BIT0;
		if(workMode==MEASURE)
			measureUpdateDisp=1;
		if(workMode==MEMORY)
			memUpdateDisp=1;

	     _bic_SR_register_on_exit(LPM0_bits);       // exit LPM3 and display

		break;
	   case 12: break;                           // Vector 12:  RTCOFIFG
	   case 14: break;                           // Vector 14:  Reserved
	   default: break;
	   }

}


/************************************************************************************************
* @brief : enable Button
* @ param  none
* @ return  none
*************************************************************************************************/
void enableButton(void)
{
 	 TA2CCTL0|=CCIE;
}

/************************************************************************************************
* @brief : disable Button
* @ param  none
* @ return  none
*************************************************************************************************/
void disableButton(void)
{
	 TA2CCTL0&=~CCIE;                               // close timer2 for button, this will affect read/write e2prom
}

/*************************************************************************************************
* @brief : delay function
* @ param : tempValue is the period that delayed
* @ return  none
***************************************************************************************************/
void _delayxSecond(unsigned char tempValue)
{
	while(tempValue){
		__bis_SR_register(LPM0_bits +GIE);    // only basic timer can exit from  LPM3
		tempValue--;
	}
}

#ifdef E2PROMTEST
//---------------------------for i2c test--------------------------------------
void E2PROMWrite(void)
 {
      test=1;
	  InitI2C(SlaveAddress);
//      EEPROM_ByteWrite(0x0000,0x12);
//      EEPROM_AckPolling();                      // Wait for EEPROM write cycle
//                                                  // completion
//      test = EEPROM_RandomRead(0x0000);           // Read from address 0x0000
 // write the first record
      ReadWrite_val[0]=9;  //month
      ReadWrite_val[1]=16;  //day
      ReadWrite_val[2]=10;   // hour
      ReadWrite_val[3]=36;    //minute
      ReadWrite_val[4]=123;  //systolic
      ReadWrite_val[5]=75;  //diastolic
      ReadWrite_val[6]=93;   // heart rate
      ReadWrite_val[7]=1;    //heart normal
      E2PROMaddress = 0x0000;                         // Set starting address at 0
      // Write a sequence of data array
      EEPROM_PageWrite(E2PROMaddress , ReadWrite_val , sizeof(ReadWrite_val));
      // Read out a sequence of data from EEPROM
      EEPROM_SequentialRead(E2PROMaddress, ReadWrite_val , sizeof(ReadWrite_val));
  // wirte the second record
      ReadWrite_val[0]=9;  //month
      ReadWrite_val[1]=22;  //day
      ReadWrite_val[2]=14;   // hour
      ReadWrite_val[3]=45;    //minute
      ReadWrite_val[4]=117;  //systolic
      ReadWrite_val[5]=69;  //diastolic
      ReadWrite_val[6]=62;   // heart rate
      ReadWrite_val[7]=0;    //heart normal
      E2PROMaddress = 0x0008;                         // Set starting address at 0
      // Write a sequence of data array
      EEPROM_PageWrite(E2PROMaddress , ReadWrite_val , sizeof(ReadWrite_val));
      // Read out a sequence of data from EEPROM
      EEPROM_SequentialRead(E2PROMaddress, ReadWrite_val , sizeof(ReadWrite_val));

 // write total records number to 0xfffd
      EEPROM_ByteWrite(0x7ffd,0x02);
      EEPROM_AckPolling();                      // Wait for EEPROM write cycle
                                                // completion
      test = EEPROM_RandomRead(0x7ffd);        // Read from address 0x0000

 // write the start address of the last record to 0xfffe, 0xffff
      EEPROM_ByteWrite(0x7ffe,0x00);
      EEPROM_AckPolling();
      EEPROM_ByteWrite(0x7fff,0x08);
      EEPROM_AckPolling();
      StartAddress1[0]=EEPROM_RandomRead(0x7ffe);   // high address
      StartAddress1[1]=EEPROM_RandomRead(0x7fff);   // low address
}
#endif

