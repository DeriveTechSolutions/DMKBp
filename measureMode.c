/*
 * measureMode.c
 *
 *  Created on: Feb 6, 2013
 *      Author: a0220221
 */

#include "ext_config.h"
#include "measureMode.h"
#include "UCB0_I2C.h"
#include "display.h"
#include "Algorithm.h"

#define I2C
#define HIGH_AIRIN 2500


/**********************************************************************//**
 * @brief  ADCInit():Configure ADC Module, ACLK as clock source and channel 0 input
 * @param  none
 * @return none
 *************************************************************************/
void ADCInit(void)
{
	P6OUT|=BIT3;
	P6DIR|=BIT3;
}
/*********************************************************************
 * @brief timer0_init : generate PWM signal for pump and valve control
 * @param  none
 * @return none
**********************************************************************/
void timer0Init(void)
{
	   P1DIR|=BIT2+BIT3;
	   P1SEL|=BIT2+BIT3;                              //SELECT P1.2/P1.3 AS TA0 OUTPUT

	   TA0CCR0 = 4000;                               //
	   TA0CCTL1 = OUTMOD_7;                          // CCR1 toggle/set,PUMP
	   TA0CCTL2 = OUTMOD_1;                           //VALVE
	   TA0CCR2 = 3400;                                // CCR1 PWM duty cycle  for valve
	   TA0CCR1 = HIGH_AIRIN;                          // for pump
	   TA0CTL = TASSEL_2 + TACLR+ MC_1;              // SMCLK, up mode,
}

/*********************************************************************
 * @brief timer1_init : generate sample start signal for ADC12
 * @param  none
 * @return none
**********************************************************************/
void timer1Init(void)
{
	   TA1CCR0 = 320;                               // 100hz sample rate for ADC10
	   TA1CCR1 = 295;                               // generate rising edge
	   TA1CCTL1 = OUTMOD_2;                         // CCR1 toggle/reset
	   TA1CTL = TASSEL_1 + TACLR+ MC_1;             // ACLK, up mode, ENABLE CCR0 INTERRUPT
	   TA1CCTL0|=CCIE;
}
/*********************************************************************
 * @brief displayMeasure : measure mode display
 * @param  none
 * @return none
**********************************************************************/
void displayMeasure(void)
{
	unsigned char address[2];          // store the address of latest record

	switch (measureSta)  {
	//display all for 3 second and
	//display the first record for 2 second and
	//display zero pressure and RTC
		 case MEASUREDISPBEGIN :
		     displayMeasureInit();
		 break;

	//air in quickly and --SMCLK work
		 case MEASUREDISPAIRINQUICK:
		      if(measureUpdateDisp)   {            //every second  update the RTC and Pressure
			      displayAerate();                  //display RTC and current air pressure
		          measureUpdateDisp=0;
		      }
	     break;

	//air in slowly
		 case MEASUREDISPAIRINSLOW:
		      if(measureUpdateDisp)   {             //every 0.5 second  update the RTC and Pressure
			      displayAerate();                  //display RTC and current air pressure
		          measureUpdateDisp=0;
		      }
	     break;

	//air out start,algorithm design to get measure result
		 case MEASUREDISPAIROUT:
			 dataProcess();
		     break;

	//display measure result
		 case MEASUREDISPPRORESULT:
				close_record();
				timeFresh();                                      // change the values of month,day,hour,minute
				displayDate(month,day);
				displayTime(hour,minute);
				displayPressure(recordContent.systolic,recordContent.diastolic,recordContent.heartRate,recordContent.heartBad_flag);                    //display it
				displaySetmem();

				recordContent.day=day;
				recordContent.month=month;
				recordContent.hour=hour;
				recordContent.minute=minute;
#ifdef I2C
				// write record to E2PROM
				ReadWrite_val[0]=recordContent.month;  //month
				ReadWrite_val[1]=recordContent.day;  //day
				ReadWrite_val[2]=recordContent.hour;   // hour
				ReadWrite_val[3]=recordContent.minute;    //minute
				ReadWrite_val[4]=recordContent.systolic;  //systolic
				ReadWrite_val[5]=recordContent.diastolic;  //diastolic
				ReadWrite_val[6]=recordContent.heartRate;   // heart rate
				ReadWrite_val[7]=recordContent.heartBad_flag;    //heart normal

				InitI2C(SlaveAddress);
				startAdd=getStartAdd();                    // get the latest record address
				E2PROMaddress=startAdd+8;
				if(E2PROMaddress>=0x0280)
					E2PROMaddress=0;
                disableButton();
                //write record into address
                EEPROM_PageWrite(E2PROMaddress , ReadWrite_val, sizeof(ReadWrite_val));
                //write new address to E2PROM
                address[1]=(E2PROMaddress&0x00ff);
                address[0]=E2PROMaddress>>8;
                EEPROM_PageWrite(0xfffe,address,sizeof(address));
                //write new recordCount to E2PROM
                itemTotalCount=itemTotalCount+1;
                if(itemTotalCount>=80)
                	itemTotalCount=80;

                EEPROM_ByteWrite(0xfffd,itemTotalCount);
                EEPROM_AckPolling();                      // Wait for EEPROM write cycle
                enableButton();
#endif
                measureSta=MEASUREEND;
			 break;

    //if there is any error in the system,display it
		 case MEASUREDISPERROR:
				if(measureUpdateDisp)   {                     //RTC update 0.5s
				   displayErrorPro(errorCode);
				   measureUpdateDisp=0;

				   P1OUT&=~(BIT2+BIT3);
				   P1DIR|=BIT2+BIT3;
				   P1SEL&=~(BIT2+BIT3);
				}
			 break;

	//measure end, display the result
		 case MEASUREEND:
				close_record();
				timeFresh();                                      // change the values of month,day,hour,minute
				displayDate(month,day);
				displayTime(hour,minute);
				displayPressure(recordContent.systolic,recordContent.diastolic,recordContent.heartRate,recordContent.heartBad_flag);                    //display it
				displaySetmem();
				P1OUT&=~(BIT2+BIT3);
				P1DIR|=BIT2+BIT3;
				P1SEL&=~(BIT2+BIT3);
			 break;

		 default :
			 __no_operation();
			 break;
	}

}

/*********************************************************************
 * @brief displayMeasureInit : measure mode display init
 *                             display all segments for 3 seconds and
 *                             display the first record for 2 seconds
 *                             display the zero pressure and RTC
 * @param  none
 * @return none
**********************************************************************/
void displayMeasureInit(void)
{
     unsigned char tempCounter=0;
 //display all segments for 3 seconds
	  disableButton();
	  SET_LCD_ON();                              //display all segments
	  P1DIR|=BIT1;                              //start buzzer
	  P1OUT|=BIT1;
	  _delayxSecond(4);                          //display 3s and buzzer running, no response to button
      P1OUT&=~BIT1;
	  P1DIR&=~BIT1;                             //stop buzzer
//display the latest record for 2 seconds
#ifdef I2C
	  InitI2C(SlaveAddress);
	  itemTotalCount = EEPROM_RandomRead(0xfffd);
#else
	  itemTotalCount=0;
#endif
	 if(itemTotalCount) {                          //display the latest record
		 dotFlashFlag=1;
#ifdef I2C
		 startAdd=getStartAdd();
		 getRecord(startAdd);
#endif
		 tempCounter=6;
		 while(tempCounter) {                      //display for 3 seconds,exit lpm3 only from basic timer
			 if(measureUpdateDisp){
		       displayRecord();
		       measureUpdateDisp=0;
		       tempCounter--;
		       __bis_SR_register(LPM0_bits +GIE);   //only RTC can wake up
			 }
		 }
		 dotFlashFlag=0;
	 }
	 else {                                           //no record, display RTC and zero
		 tempCounter=6;
		 while(tempCounter) {
			 if(measureUpdateDisp){
		       timeFresh();                                 //change the values of month,day,hour,minute
		       displayDate(month,day);
		       displayTime(hour,minute);
		       displayZero();                              // display zero at some places
		       displaySetmem();
		       measureUpdateDisp=0;
		       tempCounter--;
		       __bis_SR_register(LPM0_bits +GIE);
			 }
		 }
	 }
//display the 0mmhg pressure and RTC  for 2 seconds
	 close_record();
	 tempCounter=4;
	 while(tempCounter) {
		 if(measureUpdateDisp){
	        timeFresh();                              // change the values of month,day,hour,minute
	        displayDate(month,day);
	        displayTime(hour,minute);
            displayAirPre(0);                        //display zero pressure at the beginning of AIRIN
            displaySetmem();
		    measureUpdateDisp=0;
		    tempCounter--;
		    __bis_SR_register(LPM0_bits +GIE);
		 }
	 }
	 measureSta=MEASUREDISPAIRINQUICK;
	 enableButton();
// start AIRIN
	 ADCInit();                //configure GPIO as ADC input
	 timer0Init();             // for PWM generator
	 timer1Init();             // for ADC sample rate control
	 ADCSampleCounter=0;       //ADC sample counter init,if this counter>1000 and the pressure is lower than certain value,that means the cuff is not ok                               //aerate
	 ADCStopFlag=0;
	 endProFlag=0;
	 dotFlashFlag=0;
	 ADCDataCounter=0;
	 highPreCounter=0;
}

/*********************************************************************
 * @brief displayAerate : display in AERate air step (measureSta=2)
 *                        getting air into the cuff
 * @param  none
 * @return none
**********************************************************************/
void displayAerate(void)
{
	close_record();
	timeFresh();                         // change the values of month,day,hour,minute
	displayDate(month,day);
	displayTime(hour,minute);
    displayAirPre(preValue);
    displaySetmem();
}
/*********************************************************************
 * @brief displayDeflate : display in deflate air step (measureSta=3)
 *                         getting air out of the cuff
 * @param  none
 * @return none
**********************************************************************/
void displayDeflate(void)
{
    close_record();
    timeFresh();                              // change the values of month,day,hour,minute
    displayDate(month,day);
    displayTime(hour,minute);
    displayAirPre(preValue);
    displaySetmem();
}

/*********************************************************************
 * @brief displayAirPre : display the current air pressure of cuff
 * @param  airPre---the pressure of the cuff
 * @return none
**********************************************************************/
void displayAirPre(unsigned char airPre)
{
	unsigned char p;
	p=0;
	   while(airPre>=100)
	   {
		   airPre=airPre-100;
		   p++;
	   }
	   f_displpreh=digit4_value[p];         //display pressure high position

		p=0;
		   while(airPre>=10)
		   {
			   airPre=airPre-10;
			   p++;
		   }
		   f_displprem=digit3_value[p];   //display pressure mid position

		   f_displprel=digit3_value[airPre];   //display pressure low position

		   f_dispmmhg=1;                     // display mmhg
		   f_dispsanjiao=1;                  // display sanjiao

}

/*********************************************************************
 * @brief displayErrorPro : display  error process in measure mode
 *                         for example disconnect the pump or valve
 * @param  none
 * @return none
**********************************************************************/
void displayErrorPro(unsigned char errCode)
{
	 close_record();
	 timeFresh();                              // change the values of month,day,hour,minute
	 displayDate(month,day);
	 displayTime(hour,minute);
     displayErr(errCode);
     displaySetmem();
}
/*********************************************************************
 * @brief displayErr : display ERROR in  measure mode, if any wrong operation occurs
 *                     EE will be displayed on the position of systolic pressure (the middle and low bits )
 *                     (RTC will display , too)
 * @param  none
 * @return none
**********************************************************************/
void displayErr(unsigned char errCode )
{
	f_disphprem=0x79;     // display E in middle bit of systolic pressure
	switch(errCode) {
	case CUFF_ERR:
		f_disphprel=0x3f;     //display 0 in low bit of systolic pressure
		break;

	case OVER_FLOW:
		f_disphprel=0x06;     //display 1 in low bit of systolic pressure
		break;

	default:
		break;
	}

}


