#include "ext_config.h"
#include "key.h"

#define HIGH_AIRIN 2500
#define startButton 2
#define memoryButton 3
#define combinationButton 5
volatile unsigned char readKeyCounter=0;
unsigned char keyValue=0;
unsigned char lastKeyValue=0;
unsigned char keyid=0;

/**********************************************************************//**
 * @brief  keyJudge():Judge which work status the project should be convert to
 *                     and then call the related function
 * @param  none
 * @return none
 *************************************************************************/
void keyJudge(void)
{
    if((P6IN&BIT7)&&(P6IN&BIT6)) {  // no button pressed
    	keyValue=0;
    	lastKeyValue=0;

        if(keyid==combinationButton){
        	workStatus=COMBINATIONBUTTON;
    		statusConvert();
        }
        else if(keyid==startButton) {
        	workStatus= STARTBUTTON;
    		statusConvert();
        }
        else if (keyid==memoryButton) {
		    workStatus= MEMORYBUTTON;
			statusConvert();
        }
        else
        	__no_operation();

		keyid=0;
    }
    else {
    	if(((P6IN&BIT7)==0)&&((P6IN&BIT6)==0)) {
    		keyValue=combinationButton;
    		if(lastKeyValue==keyValue)
    			readKeyCounter++;
    		else {
    			lastKeyValue=keyValue;
    			readKeyCounter=0;
    		}
    		if(readKeyCounter>=4){
    			keyid=combinationButton;
    		}
    	}
    	else {
    	    if((P6IN&BIT7)==0){
    		    keyValue=startButton;
    		    if(lastKeyValue==keyValue)
    		        readKeyCounter++;
    			else {
    				lastKeyValue=keyValue;
    				readKeyCounter=0;
    			}
    		    if(readKeyCounter>=4){
    		    	keyid=startButton;
    		    }
    	    }

    	    if((P6IN&BIT6)==0){
    		    keyValue=memoryButton;
    		    if(lastKeyValue==keyValue)
    			    readKeyCounter++;
    			else {
    				lastKeyValue=keyValue;
    				readKeyCounter=0;
    			}
    		    if(readKeyCounter>=4){
    		    	keyid=memoryButton;
    		     }
    	     }
    	}

    }

}
/**********************************************************************//**
 * @brief  statusConvert():Judge which work status the project should be converted to
 * @param  none
 * @return none
 *************************************************************************/

void statusConvert(void)
{
	switch(workMode){

	case STANDBY:                                   //
		switch(workStatus){
		case COMBINATIONBUTTON:
			workMode=DATESET;
			break;
		case STARTBUTTON:
			workMode=MEASURE;
			measureSta=MEASUREDISPBEGIN;      // different step of measure mode 0:end one measure;1: display all;2:display the last record;3:chongqi;4: fangqi 5:display
			break;
		case MEMORYBUTTON:
			workMode=MEMORY;
			memorySta=LAST3SEC;                      // enter 3s itemCount display mode
			break;
		}
		 break;

	case DATESET:                                      //
		switch(workStatus){
		case COMBINATIONBUTTON:
			 if(setPosition==MONTH)  {                   // this is same as press memory button
				month++;
			     if(month>12)
				    month=1;
			 }
			 if(setPosition==DAY) {
				day++;
			     if((day>30) && ((month==4)||(month==6)||(month==9)||(month==11)))
				   day=1;
			     else if (day>31)
			    	 day=1;
			     else {
			         if((((RTCYEAR&0x00f0)>>4)*10  + (RTCYEAR&0x000f))%4 ){
				         if((day>28)&&(month==2))
					        day=1;
			           }
			         else {
				            if((day>29)&&(month==2))
					            day=1;
			           }
			     }
			 }
			 if(setPosition==HOUR) {
				hour++;
			    if(hour>=24)
				   hour=0;
			 }
			 if(setPosition==MINUTE) {
				minute++;
			    if(minute>=60)
				   minute=0;
			 }
			break;
		case STARTBUTTON:
			setPosition++;
            if(setPosition==5) {
            	setPosition=MONTH;
           	    workMode=STANDBY;
           	 while(BAKCTL & LOCKBAK)                    // Unlock backup system
           	         BAKCTL &= ~(LOCKBAK);
           	// Configure RTC_B
           		RTCCTL01 = RTCRDYIE + RTCHOLD +RTCBCD;  // RTC hold,
           		RTCYEAR = 0x2013;
              	RTCMON  =(month/10)*16+(month%10);
              	RTCDAY  =(day/10)*16+(day%10);
              	RTCHOUR =(hour/10)*16+(hour%10);
              	RTCMIN=(minute/10)*16+(minute%10);
           		RTCPS1CTL=RT1IP_5+RT1PSIE;
           		RTCCTL01 &= ~(RTCHOLD);
            }
			break;
		case MEMORYBUTTON:
			 if(setPosition==MONTH)  {                   // this is same as press memory button
				month++;
			     if(month>12)
				    month=1;
			 }
			 if(setPosition==DAY) {
				day++;
			     if((day>30) && ((month==4)||(month==6)||(month==9)||(month==11)))
				   day=1;
			     else if (day>31)
			    	 day=1;
			     else {
			         if((((RTCYEAR&0x00f0)>>4)*10  + (RTCYEAR&0x000f))%4 ){
				         if((day>28)&&(month==2))
					        day=1;
			           }
			         else {
				            if((day>29)&&(month==2))
					            day=1;
			           }
			     }
			 }
			 if(setPosition==HOUR) {
				hour++;
			    if(hour>=24)
				   hour=0;
			 }
			 if(setPosition==MINUTE) {
				minute++;
			    if(minute>=60)
				   minute=0;
			 }
			break;
		}
		 break;

	case MEASURE:                                          // -----------------measure mode-------------
		switch(workStatus){
		case COMBINATIONBUTTON:  //stop measure
			workMode=STANDBY;
			TA1CCTL0&=~CCIE;                            // stop AD sample
			ADC12CTL0 &= ~ADC12ENC;                        // ADC10 disabled
			ADC12CTL0 = 0;                            // ADC10, Vref disabled completely
			TA0CCTL1 = OUTMOD_5;                        // stop PUMP,reset
			TA0CCTL2 = OUTMOD_5;                        // stop valve
			TA0CCR1 = HIGH_AIRIN;

			P1OUT&=~(BIT2+BIT3);
			P1DIR|=BIT2+BIT3;
			P1SEL&=~(BIT2+BIT3);
			//stop_process;
			measureSta=MEASURESTACLOSE;                              //air escape start
			close_record();
			break;

		case STARTBUTTON:             //stop measure
			workMode=STANDBY;
			TA1CCTL0&=~CCIE;                            // stop AD sample
			ADC12CTL0 &= ~ADC12ENC;                        // ADC10 disabled
			ADC12CTL0 = 0;                            // ADC10, Vref disabled completely
			TA0CCTL1 = OUTMOD_5;                        // stop PUMP,reset
			TA0CCTL2 = OUTMOD_5;                        // stop valve
			TA0CCR1 = HIGH_AIRIN;
			P1OUT&=~(BIT2+BIT3);
			P1DIR|=BIT2+BIT3;
			P1SEL&=~(BIT2+BIT3);
			//stop_process;
			measureSta=MEASURESTACLOSE;                              //air escape start
			close_record();
			break;

		case MEMORYBUTTON:
            // no action
			break;
		}
		 break;

	case MEMORY:                                    //memory mode
		switch(workStatus){
		case COMBINATIONBUTTON:
			workMode=STANDBY;
			currentItemNum=0;
			memorySta=MEMSTACLOSE;
			close_record();//end function
			break;

		case STARTBUTTON:
			workMode=STANDBY;
			currentItemNum=0;
			memorySta=MEMSTACLOSE;
			close_record();//end function
			break;

		case MEMORYBUTTON:
			currentItemNum++;                               // list++ display
			recordUpdate=1;
            if(currentItemNum>=itemTotalCount) {              // the last record
            	workMode=STANDBY;
            	currentItemNum=0;
            	memorySta=MEMSTACLOSE;
                close_record();
                itemTotalCount=0;
                recordUpdate=0;
            }
			break;
		}
		 break;
	}
}






