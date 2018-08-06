#include "ext_config.h"
#include "memoryMode.h"
#include "UCB0_I2C.h"
#include "display.h"

volatile unsigned int currentItemAdd;         //the address of current record displayed in LCD
int signedCurrentItemAdd;

/*********************************************************************
 * @brief displayMemory : display Memory mode
 * @param  none
 * @return none
**********************************************************************/
void displayMemory(void)
{
	 if(memorySta==LAST3SEC){
         disableButton();
         InitI2C(SlaveAddress);                         // Initialize I2C module
         displayMemInitState();                         // last about 4s while in LPM3 only wake up from basic timer
         memorySta=MEMDISPRECORD;                       // Next step is display memory record
         enableButton();
         memUpdateDisp=1;
         recordUpdate=1;
         if(itemTotalCount)                             // have records? if yes  display the first record
       	 startAdd=getStartAdd();
  	 }

	 if(memUpdateDisp){
	     if(itemTotalCount) {                            //if have record, display record
	    	 if(recordUpdate) {
	    		 signedCurrentItemAdd=startAdd-currentItemNum*8;
	    		 if(signedCurrentItemAdd>=0)
	    			 currentItemAdd=startAdd-currentItemNum*8;
	    		 else
	    			 currentItemAdd=0x0280+startAdd-currentItemNum*8;
		     getRecord(currentItemAdd);
		     recordUpdate=0;
	    	 }
		     displayRecord();
	      }
	     else {                                             //no record , display 0  at some position
		     timeFresh();                                  //change the values of month,day,hour,minute
		     displayDate(month,day);
		     displayTime(hour,minute);
		     displayZero();
 	         displaySetmem();
	      }
	     memUpdateDisp=0;
	 }

}

/**********************************************************************//**
 * @brief  displayMemInitState(): display the record total count and RTC about 4s
 * @param  none
 * @return none
 *************************************************************************/
void displayMemInitState(void)
{
	unsigned char fourSecCount;

	itemTotalCount = EEPROM_RandomRead(0x7ffd);      // Read from address 0xfffd,get the total count of records
    displayRecordCou(itemTotalCount);               // display record itemTotalCount at heart rate position and memory flag
    if(itemTotalCount==0){
    	displayZero();
    }
    fourSecCount=0;
	 while(fourSecCount<=5)                      // about 3s ,RTC is enable
	 {
		 fourSecCount++;
   	     timeFresh();                            //change the values of month,day,hour,minute
   	     displayDate(month,day);
   	     displayTime(hour,minute);
 	     displaySetmem();
   	     __bis_SR_register(LPM0_bits +GIE);     // RTC can wake up
	 }
}

/**********************************************************************//**
 * @brief  displayRecordCou():
 *         display the total count of records at heartRate position
 * @param  titemCount , the total count of records
 * @return none
 *************************************************************************/
void displayRecordCou(unsigned char titemCount)
{
	unsigned char t=0;
	unsigned char tt=0;
	t=titemCount;

	while(t>=10){
		t=t-10;
		tt++;
	}

   if(titemCount){
	   MEMP7=1;
	   MEMP8=1;
	  f_dispheartm=digit3_value[tt];
	  f_dispheartl=digit3_value[t];
   }
   else{
	   MEMP7=0;
	   MEMP8=0;
	  f_dispheartm=digit3_value[0];
	  f_dispheartl=digit3_value[0];
   }

}

