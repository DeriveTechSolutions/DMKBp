
#include "ext_config.h"
#include "display.h"
#include "key.h"
#include "UCB0_I2C.h"

volatile unsigned char tempCurrentItemNum;     //used in total item count displayed

/**********************************************************************//**
 * @brief  SET_LCD_ON(): display all LCD segment
 * set LCD memory LCDMEM all as 1
 * @param  none
 * @return none
 *************************************************************************/
void SET_LCD_ON(void)
{
	LCDM1=0x7f;
	LCDM2=0x7f;
	LCDM3=0xff;
	LCDM4=0xff;
	LCDM5=0xff;
	LCDM6=0xff;
	LCDM7=0xff;
	LCDM8=0xff;
	LCDM9=0xff;
	LCDM10=0x7f;
	LCDM11=0xff;
	LCDM12=0xfe;
	LCDM13=0xfe;
	LCDM14=0xff;
	LCDM15=0xfe;
	LCDM16=0xfe;
	LCDM17=0x0e;
}

/**********************************************************************//**
 * @brief  SET_LCD_OFF(): close all LCD segment
 * Set LCD memory all as 0, close LCD
 * @param  none
 * @return none
 *************************************************************************/
void SET_LCD_OFF(void)
{
	LCDM1=0x00;
	LCDM2=0x00;
	LCDM3=0x00;
	LCDM4=0x00;
	LCDM5=0x00;
	LCDM6=0x00;
	LCDM7=0x00;
	LCDM8=0x00;
	LCDM9=0x00;
	LCDM10=0x00;
	LCDM11=0x00;
	LCDM12=0x00;
	LCDM13=0x00;
	LCDM14=0x00;
	LCDM15=0x00;
	LCDM16=0x00;
	LCDM17=0x00;
	LCDM18=0X00;
	LCDM19=0X00;
	LCDM20=0X00;
}


/**********************************************************************//**
 * @brief  Set LCD memory registers
 * @param  none
 * @return none
 *************************************************************************/
void displaySetmem(void)
{
LCDM1=f_disphprel;//BPM11.Byte;
LCDM2=f_disphprem;//BPM10.Byte;
LCDM3=f_disphpreh;//BPM9.Byte;
LCDM4=f_dispminutel;//BPM8.Byte;
LCDM5=f_dispminuteh;//BPM7.Byte;
LCDM6=f_disphourl;//BPM6.Byte;
LCDM7=(f_disphourh&0x0f)+(f_dispdayl&0xf0);//(BPM5.Byte&0x0f)+(BPM4.Byte&0xf0);
LCDM8=(f_dispdayl&0x0f)+(f_dispdayh&0xf0);//(BPM4.Byte&0x0f)+(BPM3.Byte&0xf0);
LCDM9=(f_dispdayh&0x0f)+(f_dispmonthl&0xf0);//(BPM3.Byte&0x0f)+(BPM2.Byte&0xf0);
LCDM10=(f_dispmonthl&0x0f)+(f_dispmonthh&0xf0);//(BPM2.Byte&0x0f)+(BPM1.Byte&0xf0);
LCDM11=(f_dispmonthh&0x0f)+(f_displprel<<4);//(BPM1.Byte&0x0f)+(BPM14.Byte&0x0f);
LCDM12=(f_displprel>>4)+(f_displprem<<4);//(BPM14.Byte>>4)+(BPM13.Byte<<4);
LCDM13=(f_displprem>>4)+(f_displpreh<<4);//(BPM13.Byte>>4)+(BPM12.Byte<<4);
LCDM14=(f_displpreh>>4)+(f_dispheartl<<4);//(BPM12.Byte>>4)+(BPM17.Byte<<4);
LCDM15=(f_dispheartl>>4)+(f_dispheartm<<4);//(BPM17.Byte>>4)+(BPM16.Byte<<4);
LCDM16=(f_dispheartm>>4)+(f_disphearth<<4);//(BPM16.Byte>>4)+(BPM15.Byte<<4);
LCDM17=(f_disphearth>>4)&0x0f;//(BPM15.Byte>>4)&0x0f;
}

/**********************************************************************//**
 * @brief  timeFresh, Fresh the value of date and time
 * @param  none
 * @return none
 *************************************************************************/
void timeFresh(void)
{
	 if(minute>=60){
		 hour++;
		 minute=0;
	 }
	 if(hour>=24){
		 day++;
		 hour=0;
	 }
	 if(day>=31&&(month==4||month==6||month==9||month==11)){
	     month++;
	     day=1;
	 }
	 if(day>=32&&(month==1||month==3||month==5||month==7||month==8||month==10||month==12)){
	     month++;
	     day=1;
	 }
	 if (day>=29&&month==2){
		month++;
		day=1;
	 }
	 if(month>=13)
		 month=1;
}


/**********************************************************************//**
 * @brief  setTime(): date and time set function
 * @param  none
 * @return none
 *************************************************************************/
void setTime(void)
{
	switch(setPosition)
	{
	case MONTH:                               //  month fresh
		if(freshFlag)
		{
			f_dispmonthh&=0x8f;
		    f_dispmonthl&=0x80;
		}
		break;

	case DAY:                               // day fresh
		if(freshFlag)
		{
			f_dispdayh&=0x80;
		    f_dispdayl&=0x80;
		}
		break;

	case HOUR:                              // hour fresh
		if(freshFlag)
		{
			f_disphourh&=0x08;
			f_disphourl&=0x08;
		}
		break;

	case MINUTE:                              // minute fresh
		if(freshFlag)
		{
			f_dispminuteh&=0x08;
			f_dispminutel&=0x08;
		}

		break;
	}

}


/**********************************************************************//**
 * @brief  displayPressure(): DISPLAY record content and heart rate unnormal flag if it is
 *
 * @param  systolic
 * @param  diastolic
 * @param  heart rate
 * @param  heartBad_flag   1: heart rate unnormal, 0: heart rate normal---f_dispheartlogo bit
 *
 * @return none
 *************************************************************************/
void displayPressure(unsigned char systolic, unsigned char diastolic, unsigned char heartRate, unsigned char heartBad_flag)
{
   unsigned char tempValue=0;
   close_record();
//----------------------------for systolic pressure value ---------
   while(systolic>=100){
	   systolic=systolic-100;
	   tempValue++;
   }
	f_disphpreh=digit2_value[tempValue];
	tempValue=0;
	while(systolic>=10){
	   systolic=systolic-10;
	   tempValue++;
   }
	f_disphprem=digit2_value[tempValue];
	f_disphprel=digit2_value[systolic];
//----------------------------for diastolic pressure value ---------
	tempValue=0;
	   while(diastolic>=100){
		   diastolic=diastolic-100;
		   tempValue++;
	   }
		f_displpreh=digit4_value[tempValue];
		tempValue=0;
		while(diastolic>=10){
		   diastolic=diastolic-10;
		   tempValue++;
	   }
		f_displprem=digit3_value[tempValue];
		f_displprel=digit3_value[diastolic];
//--------------------------for heart rate/itemNum value display -- in memory mode
	if((workMode==MEMORY) ||((measureSta==MEASUREDISPBEGIN)&&(workMode==MEASURE) )) {
		tempValue=0;
		//------------------ display heart Rate-------------
		if(freshFlag) {
		   while(heartRate>=100){
			   heartRate=heartRate-100;
			   tempValue++;
		   }
			f_disphearth=digit3_value[tempValue];
			tempValue=0;
			while(heartRate>=10){
				heartRate=heartRate-10;
			   tempValue++;
		   }
			f_dispheartm=digit3_value[tempValue];
			f_dispheartl=digit3_value[heartRate];
		}
		//------------------ display record number-------------
		else {
			f_disphearth=0;
			tempCurrentItemNum=currentItemNum+1;
			while(tempCurrentItemNum>=10){
				tempCurrentItemNum=tempCurrentItemNum-10;
				tempValue++;
			}
			f_dispheartm=digit3_value[tempValue];
			f_dispheartl=digit3_value[tempCurrentItemNum];
		}
	}
//for heart rate value display  in measure mode
	if((workMode==MEASURE)&&(measureSta!=MEASUREDISPBEGIN)) {
		tempValue=0;
		   while(heartRate>=100){
			   heartRate=heartRate-100;
			   tempValue++;
		   }
			f_disphearth=digit3_value[tempValue];
			tempValue=0;
			while(heartRate>=10){
				heartRate=heartRate-10;
			   tempValue++;
		   }
			f_dispheartm=digit3_value[tempValue];
			f_dispheartl=digit3_value[heartRate];
	}

//	if() {                                       //mmhg mode
	 f_dispmmhg=1;
//	 f_dispkpa=0;

//	}
//	else {                                     // kpa mode
//		 f_dispmmhg=0;
//		 f_dispkpa=1;
//	}

	 if(MEMP7) {                              //need to display memory picture
		 f_dispmemp7=1;
		 f_dispmemp8=1;
	 }

	if(heartBad_flag)
	f_dispheartlogo=1;
	else
	f_dispheartlogo=0;

}

/**********************************************************************//**
 * @brief  close_record(): close all the display except RTC
 *
 * @param  none
 *
 * @return none
 *************************************************************************/
void close_record(void)
{
	f_dispheartl=0;
	f_dispheartm=0;
	f_disphearth=0;

	f_displpreh=0;
	f_displprem=0;
	f_displprel=0;

	f_disphpreh=0;
	f_disphprem=0;
	f_disphprel=0;


	f_dispmmhg=0;
	f_dispkpa=0;
	f_dispsanjiao=0;


    batterylow=0;
    batterymid=0;
    batteryhigh=0;
    TIlogo=0;

  if(workMode==STANDBY)
  {
	  MEMP7=0;
	  MEMP8=0;
	    mmhg=0;
	    kpa=0;
  }

}


/**********************************************************************//**
 * @brief  displayTime(): display hour and minute
 *
 * @param  thour, the time of hour
 * @param  tminute, the time of minute
 *
 * @return none
 *************************************************************************/
void displayTime(unsigned char thour, unsigned char tminute)
{
	unsigned char mode1=0;
	unsigned char temp_minute;
	//--------------hour display------------------------------
	if(AMPMmode)                                   //12-hour -mode
	{
		if(0<=thour&&thour<12){
			AM_flag=1;
			PM_flag=0;
		}
		else{
			PM_flag=1;
			AM_flag=0;
		}

		if(0<=thour&&thour<10){
			f_disphourh=digit5_value[0];
			f_disphourl=digit7_value[thour];
		}
		else if(10<=thour&&thour<12){
			f_disphourh=digit5_value[1];
			f_disphourl=digit7_value[thour-10];
		}
		else if(12<=thour&&thour<22){
			f_disphourh=digit5_value[0];
			f_disphourl=digit7_value[thour-12];
		}
		else{
			f_disphourh=digit5_value[1];
			f_disphourl=digit7_value[thour-22];
		}
	}

	else {                                    //24-hour -mode
	     if(0<=thour&&thour<10){
		    f_disphourh=digit5_value[0];
		    f_disphourl=digit7_value[thour];
	    }
	     else if(10<=thour&&thour<20){
		    f_disphourh=digit5_value[1];
		    f_disphourl=digit7_value[thour-10];
	    }
	     else{
		    f_disphourh=digit5_value[2];
		    f_disphourl=digit7_value[thour-20];
	    }

	}

	//-----------------------minute display-------------------------
	temp_minute=tminute;
	while(temp_minute>=10){
	temp_minute=temp_minute-10;
	mode1++;
	}
	f_dispminuteh=digit7_value[mode1];
	if(PM_flag)
	    f_disppm=1;
	else
		f_disppm=0;
	f_dispminutel=digit7_value[temp_minute];
	if(AM_flag)
	    f_dispam=1;
	else
		f_dispam=0;
	if(workMode==STANDBY)  {         // standby mode
	   if(freshFlag)
		   f_dispcoupledots=1;
	  else
		  f_dispcoupledots=0;
	}
	else if(workMode==DATESET)       //   date set mode not flash
		f_dispcoupledots=1;
	else if(workMode==MEASURE)  {    //measure mode
		   if(dotFlashFlag)          //display the last record
			   f_dispcoupledots=1;
		   else{
			   if(freshFlag)
				   f_dispcoupledots=1;
			  else
				  f_dispcoupledots=0;
		   }
	}
	else  {                     //memory mode
	     if(memorySta==LAST3SEC){
		     if(freshFlag)
		    	 f_dispcoupledots=1;
		     else
		    	 f_dispcoupledots=0;
	       }
	     else{
	    	 if (itemTotalCount)                // have record
	    		 f_dispcoupledots=1;
	    	 else  {                       // no record display zero
			     if(freshFlag)
			    	 f_dispcoupledots=1;
			     else
			    	 f_dispcoupledots=0;
	    	 }
	      }
	}

	//f_dispcoupledots=freshFlag;
    f_dispsanjiao=sanjiao;

}

/**********************************************************************//**
 * @brief  displayDate(): display month and day,  give correct value tovariable
 *
 * @param  tmonth, the date of month
 * @param  tday, the date of day
 *
 * @return none
 *************************************************************************/
void displayDate(unsigned char tmonth, unsigned char tday)
{
    if(0<tmonth&&tmonth<10)
    {
    	f_dispmonthh=digit6_value[0] ;
        f_dispmonthl=digit1_value[tmonth] ;
    }
    else
    {
    	f_dispmonthh =digit6_value[1] ;
        f_dispmonthl=digit1_value[tmonth-10] ;
    }


    if(0<tday&&tday<10)
    {
    	f_dispdayh =digit1_value[0] ;
    	f_dispdayl=digit1_value[tday] ;
    }
    else if(10<=tday && tday<20)
    {
    	f_dispdayh =digit1_value[1] ;
    	f_dispdayl=digit1_value[tday-10] ;
    }
    else if(20<=tday && tday<30)
    {
    	f_dispdayh =digit1_value[2] ;
    	f_dispdayl=digit1_value[tday-20] ;
    }
    else
    {
    	f_dispdayh =digit1_value[3] ;
    	f_dispdayl=digit1_value[tday-30] ;
    }

    f_dispmonCday=1;           //'-'
    f_dispbatterylow=batterylow;
    f_dispbatterymid=batterymid;
    f_dispbatteryhigh=batteryhigh;
    f_dispmmhg=mmhg;
    f_dispmemp7=MEMP7;
    f_displogo=TIlogo;
    f_dispkpa=kpa;
    f_dispmemp8=MEMP8;

}

/*********************************************************************
 * @brief displayRecord : display the record content
 *
 * @param  none
 *
 * @return none
**********************************************************************/
void displayRecord(void)
{
    displayDate(recordContent.month,recordContent.day);
    displayTime(recordContent.hour,recordContent.minute);
    displayPressure(recordContent.systolic,recordContent.diastolic,recordContent.heartRate,recordContent.heartBad_flag);

    displaySetmem();
}

/*********************************************************************
 * @brief displayZero : display zero
 *
 * @param  none
 *
 * @return none
**********************************************************************/
void displayZero(void )
{
	close_record();
	f_displprel=digit3_value[0];
	f_disphprel=digit2_value[0]; //display zero

	f_dispheartm=digit3_value[0];
	f_dispheartl=digit3_value[0];


}


/*********************************************************************
 * @brief displayStandby : display RTC mode
 *
 * @param  none
 *
 * @return none
**********************************************************************/
void displayStandby(void)
{
	//timeFresh();                              // change the values of month,day,hour,minute
	displayDate(month,day);
	displayTime(hour,minute);
	displaySetmem();
}

/*********************************************************************
 * @brief displayStandby : display dateset mode
 *
 * @param  none
 *
 * @return none
**********************************************************************/
void displayDateset(void)
{
	// timeFresh();                              //change the values of month,day,hour,minute
	 displayDate(month,day);
	 displayTime(hour,minute);
	 setTime();                                // fresh function
	 displaySetmem();
}



