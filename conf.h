/*
 * conf.h
 *
 *  Created on: Sep 22, 2013
 *      Author: a0220221
 */

#ifndef CONF_H_
#define CONF_H_


#define byte unsigned char
#define word unsigned int


typedef struct
{
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;
	unsigned char systolic;
	unsigned char diastolic;        // shuzhang
	unsigned char heartRate;
	unsigned char heartBad_flag;
}RECORD;
volatile RECORD recordContent ;


typedef union{
  byte Byte;
  struct {
      byte bit0       :1;
      byte bit1       :1;
      byte bit2       :1;
      byte bit3       :1;
      byte bit4       :1;
      byte bit5       :1;
      byte bit6       :1;
      byte bit7       :1;
    } Bits;
}BYTESTR;

volatile BYTESTR BPM1    ;
volatile BYTESTR BPM2    ;
volatile BYTESTR BPM3    ;
volatile BYTESTR BPM4    ;
volatile BYTESTR BPM5    ;
volatile BYTESTR BPM6    ;
volatile BYTESTR BPM7    ;
volatile BYTESTR BPM8    ;
volatile BYTESTR BPM9    ;
volatile BYTESTR BPM10   ;
volatile BYTESTR BPM11   ;
volatile BYTESTR BPM12   ;
volatile BYTESTR BPM13   ;
volatile BYTESTR BPM14   ;
volatile BYTESTR BPM15   ;
volatile BYTESTR BPM16   ;
volatile BYTESTR BPM17   ;
volatile BYTESTR BPM18   ;

//below definition is used for judge
volatile BYTESTR MFLAG    ;
#define       batterylow                MFLAG.Bits.bit0    //
#define       batterymid                MFLAG.Bits.bit1   //
#define       batteryhigh               MFLAG.Bits.bit2    //
#define       TIlogo                    MFLAG.Bits.bit3    //
#define       kpa                       MFLAG.Bits.bit7    //
#define       mmhg                      MFLAG.Bits.bit4   //
#define       MEMP7                     MFLAG.Bits.bit5   //memory display p7
#define       MEMP8                     MFLAG.Bits.bit6    //memory display p8

volatile BYTESTR MFLAG1    ;
#define       coupleDots                MFLAG1.Bits.bit0    //
#define       sanjiao                   MFLAG1.Bits.bit1   //

// below definition is used for operation LCDMEM
#define       f_dispbatterylow                 BPM1.Bits.bit0    //
#define       f_dispbatterymid                 BPM1.Bits.bit1
#define       f_dispbatteryhigh                BPM1.Bits.bit2
#define       f_displogo                       BPM1.Bits.bit3
#define       f_disp1b                         BPM1.Bits.bit4    //
#define       f_disp1aged                      BPM1.Bits.bit5    //
#define       f_disp1c                         BPM1.Bits.bit6
#define       f_dispkpa                        BPM1.Bits.bit7
#define       f_dispmonthh                     BPM1.Byte

#define       f_dispmonthl                           BPM2.Byte
#define       f_dispmonCday                          BPM2.Bits.bit7
#define       f_dispdayh                             BPM3.Byte    // mmhg
#define       f_dispmmhg                             BPM3.Bits.bit7
#define       f_dispdayl                             BPM4.Byte    // p7
#define       f_dispmemp7                            BPM4.Bits.bit7

#define       f_disphourh                      BPM5.Byte    //
#define       f_dispsanjiao                    BPM5.Bits.bit3  // fangqi

#define       f_disphourl                      BPM6.Byte   //  diandian--yes
#define       f_dispcoupledots                 BPM6.Bits.bit3
#define       f_dispminuteh                    BPM7.Byte  // pm
#define       f_disppm                         BPM7.Bits.bit3
#define       f_dispminutel                    BPM8.Byte   // am
#define       f_dispam                         BPM8.Bits.bit3

#define       f_disphpreh                      BPM9.Byte  // p8
#define       f_dispmemp8                      BPM9.Bits.bit7
#define       f_disphprem                      BPM10.Byte  //p1--no
#define       f_disphprel                      BPM11.Byte  //p2--no
//#define       f_dispp2                         BPM11.Bits.bit3

#define       f_displpreh                      BPM12.Byte  // heart logo
#define       f_dispheartlogo                  BPM12.Bits.bit7
#define       f_displprem                      BPM13.Byte  //p3--no
#define       f_displprel                      BPM14.Byte  //p4--no
//#define       f_dispp4                         BPM14.Bits.bit4

#define       f_disphearth                     BPM15.Byte  // L14--no
#define       f_dispheartm                     BPM16.Byte  //p5--no
#define       f_dispheartl                     BPM17.Byte  //p6--no

//#define BATTERY_HIGH  80//819   //1024*1.2/1.5
#define BATTERY_MID   64 //614   //1024*0.9/1.5
#define BATTERY_LOW   45//477  //1024*0.7/1.5

// workMode choice
#define STANDBY 0
#define DATESET 1
#define MEASURE 2
#define MEMORY  3

//memorySta choice
#define MEMSTACLOSE   0
#define LAST3SEC      1
#define MEMDISPRECORD 2

//measureSta choice
#define    MEASURESTACLOSE        0
#define    MEASUREDISPBEGIN       1
#define    MEASUREDISPAIRINQUICK  2
#define    MEASUREDISPAIRINSLOW   3
#define    MEASUREDISPAIROUT      4
#define    MEASUREDISPPRORESULT   5
#define    MEASUREDISPERROR       6
#define    MEASUREEND             7

#define SlaveAddress  0x50
#define I2C_PORT_SEL  P2SEL
#define I2C_PORT_OUT  P2OUT
#define I2C_PORT_REN  P2REN
#define I2C_PORT_DIR  P2DIR
#define SDA_PIN       BIT3                  // UCB0SDA pin
#define SCL_PIN       BIT2                  // UCB0SCL pin
#define SCL_CLOCK_DIV 0x0a//0x28                  // SCL clock devider
#define MAXPAGEWRITE   64


// ------------------for date display---------------------
const unsigned char digit6_value[2]=                    //BPM1-dispmonthh
{
	0x00,	//0
	0x50,	//1
};
const unsigned char digit1_value[10]=                    //BPM2,3,4
{
	0x5f,	//0
	0x50,	//1
	0x3d, //2
	0x79,//3
	0x72,//4
	0x6b,//5
	0x6f,//6
	0x51,//7
	0x7f,//8
	0x7b//9
};

//-------------------for time display-----------------------
const unsigned char digit5_value[3]=                    //BPM5-disphourh
{
	0x00,	//0
	0x05,	//1
	0x03,   //2
};
const unsigned char digit7_value[10]=                 //BPM6,7,8
{
		0xf5,  //0
		0x05, //1
		0xd3,//2
		0x97,//3
		0x27,//4
		0xb6,//5
		0xf6,//6
		0x15,//7
		0xf7,//8
		0xb7//9
};
const unsigned char digit3_value[10]=                    //BPM13--17--0--9
{
	0xaf,	//0
	0x06,	//1
	0x6d,  //2
	0x4f, //3
	0xc6, //4
	0xcb,//5
	0xeb,//6
	0x0e,//7
	0xef,//8
	0xcf//9
};

const unsigned char digit2_value[10]=                    //BPM9-11:0--9
{
	0x5f,	//0
	0x06,	//1
	0x6b, //2
	0x2f,//3
	0x36,//4
	0x3d,//5
	0x7d,//6
	0x07,//7
	0x7f,//8
	0x3f//9
};

const unsigned char digit4_value[10]=                    //BPM12:0--9
{
	0x5f,	//0
	0x06,	//1
	0x3d, //2
	0x2f,//3
	0x66,//4
	0x6b,//5
	0x7b,//6
	0x0e,//7
	0x7f,//8
	0x6f//9
};

const float KsTable[7]=
{
		0.5, // average pressure >200
		0.29, //150~200
		0.45, //135~150
		0.52,// 120~135
		0.57, //110~120
		0.58,//70~110
		0.64 //~~70

};
const float KdTable[6]=
{
 0.75, //180~
 0.82, //140~180
 0.85, //120~140
 0.78, //60~120
 0.6, //50~60
 0.5 //~~50
};

void __RESET_WATCHDOG(void);
void _delayxSecond(unsigned char ttt);
void close_record(void);
void enableButton(void);
void disableButton(void);

volatile char workMode;               // difine diffirent work mode
volatile unsigned char workStatus;    //0: two button pressed; 1; start button pressed; 2: memory button pressed

volatile char CheckSum ;
volatile char CheckSum1;
volatile char ADC_array[7];
volatile unsigned char ii,jj;

//------memory mode variable---------------------------------
volatile unsigned int E2PROMaddress;            // define for E2PROM address
volatile unsigned char itemTotalCount;          // blood pressure record count
volatile unsigned int startAdd;                 // record the E2PROM address of the latest item
volatile unsigned char currentItemNum;
volatile unsigned char memorySta;
volatile unsigned char memUpdateDisp;
volatile unsigned char recordUpdate;

//------measure mode variable---------------------------------
volatile unsigned char measureSta;
volatile unsigned int preValue;             // the current pressure
volatile unsigned int ADCSampleCounter;
volatile unsigned char ADCStopFlag;
volatile unsigned char measureUpdateDisp;
volatile unsigned char dotFlashFlag;
unsigned char ReadWrite_val[8];
//volatile unsigned char totalE2PROMPage;
volatile unsigned int ADCDataCounter;
volatile unsigned char highPreCounter;     // this indicate the times that the pressure of cuff over 120mmhg

//unsigned char filterFlag;
unsigned char endProFlag;

#pragma DATA_SECTION(rawData, ".myData1")
volatile unsigned int rawData[3000];
#pragma DATA_SECTION(bpFilterData, ".myData2")
volatile unsigned char bpFilterData[3000];

//------RTC and time set function variable--------------------
volatile char AMPMmode;               // 12 hour mode .half_mode=0.24-hour mode;  1: 12-hour-mode
volatile char AM_flag;
volatile char PM_flag;
volatile char minute;
volatile char hour;
volatile char day;
volatile char month;
volatile char setPosition;            //0:AM/PM;1:MONTH;2:DAY;3:HOUR;4:MINUTE
volatile char freshFlag;              // 0.5s fresh
volatile char ms15Flag;               // 15ms timer flag, every 15ms to read the key value
volatile char errorCode;


#endif /* CONF_H_ */
