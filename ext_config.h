
#ifndef EXT_CONFIG_H_
#define EXT_CONFIG_H_

#include "msp430f6638.h"
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

extern volatile RECORD recordContent;


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

extern volatile BYTESTR BPM1    ;
extern volatile BYTESTR BPM2    ;
extern volatile BYTESTR BPM3    ;
extern volatile BYTESTR BPM4    ;
extern volatile BYTESTR BPM5    ;
extern volatile BYTESTR BPM6    ;
extern volatile BYTESTR BPM7    ;
extern volatile BYTESTR BPM8    ;
extern volatile BYTESTR BPM9    ;
extern volatile BYTESTR BPM10   ;
extern volatile BYTESTR BPM11   ;
extern volatile BYTESTR BPM12   ;
extern volatile BYTESTR BPM13   ;
extern volatile BYTESTR BPM14   ;
extern volatile BYTESTR BPM15   ;
extern volatile BYTESTR BPM16   ;
extern volatile BYTESTR BPM17   ;
extern volatile BYTESTR BPM18   ;

extern volatile BYTESTR MFLAG    ;
#define       batterylow                MFLAG.Bits.bit0    //
#define       batterymid                MFLAG.Bits.bit1   //
#define       batteryhigh               MFLAG.Bits.bit2    //
#define       TIlogo                    MFLAG.Bits.bit3    //
#define       kpa                       MFLAG.Bits.bit7    //
#define       mmhg                      MFLAG.Bits.bit4   //
#define       MEMP7                     MFLAG.Bits.bit5   //
#define       MEMP8                     MFLAG.Bits.bit6    //

extern volatile BYTESTR MFLAG1    ;
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
#define MEMSTACLOSE    0
#define LAST3SEC       1
#define MEMDISPRECORD  2

//measureSta choice
#define    MEASURESTACLOSE        0
#define    MEASUREDISPBEGIN       1
#define    MEASUREDISPAIRINQUICK  2
#define    MEASUREDISPAIRINSLOW   3
#define    MEASUREDISPAIROUT      4
#define    MEASUREDISPPRORESULT   5
#define    MEASUREDISPERROR       6
#define    MEASUREEND             7

#define SlaveAddress   0x50
#define I2C_PORT_SEL   P2SEL
#define I2C_PORT_OUT   P2OUT
#define I2C_PORT_REN   P2REN
#define I2C_PORT_DIR   P2DIR
#define SDA_PIN        BIT3                  // UCB0SDA pin
#define SCL_PIN        BIT2                  // UCB0SCL pin
#define SCL_CLOCK_DIV  0x0a               // SCL clock devider
#define MAXPAGEWRITE   64

extern const unsigned char digit1_value[];
extern const unsigned char digit2_value[];
extern const unsigned char digit3_value[];
extern const unsigned char digit4_value[];
extern const unsigned char digit5_value[];
extern const unsigned char digit6_value[];
extern const unsigned char digit7_value[];
extern const float KdTable[];
extern const float KsTable[];


extern void __RESET_WATCHDOG(void);
extern void _delayxSecond(unsigned char ttt);
extern void close_record(void);
extern void enableButton(void);
extern void disableButton(void);

/*********************VARIABLES DEFINE*********************************/
extern volatile char workMode;              //work mode:STANDBY, DATESET,MEMORY,MEASURE
extern volatile unsigned char workStatus;   //WORKSTATUS:STARTBUTTON,MEMORY BUTTON OR COMBINATION BUTTON


/*******************memory mode variable****************************/
extern volatile unsigned int E2PROMaddress;                     // define for E2PROM address
extern volatile unsigned char itemTotalCount;               // blood pressure record count
extern volatile unsigned int startAdd;                 // record the E2PROM address of the latest item
extern volatile unsigned char currentItemNum;
extern volatile unsigned char tempCurrentItemNum;
extern volatile unsigned char memorySta;
extern volatile unsigned char memUpdateDisp;
extern volatile unsigned char recordUpdate;

/*******************measure mode variable****************************/
extern volatile unsigned char measureSta;
extern volatile unsigned int preValue;             // the current pressure
extern volatile unsigned int ADCSampleCounter;
extern volatile unsigned char E2PROMPageCounter;
extern volatile unsigned char ADCStopFlag;
extern volatile unsigned char measureUpdateDisp;
extern volatile unsigned char dotFlashFlag;
extern unsigned char ReadWrite_val[8];
//extern volatile unsigned char totalE2PROMPage;
extern volatile unsigned int ADCDataCounter;
extern volatile unsigned char highPreCounter;

extern volatile unsigned int rawData[];
extern volatile unsigned char bpFilterData[];
//extern unsigned char filterFlag;
extern unsigned char endProFlag;

extern volatile char CheckSum ;
extern volatile char CheckSum1;
extern volatile char ADC_array[7];
extern volatile unsigned char ii,jj;
/*******************RTC&DATESET mode variable**************************/
extern volatile char AMPMmode;               // 0:24-hour mode;  1: 12-hour-mode
extern volatile char AM_flag;
extern volatile char PM_flag;
extern volatile char minute;
extern volatile char hour;
extern volatile char day;
extern volatile char month;
extern volatile char setPosition;           //0:AM/PM;1:MONTH;2:DAY;3:HOUR;4:MINUTE
extern volatile char freshFlag;
extern volatile char ms15Flag;               // 15ms timer flag, every 15ms to read the key value
extern volatile char errorCode;               // 15ms timer flag, every 15ms to read the key value


#endif /* EXT_CONFIG_H_ */


