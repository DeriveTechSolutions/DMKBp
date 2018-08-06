/*
 * measureMode.h
 *
 *  Created on: Feb 6, 2013
 *      Author: a0220221
 */

#ifndef MEASUREMODE_H_
#define MEASUREMODE_H_

void ADCInit(void);
void timer1Init(void);
void timer0Init(void);

void displayMeasure(void);
void displayMeasureInit(void);

void calHeartRate(void );
unsigned int getSystolicPosition(void);
unsigned int getDiastolicPosition(void);

void displayErrorPro(unsigned char errCode);
void displayErr(unsigned char errCode);
void displayAerate(void);
void displayDeflate(void);
void displayAirPre(unsigned char pressure);


#define     rawDataSAdd   0x0280   // start address of raw data    160*64/100---can store up to 102 seconds' sample data,160pages
#define     bpDataSAdd   0x2a80    // start address of band pass filtered  data
#define     lpDataSAdd   0x5280   // start address of low pass filtered  data

#define CUFF_ERR         0x00     //cuff is not OK
#define OVER_FLOW        0x01     //data over flow

#endif /* MEASUREMODE_H_ */
