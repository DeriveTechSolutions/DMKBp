/*
 * display.h
 *
 *  Created on: Feb 1, 2013
 *      Author: a0220221
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

void SET_LCD_ON(void);
void SET_LCD_OFF(void);
void displaySetmem(void);
void timeFresh(void);
void setTime(void);

void displayStandby(void);
void displayDateset(void);

void displayDate(unsigned char tmonth, unsigned char tday);
void displayTime(unsigned char thour, unsigned char tminute);
void displayZero(void );

void displayRecord(void);
void displayPressure(unsigned char shou, unsigned char shu, unsigned char heartR,unsigned char bflag);




#endif /* DISPLAY_H_ */
