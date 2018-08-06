/*
 * Algorithm.h
 *
 *  Created on: Sep 25, 2013
 *      Author: a0220221
 */

#ifndef ALGORITHM_H_
#define ALGORITHM_H_


void dataProcess(void);                          //process rawdata to get bandpass filtered data
void dataProcessInit(void);
void lpFilterData(void);                         // low pass filter filtered data
unsigned int getAveragePressure(void);                //get max value from filtered data
unsigned char getHeartrate(unsigned int ppCont);      //get heart rate from filtered data ????
unsigned int getSysPos(void);                         //get the position of systolic pressure from filtered data
unsigned int getDiasPos(void);                        //get the position of diastolic pressure from filtered data

unsigned char funLeftJudge(unsigned int index);
unsigned char funRightJudge(unsigned int index);
void leftResort(unsigned char index);
void rightResort(unsigned char index);


#endif /* ALGORITHM_H_ */
