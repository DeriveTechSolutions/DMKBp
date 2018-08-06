/*
 * UCB0_I2C.h
 *
 *  Created on: Aug 30, 2013
 *      Author: a0220221
 */

#ifndef UCB0_I2C_H_
#define UCB0_I2C_H_


void I2CInit(void);
void InitI2C(unsigned char eeprom_i2c_address);
void I2CReadInit(void);
void I2CWriteInit(void);
void EEPROM_ByteWrite(unsigned int Address , unsigned char Data);
void EEPROM_PageWrite(unsigned int StartAddress , unsigned char * Data , unsigned int Size);
unsigned char EEPROM_RandomRead(unsigned int Address);
unsigned char EEPROM_CurrentAddressRead(void);
void EEPROM_SequentialRead(unsigned int Address , unsigned char * Data , unsigned int Size);
void EEPROM_AckPolling(void);

unsigned int getStartAdd(void);
void getRecord(unsigned int tempAdd);


#endif /* UCB0_I2C_H_ */
