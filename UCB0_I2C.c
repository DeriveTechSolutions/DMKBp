/*
 * UCB0_I2C.c
 *
 *  Created on: Aug 30, 2013
 *      Author: a0220221
 */

#include "ext_config.h"
#include "UCB0_I2C.h"

volatile int PtrTransmit;
volatile unsigned char I2CBufferArray[66];
volatile unsigned char I2CBuffer;

volatile unsigned char Addh;
volatile unsigned char Addl;

/*********************************************************************
 * @brief getStartAdd : get the start address of record total count placed
 * @param  none
 * @return startAdd  the address of the total records placed
**********************************************************************/
unsigned int getStartAdd(void)
{
	unsigned int tempstartAdd;
    disableButton();
    Addh=EEPROM_RandomRead(0x7ffe);
    Addl=EEPROM_RandomRead(0x7fff);
    tempstartAdd=(((unsigned int) (Addh))<<8)+((unsigned int)(Addl));                     // got the latest record address
    enableButton();
    return tempstartAdd;
}

/*********************************************************************
 * @brief getRecord : get the record content
 * @param  the start address of the record
 * @return none
**********************************************************************/
void getRecord(unsigned int tempAdd)
{
	disableButton();
    EEPROM_SequentialRead(tempAdd, ReadWrite_val, sizeof(ReadWrite_val));
    enableButton();
    recordContent.month=ReadWrite_val[0];
    recordContent.day=ReadWrite_val[1];
    recordContent.hour=ReadWrite_val[2];
    recordContent.minute=ReadWrite_val[3];
    recordContent.systolic=ReadWrite_val[4];
    recordContent.diastolic=ReadWrite_val[5];
    recordContent.heartRate=ReadWrite_val[6];
    recordContent.heartBad_flag=ReadWrite_val[7];
}
/**********************************************************************//**
 * @brief  I2CInit():Assign pins to I2C bus
 * @param  none
 * @return none
 *************************************************************************/
 void I2CInit(void)
 {
	 unsigned char m=10;
   //configure I2C bus
	  I2C_PORT_SEL|=SCL_PIN+SDA_PIN;
	  I2C_PORT_DIR|=SCL_PIN+SDA_PIN;
      PMAPPWD = 0x02D52;          // Enable Write-access to modify port mapping registers
      PMAPCTL = PMAPRECFG;        // Allow reconfiguration during runtime
      P2MAP2 = PM_UCB0SCL;        //P2.2---UCB0SCL
      P2MAP3 = PM_UCB0SDA;        //P2.3---UCB0SDA
      PMAPPWD = 0;                // Disable Write-Access to modify port mapping registers
     //STOP I2C TO INIT THE I2C TO STOP STATUS
      while (m>=1)  {
	    I2C_PORT_SEL&=~(SCL_PIN+SDA_PIN);
	    I2C_PORT_OUT |=SCL_PIN;
	    I2C_PORT_OUT &=~SDA_PIN;
	    I2C_PORT_DIR|=SCL_PIN+SDA_PIN;
	    __delay_cycles(20);
	    I2C_PORT_OUT |=SCL_PIN;
	    I2C_PORT_OUT |=SDA_PIN;
	    I2C_PORT_DIR|=SCL_PIN+SDA_PIN;
	    __delay_cycles(20);
	    m=m-1;
      }
	  I2C_PORT_SEL|=SCL_PIN+SDA_PIN;
	  I2C_PORT_DIR|=SCL_PIN+SDA_PIN;
 }

 /**********************************************************************//**
  * @brief  InitI2C():Initialization of the I2C Module
  * @param  eeprom_i2c_address:  SlaveAddress   0x50
  * @return none
  *************************************************************************/
 void InitI2C(unsigned char eeprom_i2c_address)
 {
   // Recommended initialisation steps of I2C module as shown in User Guide:
   UCB0CTL1 |= UCSWRST;                      // Enable SW reset
   UCB0CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
   UCB0CTL1 = UCSSEL_2 + UCSWRST;            // Use SMCLK, TX mode, keep SW reset
   UCB0BR0 = SCL_CLOCK_DIV;                   // fSCL = SMCLK/10 = ~400kHz
   UCB0BR1 = 0;
   UCB0I2CSA  = eeprom_i2c_address;          // define Slave Address
                                             // In this case the Slave Address
                                             // defines the control byte that is
                                             // sent to the EEPROM.
  // UCB0I2COA = 0x01A5;                     // own address.
   UCB0CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation

   if (UCB0STAT & UCBBUSY)                   // test if bus to be free
   {                                         // otherwise a manual Clock on is
                                             // generated
	    I2C_PORT_SEL &= ~SCL_PIN;               // Select Port function for SCL;p6sel
	    I2C_PORT_OUT &= ~SCL_PIN;
	    I2C_PORT_DIR |= SCL_PIN;                // drive SCL low;p6dir
	    I2C_PORT_SEL |= SDA_PIN + SCL_PIN;      // select module function for the
	                                            // used I2C pins
   }

 }

 /**********************************************************************//**
  * @brief   Initialization of the I2C Module for Write operation.
  * @param  none
  * @return none
  *************************************************************************/

 void I2CWriteInit(void)
 {
	 UCB0CTL1 |= UCSWRST;
	 UCB0CTL1 |=UCTR;
	 UCB0IFG &= ~UCTXIFG;
	 UCB0CTL1 &=~ UCSWRST;
	 UCB0IE &=~ UCRXIE;                         // Disnable RX interrupt
	 UCB0IE |= UCTXIE;
 }

 /**********************************************************************//**
  * @brief   Initialization of the I2C Module for Read operation.
  * @param  none
  * @return none
  *************************************************************************/
 void I2CReadInit(void)
 {
	 UCB0CTL1 |= UCSWRST;
	 UCB0CTL1 &=~UCTR;
	 UCB0IFG &= ~UCRXIFG;
	 UCB0CTL1 &=~ UCSWRST;
	 UCB0IE &=~ UCTXIE;                         // Disnable RX interrupt
	 UCB0IE |= UCRXIE;                  // Enable TX interrupt,acknowledge interrupt
 }


 /**********************************************************************//**
  * @brief  Byte Write Operation. The communication via the I2C bus with an EEPROM
  *  (2465) is realized. A data byte is written into a user defined address.
  * @param  Address : the address that will be written data to
  * @param  Data    : the data will be written
  * @return none
  *************************************************************************/
 void EEPROM_ByteWrite(unsigned int Address, unsigned char Data)
 {
   unsigned char adr_hi;
   unsigned char adr_lo;

   while (UCB0STAT & UCBUSY);                // wait until I2C module has
                                             // finished all operations.
   adr_hi = Address >> 8;                    // calculate high byte
   adr_lo = Address & 0xFF;                  // and low byte of address

   I2CBufferArray[2] = adr_hi;               // Low byte address.
   I2CBufferArray[1] = adr_lo;               // High byte address.
   I2CBufferArray[0] = Data;
   PtrTransmit = 2;                          // set I2CBufferArray Pointer

   I2CWriteInit();
   UCB0CTL1 |= UCTXSTT;                      // start condition generation
	                                         // => I2C communication is started
   __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts
   __no_operation();

   UCB0CTL1 |= UCTXSTP;                      // I2C stop condition
   while(UCB0CTL1 & UCTXSTP);               // Ensure stop condition got sent

 }

 /**********************************************************************//**
  * @brief  Page Write Operation. The communication via the I2C bus with an EEPROM
  *   (24xx65) is realized. A data byte is written into a user defined address.
  *
  * @param  StartAddress : the start address that will be written data to
  * @param  * Data    : the pointer that point the start address of the data will be written
  * @para   Size      : the size of the data will be written
  *
  * @return none
  *************************************************************************/

 void EEPROM_PageWrite(unsigned int StartAddress, unsigned char * Data, unsigned int Size)
 {
   volatile unsigned int i = 0;
   volatile unsigned char counterI2cBuffer;
   unsigned char adr_hi;
   unsigned char adr_lo;
   unsigned int currentAddress = StartAddress;
   unsigned int currentSize = Size;
   unsigned int bufferPtr = 0;
   unsigned char moreDataToRead = 1;

   while (UCB0STAT & UCBUSY);                // wait until I2C module has
                                             // finished all operations.

   // Execute until no more data in Data buffer
   while(moreDataToRead)
   {
     adr_hi = currentAddress >> 8;           // calculate high byte
     adr_lo = currentAddress & 0xFF;         // and low byte of address

     // Chop data down to 64-byte packets to be transmitted at a time
     // Maintain pointer of current startaddress
     if(currentSize > MAXPAGEWRITE)
     {
       bufferPtr = bufferPtr + MAXPAGEWRITE;
       counterI2cBuffer = MAXPAGEWRITE - 1;
       PtrTransmit = MAXPAGEWRITE + 1;       // set I2CBufferArray Pointer
       currentSize = currentSize - MAXPAGEWRITE;
       currentAddress = currentAddress + MAXPAGEWRITE;

       // Get start address
       I2CBufferArray[MAXPAGEWRITE + 1] = adr_hi; // High byte address.
       I2CBufferArray[MAXPAGEWRITE] = adr_lo; // Low byte address.
     }
     else
     {
       bufferPtr = bufferPtr + currentSize;
       counterI2cBuffer = currentSize - 1;
       PtrTransmit = currentSize + 1;        // set I2CBufferArray Pointer.
       moreDataToRead = 0;
       currentAddress = currentAddress + currentSize;

       // Get start address
       I2CBufferArray[currentSize + 1] = adr_hi; // High byte address.
       I2CBufferArray[currentSize] = adr_lo; // Low byte address.
     }

     // Copy data to I2CBufferArray
     unsigned char temp;
     for(i ; i < bufferPtr ; i++)
     {
       temp = Data[i];                       // Required or else IAR throws a
                                             // warning [Pa082]
       I2CBufferArray[counterI2cBuffer] = temp;
       counterI2cBuffer--;
     }

     I2CWriteInit();
     UCB0CTL1 |= UCTXSTT;                    // start condition generation
                                             // => I2C communication is started
     __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
     UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
     while(UCB0CTL1 & UCTXSTP);              // Ensure stop condition got sent

     EEPROM_AckPolling();                    // Ensure data is written in EEPROM
   }
 }

 /***********************************************************************//*
  * @brief  Current Address Read Operation. Data is read from the EEPROM.
  *      The current address from the EEPROM is used.
  * @param  none
  * @return none
  *************************************************************************/
 unsigned char EEPROM_CurrentAddressRead(void)
 {
   while(UCB0STAT & UCBUSY);                 // wait until I2C module has
                                             // finished all operations
   I2CReadInit();

   UCB0CTL1 |= UCTXSTT;                      // I2C start condition

   while(UCB0CTL1 & UCTXSTT);                // Start condition sent?
   UCB0CTL1 |= UCTXSTP;                      // I2C stop condition
   __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts
   while(UCB0CTL1 & UCTXSTP);                // Ensure stop condition got sent
   return I2CBuffer;
 }

 /***********************************************************************//*
  * @brief  Random Read Operation. Data is read from the EEPROM. The EEPROM
  *      address is defined with the parameter Address.
  * @param  Address, the address that read data from
  * @return the value of the address
  *************************************************************************/
 unsigned char EEPROM_RandomRead(unsigned int Address)
 {
   unsigned char adr_hi;
   unsigned char adr_lo;
   while (UCB0STAT & UCBUSY);                // wait until I2C module has
                                                // finished all operations
      adr_hi = Address >> 8;                    // calculate high byte
      adr_lo = Address & 0xFF;                  // and low byte of address

      I2CBufferArray[1] = adr_hi;               // store single bytes that have to
      I2CBufferArray[0] = adr_lo;               // be sent in the I2CBuffer
      PtrTransmit = 1;                          // set I2CBufferArray Pointer
      // Write Address first
       I2CWriteInit();
       UCB0CTL1 |= UCTXSTT;                      // start condition generation
                                                 // => I2C communication is started
       __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts
       __no_operation();                         //wait for ACK to start read command
       __delay_cycles(30);
       // Read Data byte
       I2CReadInit();

       UCB0CTL1 |= UCTXSTT;                      // I2C start condition
       while(UCB0CTL1 & UCTXSTT);                // Start condition sent?

       UCB0CTL1 |= UCTXSTP;                      // I2C stop condition
       __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts
       while(UCB0CTL1 & UCTXSTP);                // Ensure stop condition got sent
       return I2CBuffer;
 }

 /**********************************************************************************//*
  * @brief   Sequential Read Operation. Data is read from the EEPROM in a sequential
  *       form  the parameter address as a starting point. Specify the size to
  *       be read and populate to a Data buffer.
  * @param  Address, the start address that read data from
  * @param * Data, the pointer that pointer to read data
  * @param  Size, the size of data that will be read
  * @return none
  *********************************************************************************/
 void EEPROM_SequentialRead(unsigned int Address , unsigned char * Data , unsigned int Size)
 {
	  unsigned char adr_hi;
	  unsigned char adr_lo;
	  unsigned int counterSize;
	  while (UCB0STAT & UCBUSY);                // wait until I2C module has
	                                            // finished all operations
	  adr_hi = Address >> 8;                    // calculate high byte
	  adr_lo = Address & 0xFF;                  // and low byte of address

	  I2CBufferArray[1] = adr_hi;               // store single bytes that have to
	  I2CBufferArray[0] = adr_lo;               // be sent in the I2CBuffer.
	  PtrTransmit = 1;                          // set I2CBufferArray Pointer
	  // Write Address first
	  I2CWriteInit();
	  UCB0CTL1 |= UCTXSTT;                      // start condition generation
	                                            // => I2C communication is started
	  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts
      __no_operation();
      __delay_cycles(30);
	  // Read Data byte
	  I2CReadInit();

	  UCB0CTL1 |= UCTXSTT;                      // I2C start condition
	  while(UCB0CTL1 & UCTXSTT);                // Start condition sent?

	  for(counterSize = 0 ; counterSize < Size ; counterSize++)
	  {
	    __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
	    Data[counterSize] = I2CBuffer;
	  }
	  UCB0CTL1 |= UCTXSTP;                      // I2C stop condition
	  __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupts
	  while(UCB0CTL1 & UCTXSTP);                // Ensure stop condition got sent
 }

 /**********************************************************************************//*
  * @brief   Acknowledge Polling. The EEPROM will not acknowledge if a write cycle is
  *          in progress. It can be used to determine when a write cycle is completed.
  * @param  none
  * @return none
  *********************************************************************************/
 void EEPROM_AckPolling(void)
 {
   while (UCB0STAT & UCBUSY);                // wait until I2C module has
                                             // finished all operations
   do
   {
     UCB0IFG = 0x00;                        // clear I2C interrupt flags
     UCB0CTL1 |= UCTR;                       // I2CTRX=1 => Transmit Mode (R/W bit = 0)
     UCB0CTL1 &= ~UCTXSTT;
     UCB0CTL1 |= UCTXSTT;                    // start condition is generated
     while(UCB0CTL1 & UCTXSTT)               // wait till I2CSTT bit was cleared
     {
       if(!(UCNACKIFG & UCB0IFG))           // Break out if ACK received
         break;
     }
     UCB0CTL1 |= UCTXSTP;                    // stop condition is generated after
                                             // slave address was sent => I2C communication is started
     while (UCB0CTL1 & UCTXSTP);             // wait till stop bit is reset
     __delay_cycles(500);                    // Software delay
   }while(UCNACKIFG & UCB0IFG);


 }

 /*---------------------------------------------------------------------------*/
 /*  Interrupt Service Routines                                               */
 /*     Note that the Compiler version is checked in the following code and   */
 /*     depending of the Compiler Version the correct Interrupt Service       */
 /*     Routine definition is used.                                           */
 /*----------------------------------------------------------------------------*/

 #pragma vector = USCI_B0_VECTOR
 __interrupt void USCIAB0TX_ISR(void)
 {

	 switch(__even_in_range(UCB0IV,12))
	  {
	  case  0: break;                           // Vector  0: No interrupts
	  case  2: break;                           // Vector  2: ALIFG
	  case  4:
	  __no_operation();
	   break;                           // Vector  4: NACKIFG
	  case  6: break;                           // Vector  6: STTIFG
	  case  8: break;                           // Vector  8: STPIFG
	  case 10:                                  // Vector 10: RXIFG
	    I2CBuffer = UCB0RXBUF;                  // store received data in buffer
//	    UCB0CTL1 |= UCTXSTP;
	    __bic_SR_register_on_exit(LPM0_bits);   // Exit LPM0

	    break;
	  case 12:                                       // Vector 12: TXIFG
		     UCB0TXBUF = I2CBufferArray[PtrTransmit];// Load TX buffer
		     PtrTransmit--;                         // Decrement TX byte counter
		     if(PtrTransmit < 0)
		     {
		       while(!(UCB0IFG & UCTXIFG));
		       UCB0IE &= ~UCTXIE;                     // disable interrupts
		       UCB0IFG &= ~UCTXIFG;                   // Clear USCI_B0 TX int flag
//		       UCB0CTL1 |= UCTXSTP;
		       __bic_SR_register_on_exit(LPM0_bits); // Exit LPM0
		     }

		break;
	  default: break;
	  }


 }





