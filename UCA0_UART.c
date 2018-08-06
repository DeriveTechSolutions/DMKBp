/*
 * UCA0_UART.c
 *
 *  Created on: Aug 30, 2013
 *      Author: a0220221
 */

#include "UCA0_UART.h"
#include "ext_config.h"

void UARTInit(void)
{
    P2SEL|=BIT0+BIT1;
    PMAPPWD = 0x02D52;          // Enable Write-access to modify port mapping registers
    P2MAP0= PM_UCA0RXD;         // P2.0---UCA0RXD
    P2MAP1 =PM_UCA0TXD ;         //P2.1---UCA0TXD
    PMAPPWD = 0;           // Disable Write-Access to modify port mapping registers

    UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
    UCA0CTL1 |= UCSSEL__ACLK;                     // CLK = ACLK
    UCA0BR0 = 0x03;                           // 32kHz/9600=3.41 (see User's Guide)
    UCA0BR1 = 0x00;                           //
    UCA0MCTL = UCBRS_3+UCBRF_0;               // Modulation UCBRSx=3, UCBRFx=0
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
//  UCA0IE |= UCTXIE;                         // Enable USCI_A0 RX interrupt

}

void sendVar_init(void)
{
	CheckSum=0;
	ii=4;
	jj=0;

	ADC_array[0]=0x55;
	ADC_array[1]=0xaa;
	ADC_array[2]=0x04;
	ADC_array[3]=0x01;
	CheckSum1=0x55+0xaa+0x04+0x01;
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    break;
  case 4:

		UCA0TXBUF =ADC_array[jj];
		jj++;
		if(jj==0x07)
		{
			UCA0IE &= ~UCTXIE;
		    jj=0;
		}

	  break;                             // Vector 4 - TXIFG

  default: break;
  }

}
