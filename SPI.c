/*
 * SPI.c
 *
 *  Created on: 13/ago/2014
 *      Author: Mihamed Hammouda
 */
#include <msp430.h>
#include "spi.h"
#include "function.h"
/*
 * CS_LoRa Chip Select for LoRa module at P1.4
 *
*/
#define CS_LoRa BIT4
char DATA;

void initSPIA0(void) {
//                   MSP430FR5739
//                 -----------------
//             /|\|                 |
//              | |                 |
//              --|RST              |
//                |                 |
//                |             P2.0|-> Data Out (UCA0SIMO)
//                |                 |
//                |             P2.1|<- Data In (UCA0SOMI)
//                |                 |
//                |             P1.5|-> Serial Clock Out (UCA0CLK)
//                |                 |
//                |             P1.4|-> CS_LoRa

	// Configure GPIO
	P1DIR |= CS_LoRa;
	P1SEL1 |= BIT5;                           // USCI_A0 operation
	P2SEL1 |= BIT0 | BIT1;                    // USCI_A0 operation

	// Configure USCI_A0 for SPI operation
	UCA0CTLW0 = UCSWRST;                      // **Put state machine in reset**
	UCA0CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB; // 3-pin, 8-bit SPI master
											// Clock polarity high, MSB
	UCA0CTLW0 |= UCSSEL__SMCLK;                // MCLK
	UCA0BR0 = 0x02;                           // /2
	UCA0BR1 = 0;                              //
	UCA0MCTLW = 0;                            // No modulation
	UCA0CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**
	UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void disableSPIA0(){
	P1SEL1 &= ~BIT5;                           // USCI_A0 operation
	P2SEL1 &= ~BIT0 & ~BIT1;                    // USCI_A0 operation
}

void spi_csh(void) {
	P1OUT |= CS_LoRa;
}

void spi_csl(void) {
	P1OUT &= ~CS_LoRa;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
	volatile unsigned int i;

//  while (!(IFG2 & UCB0TXIFG));
//  if(UCB0RXIE)
//  {
//
//	  DATA=UCB0RXBUF;
//  }

	switch (__even_in_range(UCA0IV, 0x04)) {
	case 0:
		break;                          // Vector 0 - no interrupt
	case 2:
//		DATA = UCA0RXBUF;
//		UCA0IFG &= ~UCRXIFG;
		break;
	case 4:
		break;
	default:
		break;
	}
}

