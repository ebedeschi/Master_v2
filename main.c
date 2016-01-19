//***************************************************************************************
//                   MSP430FR5969
//                 -----------------
//             /|\|                 |
//              | |                 |
//              --|RST              |
//                |                 |
//                |             P2.0|-> SPI MOSI Data Out (UCA0SIMO)
//                |                 |
//                |             P2.1|<- SPI MISO Data In (UCA0SOMI)
//                |                 |
//                |             P1.5|-> Serial Clock Out (UCA0CLK)
//                |                 |
//                |             P1.4|-> CS_LoRa
//                |                 |
//                |             P1.3|-> NRESET (next )
//                |                 |
//                |             P4.6|-> TXEnable (next )
//                |                 |
//                |             P4.7|-> RXEnable (next )
//                |                 |
//                |             P3.2|-> DE
//                |                 |
//                |             P3.3|-> RE
//                |                 |
//                |             P3.7|-> LED green
//                |                 |
//                |             P4.0|-> DIO0
//                |                 |
//                |             P4.1|-> DIO1
//                |                 |
//                |             P4.2|-> DIO2
//                |                 |
//                |             P1.2|-> VDD_LoRa (next )
//                |                 |
//                |             P3.4|-> PSQ1 (next )
//                |                 |
//                |             P3.5|-> PSQ2 (next )
//                |                 |
//                |             P3.6|-> PSQ3 (next )
//                |                 |
//***************************************************************************************

#include <msp430.h>
#include <STDIO.H>
#include "SPI.h"
#include "function.h"
#include "sx1276Regs-LoRa.h"
#include "shtLib.h"

#define PACKET_LENGHT	16
#define BUFFER_LENGHT	40
#define MAXSLAVE	3
#define TIMEOUTSLAVE	150

	Modem_Config1 Modem_Config1_Struct;
	Modem_Config2 Modem_Config2_Struct;
	Modem_Config3 Modem_Config3_Struct;
	OpMode OpMode_Struct;
	PaConfig Pa_Config_Struct;
	OCP_Config OCP_Config_Struct;
	LNA_Config LNA_Config_Struct;
	volatile u8 PayLoadLenghtSet_Value=PACKET_LENGHT,PreambleLenghtSet_Value;
int Packet_Snr,Packet_Rssi,Rssi_Value,Packet_Count,Modem_Status;

unsigned char ch;
u8 slave = 1;
u8 timeout_slave = 0;
u8 ok_slave = 0;
u8 s = 1;

/**
 * Initialize UCA0 module in UART mode with boud rate 9600
 */
void initUARTA0()
{
	// Configure GPIO
	P2SEL1 |= BIT0 | BIT1;                    // USCI_A0 UART operation
	P2SEL0 &= ~(BIT0 | BIT1);
	// Configure USCI_A0 for UART mode
	UCA0CTLW0 = UCSWRST;                      // Put eUSCI in reset
	UCA0CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
	// Baud Rate calculation
	// 1000000/(16*9600) = 6.5104167
	UCA0BR0 = 6;                             // 1000000/16/9600
	UCA0BR1 = 0x00;
	UCA0MCTLW |= UCBRS0 + UCBRF_8 + UCOS16;
	UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
	UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void sendByteUARTA0(unsigned char byte)
{
	while(!(UCA0IFG&UCTXIFG));
	UCA0TXBUF = byte;
}

/**
 * Initialize UCA1 module in UART mode with boud rate 9600
 */
void initUARTA1()
{
	// Configure GPIO
	P2SEL1 |= BIT5 | BIT6;                    // USCI_A1 UART operation
	P2SEL0 &= ~(BIT5 | BIT6);
	// Configure USCI_A0 for UART mode
	UCA1CTLW0 = UCSWRST;                      // Put eUSCI in reset
	UCA1CTLW0 |= UCSSEL__SMCLK;               // CLK = SMCLK
	// Baud Rate calculation
	// 1000000/(16*9600) = 6.5104167
	UCA1BR0 = 6;                             // 1000000/16/9600
	UCA1BR1 = 0x00;
	UCA1MCTLW |= UCBRS0 + UCBRF_8 + UCOS16;
	UCA1CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
	UCA1IE |= UCRXIE;                         // Enable USCI_A1 RX interrupt
}

void sendByteUARTA1(unsigned char byte)
{
	while(!(UCA1IFG&UCTXIFG));
	UCA1TXBUF = byte;
}

void radioInit()
{

	OpMode read_OpMode_Struct;
	OPMode_Get(&read_OpMode_Struct);
	/*
	 * Definizione Struttura OpMode
	 * LongRangeMode_TypeDef LongRangeMode: Definisce se si vuole usare la Modalità LoRa oppure FSK
	   Mode_TypeDef Mode: Definisce in che modalità deve operare il modem. Attenzione: I registri non possono essere ne scritti ne letti nelle modalità trasmissione e ricezione
	   Low_FrequencyModeOn_TypeDef Low_FrequencyModeOn: Definisce se si vuole operare in LF o HF
	 *
	 */
	OpMode_Struct.LongRangeMode=Mode_Lora;
	OpMode_Struct.Low_FrequencyModeOn=Mode_HF;
	OpMode_Struct.Mode = Mode_Sleep;
	// Inizializza la strutta
	OPMode_Init(&OpMode_Struct);

	OPMode_Get(&read_OpMode_Struct);


//	u8 lora = Recieve_RegisterData(0x01);

	/*
	 * Inizializzazione paramenteri della struttura Modem_Config1
	 * BandWidth_TypeDef BandWidth: Imposta la Bandwith che si vuole usare
	   CodingRate_TypeDef CodingRate: Imposta il CR per la codifica degli errori nei pacchetti
	   ImplicitHeaderModeOn_TypeDef ImplicitHeaderModeOn: Abilita o disablita l'header nel pacchetto
	 * Nota: Questi paramentri devono avere gli stessi valori sia nell'antenna ricevente che in quella trasmittente
	 * Le BandWidth 250 e 500 sono incompatibili con la frequenza 169MHz
	 */
	Modem_Config1_Struct.BandWidth=Bandwidth_500KHz;
	Modem_Config1_Struct.CodingRate=CodingRate_4_6;
	Modem_Config1_Struct.ImplicitHeaderModeOn=ExplicitHeaderMode;//ImplicitHeaderMode;
	//Inizializza la strutta Modem_Config1
	Modem_Config1_Init(&Modem_Config1_Struct);

	Modem_Config1 read_Modem_Config1_Struct;
	Modem_Config1_Get(&read_Modem_Config1_Struct);

	/*
	 * Inizializzazione paramentri struttura Modem_Config2
	 * SpreadingFactor_TypeDef SpreadingFactor: Imposta il valore del SF che si vuole usare per la trasmissione dei bit
	   Tx_Continuos_Mode_TypeDef Tx_Continuos_Mode: Imposta la modalità di trasmissione, Singola o continua
	   RxPayLoadCrcOn_TypeDef RxPayLoadCrcOn: Imposta il controllo del pacchetto ricevuto
	 * Nota: Il valore di SpreadingFactor deve essere il medesimo sia nell'antenna ricevente che in quella trasmittente
	 */
	Modem_Config2_Struct.SpreadingFactor=SpreadingFactor_12;
	Modem_Config2_Struct.RxPayLoadCrcOn=Header_Indicates_CRC_ON;
	Modem_Config2_Struct.Tx_Continuos_Mode=Tx_ContinuosMode_NormalMode;
	Modem_Config2_Init(&Modem_Config2_Struct);

	Modem_Config2 read_Modem_Config2_Struct;
	Modem_Config2_Get(&read_Modem_Config2_Struct);

	Modem_Config3_Struct.AGC=AGCAUTO_ON;
	Modem_Config3_Struct.LowDataRateOptimize=LowDataRateOptimize_Enable;
	Modem_Config3_Init(&Modem_Config3_Struct);

	Modem_Config3 read_Modem_Config3_Struct;
	Modem_Config3_Get(&read_Modem_Config3_Struct);

	OCP_Config_Struct.OCPTriming=27;
	OCP_Config_Struct.OCP_ON=OCP_DISBALE;
	OCP_Config_Init(&OCP_Config_Struct);

	OCP_Config read_OCP_Config_Struct;
	OCP_Config_Get(&read_OCP_Config_Struct);

	/*
	 *  Inizializzazione paramentri struttura Pa_Config_Struct
	 * MaxPower: Definisce la potenza massima che può essere emessa dall'antenna
	 * Pmax=10.8+0.6*MaxPower [dBm]
	 * OutputPower:
	 * Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)
	 * Pout=17-(15-OutputPower) if PaSelect = 1 (PA_BOOST pin)
	 * PaSelect: Definisce se usare il Pin Boost(Permette una potenza in uscita di 20dBm) oppure l'RFO normale per la trasmissione in LF
	 * Nota: Il pin RFO non è mappato, in caso di trasmissione LF bisogna usare necessariamente il Pin Boost
	 */
	Pa_Config_Struct.MaxPower=7;
	Pa_Config_Struct.OutputPower=0;
	Pa_Config_Struct.PaSelect=PaSelect_PA_Boost_Pin;
//	Pa_Config_Struct.MaxPower=7;
//	Pa_Config_Struct.OutputPower=0;
//	Pa_Config_Struct.PaSelect=PaSelect_RFO_Pin;
	Pa_Config_Init(&Pa_Config_Struct);

//	Send_Data(RegPaDac, 0x87);

	PaConfig read_Pa_Config_Struct;
	Pa_Config_Get(&read_Pa_Config_Struct);

	/*
	 * Inizializzazione paramentri struttura LNA_Config_Struct
	 * LNA_BoostHf: High Frequency LNA current adjustment
	 * LNA_BoostLF: Low Frequency LNA current adjustment
	 * LNA_GAIN:
	 * LNA_Config_Struct non va impostato nel caso di LF a 169MHz
	 */
	LNA_Config_Struct.LNA_BoostHf=LNABOOSTHF_Default;
	LNA_Config_Struct.LNA_BoostLF=LNA_BOOSTLF_Default;
	LNA_Config_Struct.LNA_GAIN=LNAGAIN_G1;
	LNAConfig_Init(&LNA_Config_Struct);

	LNA_Config read_LNA_Config_Struct;
	LNAConfig_Get(&read_LNA_Config_Struct);

	/*
	 * Impostazione del Rise/Fall time della rampa trasmessa
	 */
	PaRamp_Set(Paramp_50us);

	u8 d1=0;
	PaRamp_Get(&d1);

	// Impostazione della frequenza: Il modem opera solo a 169MHz oppure 870 MHz
	Frequency_Set(868750000);

	unsigned long long d2=0;
	Frequency_Get(&d2);

	/*
	 * Impostazione formato pacchetto
	 * PayLoadLenghtSet_Value: Imposta la dimensione in byte del pacchetto da trasmettere o ricevere
	   PreambleLenghtSet_Value: Imposta la dimensione del Pramble
	 * Nota1: Questi due parametri devono coincidere sia nella antenna ricevente che in quella trasmittente altrimenti i pacchetti trasmessi vengono ignorati
	 * Nota2. Per la trasmissione di pacchetti di una dimensione non prefissata o di dimensioni maggiori della FIFO, il PayLoadLenght_SetValue e il PreambleLenghtSet_Value
	 * vanno settati a zero. In questa modalità la fifo opera in continuo,
	 */
//	PayLoadLenghtSet_Value=PACKET_LENGHT;
	PreambleLenghtSet_Value=12;
	// Inizializzazione del Payload e del Pramble
	Packet_Set(PreambleLenghtSet_Value,PayLoadLenghtSet_Value);

	u8 d3=0, d4=0;
	Packet_Get(&d3,&d4);

	// Inizializzazione della fase per il phas-locked-
	PLL_Set(PLL_300MHz);

	u8 d5=0;
	PLL_Get(&d5);

//	Rx_TimeOut(0x1F);
//	OpMode_Struct.Mode = Mode_RxContinuous;
//	OPMode_Init(&OpMode_Struct);


	DIO_Configuratio_01_TXDONE();
}

void radioON()
{
	P1OUT |= BIT2;
	P1DIR |= BIT2;
	P1OUT &= ~BIT2;
	__delay_cycles(5000);
}

void radioOFF()
{
	P1OUT |= BIT2;
	P1DIR |= BIT2;
	__delay_cycles(5000);
}

void radioReset()
{
	P1DIR |= BIT3;
	P1OUT &= ~BIT3;
	__delay_cycles(10000);
	P1DIR &= ~BIT3;
	__delay_cycles(5000);
}

void gpioLowPower()
{
	P1OUT = 0x00;
	P1OUT = BIT1 + BIT2 + BIT6 + BIT7;
	P1DIR = 0x00;
	P1DIR = BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;

	P2OUT = 0x00;
	P2OUT = BIT2 + BIT4 + BIT5 + BIT6 + BIT7;
	P2DIR = 0x00;
	P2DIR = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;

	P3OUT = 0xFF;
	P3DIR = 0xFF;

	P4OUT = 0x00;
	P4OUT = BIT6 + BIT7;
	P4DIR = 0x00;
	P4DIR = BIT6 + BIT7;

	PJOUT = 0x00;
	PJOUT = BIT0 + BIT1 + BIT2 + BIT4 + BIT5 + BIT6 + BIT7;
	PJDIR = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;

	// -------- BOARD SUPERFICE --------
	// Shout down mode
	P3OUT &= ~BIT2;					// DE enable low
	P3OUT |= BIT3;					// !RE enable hight
	P3DIR |= BIT2 + BIT3;			// 3.2 = DE, 3.3 = RE

	// LED power off
	P3OUT &= ~BIT7;
	P3DIR |= BIT7;

	// Radio off
//	P1OUT |= BIT2;
//	P1DIR |= BIT2;

	// ADCEN off
	P2OUT |= BIT2;
	P2DIR |= BIT2;

	// ADC level low
	P2OUT &= ~BIT3;
	P2DIR |= BIT3;

	// -------- eink --------
	// #RST
	P1OUT &= ~BIT1;
	P1DIR |= BIT1;
	// PWM, DCH
	P2OUT |= BIT4;
	P2OUT &= ~BIT7;
	P2DIR &= BIT7;
	// #DCS, BUSY
	P3OUT |= BIT0;
	P3OUT &= ~BIT1;
	P3DIR |= BIT0;
	P3DIR &= ~BIT0;
	// #RST
	PJOUT |= BIT0;
	PJDIR |= BIT0;
}

void gpioInit()
{
	// Configure GPIO
	P1OUT &= ~BIT0;                           // Clear P1.0 output latch
	P3DIR |= BIT7;                            // For LED green
	P3OUT &= ~BIT7;
	P1SEL1 |= BIT6 | BIT7;                    // I2C pins

//	P2OUT |= BIT3;
//	P2DIR |= BIT3;

//	// Normal mode - Receive
//	P3DIR |= BIT3;					// 3.2 = DE, 3.3 = RE
//	P3OUT &= ~BIT3;					// DE hight-impedence, !RE enable low

	// Shout down mode
	P3OUT &= ~BIT2;					// DE enable low
	P3OUT |= BIT3;					// !RE enable hight
	P3DIR |= BIT2 + BIT3;			// 3.2 = DE, 3.3 = RE

//	P4DIR |= BIT6 + BIT7;			// 4.6 = TXEnable, 4.7 = RXEnable
//	P4OUT |= BIT6;					// TXEnable hight, RXEnable low

	// P4.0|-> DIO0
	P4DIR &= ~BIT0;                // input mode (P4.0), 0
	P4OUT &= ~BIT0;                // select pull-down mode, 0
	P4IE |= BIT0;                  // P4.0 interrupt enabled
	P4IES &= ~BIT0;                // P4.0 Hi/lo edge
	P4IFG &= ~BIT0;                // P4.0 IFG cleared
	// P4.1|-> DIO1
	P4DIR &= ~BIT1;                // input mode (P4.1), 0
	P4OUT &= ~BIT1;                // select pull-down mode, 0
	P4IE |= BIT1;                  // P4.1 interrupt enabled
	P4IES &= ~BIT1;                // P4.1 Hi/lo edge
	P4IFG &= ~BIT1;                // P4.1 IFG cleared
	// P4.2|-> DIO2
	P4DIR &= ~BIT2;                // input mode (P4.2), 0
	P4OUT &= ~BIT2;                // select pull-down mode, 0
	P4IE |= BIT2;                  // P4.2 interrupt enabled
	P4IES &= ~BIT2;                // P4.2 Hi/lo edge
	P4IFG &= ~BIT2;                // P4.2 IFG cleared
}

void enableSlave(u8 slave)
{
	timeout_slave = 0;
	ok_slave = 0;
	if(slave == 1)
	{
		P3OUT &= ~BIT4;
	}
	else if(slave == 2)
	{
		P3OUT &= ~BIT5;
	}
	else if(slave == 3)
	{
		P3OUT &= ~BIT6;
	}
	// Inizializzazione del timer per timeout
//	TA0CCTL0 = CCIE;                    // TACCR0 interrupt enabled
//	TA0CCR0 = 62500 - 1;
//	TA0CTL = TASSEL__SMCLK | MC__CONTINOUS;   // SMCLK, continuous mode
}

void disableSlave(u8 slave)
{
	if(slave == 1)
	{
		P3OUT |= BIT4;
	}
	else if(slave == 2)
	{
		P3OUT |= BIT5;
	}
	else if(slave == 3)
	{
		P3OUT |= BIT6;
	}
}

char packet[PACKET_LENGHT+1] = {'C','i','a','o','M','o','n','d','o'};
char buffer[BUFFER_LENGHT+1];
int i=0, c=0;
unsigned long count_tx=0;
unsigned long count_tx_irq=0;
unsigned long c_timeout=0;

int main(void) {
	WDTCTL = WDTPW | WDTHOLD;       // Stop WDT

	__delay_cycles(1000000);

//	gpioLowPower();
	gpioInit();

	// Disable the GPIO power-on default high-impedance mode to activate
	// previously configured port settings
	PM5CTL0 &= ~LOCKLPM5;

	// Clock setup
	CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
	CSCTL1 = DCOFSEL_0 | DCORSEL;             // Set DCO to 1MHz
	CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; 	// Set MCLK = DCO
	CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
	CSCTL4 &= ~LFXTOFF;						// Turn on LFXT

	// Lock CS registers - Why isn't PUC created?
	CSCTL0_H = 0;

	initSPIA0();
	//	initUARTA0();
//	initUARTA1();

//	P3OUT |= BIT4 + BIT5 + BIT6;
//	P3DIR |= BIT4 + BIT5 + BIT6;
//	P3OUT |= BIT4;
//	P3OUT |= BIT5;
//	P3OUT |= BIT6;
//	P3OUT &= ~BIT4;
//	P3OUT &= ~BIT5;
//	P3OUT &= ~BIT6;


	// Enable interrupts
	__bis_SR_register(GIE);


    for(;;) {

		// ***** STAND-ALONE - START *****

    	P1SEL1 |= BIT6 | BIT7;                    // I2C pins
		// Read Temperature
		SHT21ReadTemperature();
		g_temp = g_temp;

//    	gpioInit();
//    	initSPIA0();
    	radioON();
    	radioInit();

		packet[0]='\0';
		sprintf(packet, "T:%.2f:%d:%d", g_temp, (Pa_Config_Struct.PaSelect==PaSelect_PA_Boost_Pin)?1:0, Pa_Config_Struct.OutputPower);
		PayLoadLenghtSet_Value=strlen(packet);
		Packet_Set(PreambleLenghtSet_Value,PayLoadLenghtSet_Value);
		P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR
		Packet_Tx(PayLoadLenghtSet_Value, packet);
    	count_tx++;

    	__bis_SR_register(LPM0_bits + GIE);
    	P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR

    	radioOFF();
    	gpioLowPower();

    	__bis_SR_register(LPM3_bits);

		// ***** STAND-ALONE - FINE *****

		// ***** BOARD SUPERFICE - START *****
//		for(s=1; s<=2; s++)
//		{
//
//			memset(buffer, 0, BUFFER_LENGHT);
//			memset(packet, 0, PACKET_LENGHT);
//			__bis_SR_register(GIE);
//			enableSlave(s);
//			__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
//			disableSlave(s);
//
//			// No timeout. Ricevuto dato da Slave.
//			if(timeout_slave == 0 && ok_slave == 1)
//			{
//				radioON();
//				radioInit();
//
//				packet[0]='\0';
//				sprintf(packet, "%s:%d:%d", buffer, (Pa_Config_Struct.PaSelect==PaSelect_PA_Boost_Pin)?1:0, Pa_Config_Struct.OutputPower);
//				PayLoadLenghtSet_Value=strlen(packet);
//				Packet_Set(PreambleLenghtSet_Value,PayLoadLenghtSet_Value);
//
//		//		for(i=strlen(packet);i<PayLoadLenghtSet_Value;i++)
//		//			packet[i]=':';
//		//		packet[PayLoadLenghtSet_Value]='\0';
//				 P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR
//				Packet_Tx(PayLoadLenghtSet_Value, packet);
//				count_tx++;
//
//				__bis_SR_register(LPM0_bits + GIE);
//				 P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR
//
//				radioOFF();
//
//
////				for(i=0;i<strlen(buffer);i++)
////				{
////					sendByteUARTA0(buffer[i]);
////				}
////				sendByteUARTA0('\n');
//			}
//			else
//			{
////				sprintf(buffer, "Timeout\n");
////				for(i=0;i<strlen(buffer);i++)
////				{
////					sendByteUARTA0(buffer[i]);
////				}
////				sendByteUARTA0('\n');
//			}
//		}
////    	// Inizializzazione del timer
////    	TA0CCTL0 = CCIE;                          // TACCR0 interrupt enabled
////    	TA0CCR0 = 65535 - 1;
////    	TA0CTL = TASSEL_2 + MC_2;                 // SMCLK, continuous mode
//
////    	__bis_SR_register(LPM0_bits | GIE);

		// ***** BOARD SUPERFICE - FINE *****

    	__delay_cycles(2000000);
    	__delay_cycles(2000000);

    }

    return 0;
}

//  Port    4   interrupt   service routine
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
{
	volatile int counter = 0;
	volatile u8 val = 0;
	volatile u8 size = 0;
	switch(__even_in_range(P4IV,16))
	{
	case   0:  break;  //  No  Interrupt
	case   2: 		   //  P4.0
		if((P4IN & BIT0) == BIT0)
		{
			Send_Data(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
			Send_Data(RegIRQFlags, 0x00);

//			 P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR
		}
		P4IFG &= ~BIT0;                       // P4.0 IFG cleared
		break;
	case   4: 		   //  P4.1
		if((P4IN & BIT1) == BIT1)
		{
			// TODO
		}
		P4IFG &= ~BIT1;                       // P4.1 IFG cleared
		break;
	case   6: 		   //  P4.2
		if((P4IN & BIT2) == BIT2)
		{
			// TODO
		}
		P4IFG &= ~BIT2;                       // P4.1 IFG cleared
		break;
	case   8:  break;  //  P4.3
	case   10: break;  //  P4.4
	case   12: break;  //  P4.5
	case   14: break;  //  P4.6
	case   16: break;  //  P4.7
	}
	count_tx_irq++;
	__bic_SR_register_on_exit(LPM0_bits);
}

// Timer A0 interrupt service routine
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void)
{
	 if (++c_timeout == TIMEOUTSLAVE) {
		 	c_timeout=0;
		 	timeout_slave = 1;
		 	if(ok_slave == 0)
		 		__bic_SR_register_on_exit(LPM0_bits); 	// Exit LPM3

	}
}

// Timer B1 interrupt service routine
#pragma vector = TIMER0_B0_VECTOR
__interrupt void Timer0_B0_ISR(void)
{
	TB0CTL &= ~MC_3;
	__bic_SR_register_on_exit(LPM0_bits); 	// Exit LPM0
}

int cu=0;
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
//	P3OUT ^= BIT7;
	ch = UCA1RXBUF;
	if(ch == 'T')
		c=0;
    buffer[c++] = ch;
    UCA1IFG &= ~UCRXIFG;
    if (ch == '\n') // '\n' received?
    {
    	buffer[c-1] = '\0';
    	c=0;
    	ok_slave = 1;
	    __bic_SR_register_on_exit(LPM0_bits);
    }

//    P3OUT ^= BIT7;
}
