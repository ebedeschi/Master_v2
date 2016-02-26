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

#define PACKET_LENGHT	40
#define BUFFER_LENGHT	40
#define MAXSLAVE	3
#define TIMEOUTSLAVE	2
#define RTCSECONDS_START	15
#define RTCSECONDS_AFTER	900

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
u8 timeout_slave = 0;
u8 ok_slave = 0;
unsigned long c_timeout=0;
unsigned long second = 0;
unsigned long rtc_seconds = 0;
unsigned long count = 0;

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

void initI2CB0()
{
	P1SEL1 |= BIT6 | BIT7;                    // I2C pins
}

void disableI2CB0()
{
	P1SEL1 &= ~BIT6 & ~BIT7;                    // I2C pins
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

void disableUART1()
{
	P2SEL1 &= ~BIT5 & ~BIT6;                    // USCI_A1 UART operation
	P2OUT = BIT5 | BIT6;
	P2DIR = BIT5 | BIT6;
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
	PJOUT |= BIT2; // Errore P1.2 - PJ.2
	P1DIR |= BIT2;
	P1OUT &= ~BIT2;
	PJOUT &= ~BIT2; // Errore P1.2 - PJ.2
	__delay_cycles(5000);
}

void radioOFF()
{
	P1OUT |= BIT2;
	PJOUT |= BIT2; // Errore P1.2 - PJ.2
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

void initGPIO()
{

	// -------- BOARD STAND ALONE - START --------
//	P1OUT = 0x00;
//	P1OUT = BIT1 + BIT2 + BIT3 + BIT6 + BIT7;
//	P1DIR = 0x00;
//	P1DIR = BIT1 + BIT2 + BIT4 + BIT5; // no BIT3
//
//	P2OUT = 0x00;
//	P2OUT = BIT2 + BIT4 + BIT5 + BIT6 + BIT7;
//	P2DIR = 0x00;
//	P2DIR = BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
//
//	P3OUT = 0xFF;
//	P3DIR = 0xFF;
//
//	P4OUT = 0x00;
//	P4OUT = BIT6 + BIT7;
//	P4DIR = 0x00;
//	P4DIR = BIT6 + BIT7;
//
//	PJOUT = 0x00;
//	PJOUT = BIT0 + BIT1 + BIT4 + BIT5 + BIT6 + BIT7;
//	PJDIR = BIT0 + BIT1 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
//
//	// Shout down mode
//	P3OUT &= ~BIT2;					// DE enable low
//	P3OUT |= BIT3;					// !RE enable hight
//	P3DIR |= BIT2 + BIT3;			// 3.2 = DE, 3.3 = RE
//
//	// LED power off
//	P3OUT &= ~BIT7;
//	P3DIR |= BIT7;
//
//	// Radio off
////	P1OUT |= BIT2;
////	P1DIR |= BIT2;
//
//	// ADCEN off
//	P2OUT |= BIT2;
//	P2DIR |= BIT2;
//
//	// ADC level low
//	P2OUT &= ~BIT3;
//	P2DIR |= BIT3;
//
////	// -------- eink --------
////	// #RST
////	P1OUT &= ~BIT1;
////	P1DIR |= BIT1;
////	// PWM, DCH
////	P2OUT |= BIT4;
////	P2OUT &= ~BIT7;
////	P2DIR &= BIT7;
////	// #DCS, BUSY
////	P3OUT |= BIT0;
////	P3OUT &= ~BIT1;
////	P3DIR |= BIT0;
////	P3DIR &= ~BIT0;
////	// #RST
////	PJOUT |= BIT0;
////	PJDIR |= BIT0;

	// -------- BOARD STAND ALONE - STOP --------

	// -------- BOARD SUPERFICE - START --------

	P1OUT = 0x00;
	P1OUT = BIT1 + BIT2 + BIT3 + BIT6 + BIT7;
	P1DIR = 0x00;
	P1DIR = BIT1 + BIT2 + BIT4 + BIT5; // no BIT3, no BIT1

	P2OUT = 0x00;
	P2OUT = BIT4 + BIT5 + BIT6;
	P2DIR = 0x00;
	P2DIR = BIT0 + BIT1 + BIT3 + BIT4 + BIT5 + BIT6;

	P3OUT = 0x00;
	P3OUT = BIT2 + BIT3 + BIT7;
	P3DIR = 0x00;
	P3DIR = BIT2 + BIT3 + BIT7;

	P4OUT = 0x00;
	P4OUT = BIT6 + BIT7;
	P4DIR = 0x00;
	P4DIR = BIT6 + BIT7;

	PJOUT = 0x00;
	PJOUT = BIT0 + BIT1 + BIT2 + BIT4 + BIT5 + BIT6 + BIT7;
	PJDIR = BIT0 + BIT1 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;


	// -------- BOARD SUPERFICE --------

	// LED power off
	P3OUT &= ~BIT7;
	P3DIR |= BIT7;

	// Radio off
//	P1OUT |= BIT2;
//	P1DIR |= BIT2;

//	// ADCEN off
//	P2OUT |= BIT2;
//	P2DIR |= BIT2;

//	// ADC level low
//	P2OUT &= ~BIT3;
//	P2DIR |= BIT3;
}

void setReceiveRS485()
{
	// Normal mode - Receive
	P3DIR |= BIT3;					// 3.2 = DE, 3.3 = RE
	P3OUT &= ~BIT3;					// DE hight-impedence, !RE enable low
}

void setShoutDownModeRS485()
{
	// Shout down mode
	P3OUT &= ~BIT2;					// DE enable low
	P3OUT |= BIT3;					// !RE enable hight
	P3DIR |= BIT2 + BIT3;			// 3.2 = DE, 3.3 = RE
}

void initLED()
{
	P3DIR |= BIT7;                            // For LED green
	P3OUT &= ~BIT7;

//	P2OUT |= BIT3;
//	P2DIR |= BIT3;

//	P4DIR |= BIT6 + BIT7;			// 4.6 = TXEnable, 4.7 = RXEnable
//	P4OUT |= BIT6;					// TXEnable hight, RXEnable low
}

void initRadioGPIO()
{
	// *** Da vedere perchè a volte i DIO danno dei problemi ***
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

void enableSlave(int slave)
{
	ok_slave = 0;
	if(slave == 1)
	{
		P3OUT &= ~BIT4;
	}
	if(slave == 2)
	{
		P3OUT &= ~BIT5;
	}
	if(slave == 3)
	{
		P3OUT &= ~BIT6;
	}
	c_timeout=0;
	timeout_slave = 0;
	// Inizializzazione del timer per timeout
	TA0CCTL0 = CCIE;                    // TACCR0 interrupt enabled
	TA0CCR0 = 62500 - 1;
	TA0CTL = TASSEL__ACLK | MC__CONTINUOUS;   // SMCLK, continuous mode
}

void disableSlave(int slave)
{
	if(slave == 1)
	{
		P3OUT |= BIT4;
	}
	if(slave == 2)
	{
		P3OUT |= BIT5;
	}
	if(slave == 3)
	{
		P3OUT |= BIT6;
	}
	TA0CTL = MC__STOP;
}

void initRTC()
{
    // Configure RTC_C
    RTCCTL01 = RTCTEVIE | RTCRDYIE | RTCBCD | RTCHOLD;
                                            // RTC enable, BCD mode, RTC hold
                                            // enable RTC read ready interrupt
                                            // enable RTC time event interrupt

    RTCYEAR = 0x2010;                       // Year = 0x2010
    RTCMON = 0x4;                           // Month = 0x04 = April
    RTCDAY = 0x05;                          // Day = 0x05 = 5th
    RTCDOW = 0x01;                          // Day of week = 0x01 = Monday
    RTCHOUR = 0x10;                         // Hour = 0x10
    RTCMIN = 0x32;                          // Minute = 0x32
    RTCSEC = 0x45;                          // Seconds = 0x45

    RTCADOWDAY = 0x80;                       	// RTC Day of week alarm
    RTCADAY = 0x80;                         	// RTC Day Alarm
    RTCAHOUR = 0x80;                        	// RTC Hour Alarm
    RTCAMIN = 0x33;                         	// RTC Minute Alarm
    RTCSEC = 0x80;								// RTC Second Alarm
}

void startRTC()
{
	RTCCTL01 &= ~(RTCHOLD);                 // Start RTC
}

void stopRTC()
{
	 RTCCTL01 |= RTCHOLD;                 // Stop RTC
}

void initVccRS485()
{
		P3OUT |= BIT4 + BIT5 + BIT6;
		P3DIR |= BIT4 + BIT5 + BIT6;
//		P3OUT &= ~BIT4;
//		P3OUT &= ~BIT5;
//		P3OUT &= ~BIT6;
		P3OUT |= BIT4;
		P3OUT |= BIT5;
		P3OUT |= BIT6;
}

void disableVccRS485()
{
	P3DIR &= ~BIT4 + ~BIT5 + ~BIT6;
	P3OUT &= ~BIT4 + ~BIT5 + ~BIT6;
}

char packet[PACKET_LENGHT+1] = {'C','i','a','o','M','o','n','d','o'};
char buffer[BUFFER_LENGHT+1];
int i=0, c=0;
unsigned long count_tx=0;
unsigned long count_tx_irq=0;
unsigned long hum=0;

int main(void) {

//	WDTCTL = WDTPW | WDTHOLD;                 // Stop WDT
//	initRadioGPIO();
//	// GPIO Setup
//	P1OUT &= ~BIT0;                           // Clear LED to start
//	P1DIR |= BIT0;                            // P1.0 output
//	P3SEL1 |= BIT0;                           // Configure P1.1 for ADC
//	P3SEL0 |= BIT0;
//
//	// Disable the GPIO power-on default high-impedance mode to activate
//	// previously configured port settings
//	PM5CTL0 &= ~LOCKLPM5;
//
//	// By default, REFMSTR=1 => REFCTL is used to configure the internal reference
//	while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
//	REFCTL0 |= REFVSEL_0 | REFON;             // Select internal ref = 1.2V
//											// Internal Reference ON
//
//	// Configure ADC12
//	ADC12CTL0 = ADC12SHT0_2 | ADC12ON;
//	ADC12CTL1 = ADC12SHP;                     // ADCCLK = MODOSC; sampling timer
//	ADC12CTL2 |= ADC12RES_2;                  // 12-bit conversion results
//	ADC12IER0 |= ADC12IE0;                    // Enable ADC conv complete interrupt
//	ADC12MCTL0 |= ADC12INCH_12 | ADC12VRSEL_12; // A1 ADC input select; Vref=1.2V
//
//	while(!(REFCTL0 & REFGENRDY));            // Wait for reference generator
//											// to settle
//
//	while(1)
//	{
//	__delay_cycles(5000000);                    // Delay between conversions
//	ADC12CTL0 |= ADC12ENC | ADC12SC;         // Sampling and conversion start
//
//	__bis_SR_register(LPM0_bits + GIE);      // LPM0, ADC10_ISR will force exit
//	__no_operation();                        // For debug only
//
//	initSPIA0();
//	radioON();
//	radioInit();
//
//	packet[0]='\0';
//	sprintf(packet, "H1:%lu:%d:%d", hum, (Pa_Config_Struct.PaSelect==PaSelect_PA_Boost_Pin)?1:0, Pa_Config_Struct.OutputPower);
////		sprintf(packet, "T5:%.2f:%d:%d", g_temp, (Pa_Config_Struct.PaSelect==PaSelect_PA_Boost_Pin)?1:0, Pa_Config_Struct.OutputPower);
//	//		sprintf(packet, "T3:%.2f", g_temp);
//	PayLoadLenghtSet_Value=strlen(packet);
//	Packet_Set(PreambleLenghtSet_Value,PayLoadLenghtSet_Value);
//	P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR
//	Packet_Tx(PayLoadLenghtSet_Value, packet);
//	count_tx++;
//
//	__bis_SR_register(LPM0_bits + GIE);
//	P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR
//
//	radioOFF();
//	disableSPIA0();
//
//	}


	WDTCTL = WDTPW | WDTHOLD;       // Stop WDT

	rtc_seconds = RTCSECONDS_START;
//	__delay_cycles(1000000);

	initVccRS485();
	initGPIO();
	initRadioGPIO();
	setShoutDownModeRS485();


//	//------ MCLK -------
//	// Disable the GPIO power-on default high-impedance mode to activate
//	// previously configured port settings
//	PM5CTL0 &= ~LOCKLPM5;
//	// Clock setup
//	CSCTL0_H = CSKEY >> 8;                    // Unlock CS registers
//	CSCTL1 = DCOFSEL_0 | DCORSEL;             // Set DCO to 1MHz
//	CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK; 	// Set MCLK = DCO
//	CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;     // Set all dividers to 1
//	CSCTL4 &= ~LFXTOFF;						// Turn on LFXT
//	// Lock CS registers - Why isn't PUC created?
//	CSCTL0_H = 0;

	//------ ACLK -------
	 PJSEL0 = BIT4 | BIT5;                   // Initialize LFXT pins
	// Disable the GPIO power-on default high-impedance mode to activate
	// previously configured port settings
	PM5CTL0 &= ~LOCKLPM5;

	// Configure LFXT 32kHz crystal
	CSCTL0_H = CSKEY >> 8;                  // Unlock CS registers
	CSCTL4 &= ~LFXTOFF;                     // Enable LFXT
	do
	{
	  CSCTL5 &= ~LFXTOFFG;                  // Clear LFXT fault flag
	  SFRIFG1 &= ~OFIFG;
	} while (SFRIFG1 & OFIFG);              // Test oscillator fault flag
	CSCTL0_H = 0;                           // Lock CS registers

//	initSPIA0();
//	initUARTA0();
//	initUARTA1();


//	// Enable interrupts
//	__bis_SR_register(GIE);

	initRTC();
	stopRTC();
	second = rtc_seconds - 10;

//	setReceiveRS485();
//
//	disableVccRS485();
//	setShoutDownModeRS485();
//	disableUART1();
//	initSPIA0();
////	radioON();
////	radioInit();
////	radioOFF();
//	disableSPIA0();
//
//	startRTC();
//	__bis_SR_register(LPM3_bits + GIE);

    for(;;) {
		// ***** STAND-ALONE - START *****

    	stopRTC();

		if(count>120)
				rtc_seconds = RTCSECONDS_AFTER;

    	initI2CB0();
		// Read Temperature
//		SHT21ReadTemperature();
		g_temp = g_temp;
		disableI2CB0();

		g_temp = 6.78;

		// random start delay for next
		int start = ((float)g_temp - (int)g_temp) * 1000;
		start &= 0x07;
		if(start<0 || start >9)
			second = 0;
		else
			second+= start;

		initSPIA0();
    	radioON();
    	radioInit();

		packet[0]='\0';
		sprintf(packet, "T7:%.2f:%d:%d", g_temp, (Pa_Config_Struct.PaSelect==PaSelect_PA_Boost_Pin)?1:0, Pa_Config_Struct.OutputPower);
		//		sprintf(packet, "T3:%.2f", g_temp);
		PayLoadLenghtSet_Value=strlen(packet);
		Packet_Set(PreambleLenghtSet_Value,PayLoadLenghtSet_Value);
		unsigned short pr_l = 0, pa_l = 0;
		Packet_Get(&pr_l, &pr_l);
		P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR
		Packet_Tx(PayLoadLenghtSet_Value, packet);
    	count_tx++;

    	__bis_SR_register(LPM0_bits + GIE);
    	P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR

    	radioOFF();
    	disableSPIA0();

    	startRTC();
    	__bis_SR_register(LPM3_bits + GIE);
    	// ***** STAND-ALONE - FINE *****



		// ***** BOARD SUPERFICE - START *****
//    	stopRTC();
//
//    	if(count>120)
//    			rtc_seconds = RTCSECONDS_AFTER;
//
//    	int s = 1;
//    	for(s=1;s<=3;s++)
//		{
//
//			memset(buffer, 0, BUFFER_LENGHT);
//			memset(packet, 0, PACKET_LENGHT);
//			initUARTA1();
//			setReceiveRS485();
//			initVccRS485();
////			__bis_SR_register(GIE);
//			enableSlave(s);
//			__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
//			disableSlave(s);
//			disableVccRS485();
//			setShoutDownModeRS485();
//			disableUART1();
//
//
//			// No timeout. Ricevuto dato da Slave.
//			if(timeout_slave == 0 && ok_slave == 1)
//			{
//
//				initSPIA0();
//				radioON();
//				radioInit();
//
//				packet[0]='\0';
//				sprintf(packet, "%s:%d:%d", buffer, (Pa_Config_Struct.PaSelect==PaSelect_PA_Boost_Pin)?1:0, Pa_Config_Struct.OutputPower);
////				sprintf(packet, "%s", buffer);
//				PayLoadLenghtSet_Value=strlen(packet);
//				Packet_Set(PreambleLenghtSet_Value,PayLoadLenghtSet_Value);
////				P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR
//				Packet_Tx(PayLoadLenghtSet_Value, packet);
//				count_tx++;
//
//				__bis_SR_register(LPM0_bits + GIE);
////				P3OUT ^= BIT7;                      // Toggle P3.7 using exclusive-OR
//
//				radioOFF();
//				disableSPIA0();
//
//			}
//
//		}
//
////    	__delay_cycles(1000000);
////    	__delay_cycles(1000000);
//
//
////    	disableVccRS485();
////    	setShoutDownModeRS485();
////    	disableUART1();
//
//		startRTC();
//    	__bis_SR_register(LPM3_bits + GIE);

//		 ***** BOARD SUPERFICE - FINE *****



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
	if (++c_timeout >= TIMEOUTSLAVE) {
		c_timeout=0;
		timeout_slave = 1;
		if(ok_slave == 0)
			__bic_SR_register_on_exit(LPM0_bits); 	// Exit LPM0

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
		cu=0;
    buffer[cu++] = ch;
    UCA1IFG &= ~UCRXIFG;
    if (ch == '\n') // '\n' received?
    {
    	buffer[cu-1] = '\0';
    	cu=0;
    	ok_slave = 1;
	    __bic_SR_register_on_exit(LPM0_bits);
    }

//    P3OUT ^= BIT7;
}


#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void)
{
    switch(__even_in_range(RTCIV, RTCIV_RT1PSIFG))
    {
        case RTCIV_NONE:      break;        // No interrupts
        case RTCIV_RTCOFIFG:  break;        // RTCOFIFG
        case RTCIV_RTCRDYIFG:               // RTCRDYIFG
        	if(++second>=rtc_seconds)
        	{
            	count++;
//                P3OUT ^= BIT7;                  // Toggles P1.0 every second
        		second = 0;
        		__bic_SR_register_on_exit(LPM3_bits); 	// Exit LPM3
        	}
            break;
        case RTCIV_RTCTEVIFG:               // RTCEVIFG
            __no_operation();               // Interrupts every minute
            break;
        case RTCIV_RTCAIFG:	  break;        // RTCAIFG
        case RTCIV_RT0PSIFG:  break;        // RT0PSIFG
        case RTCIV_RT1PSIFG:  break;        // RT1PSIFG
        default: break;
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch (__even_in_range(ADC12IV, ADC12IV_ADC12RDYIFG))
  {
    case ADC12IV_NONE:        break;        // Vector  0:  No interrupt
    case ADC12IV_ADC12OVIFG:  break;        // Vector  2:  ADC12MEMx Overflow
    case ADC12IV_ADC12TOVIFG: break;        // Vector  4:  Conversion time overflow
    case ADC12IV_ADC12HIIFG:  break;        // Vector  6:  ADC12BHI
    case ADC12IV_ADC12LOIFG:  break;        // Vector  8:  ADC12BLO
    case ADC12IV_ADC12INIFG:  break;        // Vector 10:  ADC12BIN
    case ADC12IV_ADC12IFG0:                 // Vector 12:  ADC12MEM0 Interrupt
    	hum=ADC12MEM0;
      __bic_SR_register_on_exit(LPM0_bits); // Exit active CPU
      break;                                // Clear CPUOFF bit from 0(SR)

    case ADC12IV_ADC12IFG1:   break;        // Vector 14:  ADC12MEM1
    case ADC12IV_ADC12IFG2:   break;        // Vector 16:  ADC12MEM2
    case ADC12IV_ADC12IFG3:   break;        // Vector 18:  ADC12MEM3
    case ADC12IV_ADC12IFG4:   break;        // Vector 20:  ADC12MEM4
    case ADC12IV_ADC12IFG5:   break;        // Vector 22:  ADC12MEM5
    case ADC12IV_ADC12IFG6:   break;        // Vector 24:  ADC12MEM6
    case ADC12IV_ADC12IFG7:   break;        // Vector 26:  ADC12MEM7
    case ADC12IV_ADC12IFG8:   break;        // Vector 28:  ADC12MEM8
    case ADC12IV_ADC12IFG9:   break;        // Vector 30:  ADC12MEM9
    case ADC12IV_ADC12IFG10:  break;        // Vector 32:  ADC12MEM10
    case ADC12IV_ADC12IFG11:  break;        // Vector 34:  ADC12MEM11
    case ADC12IV_ADC12IFG12:  break;
    case ADC12IV_ADC12IFG13:  break;        // Vector 38:  ADC12MEM13
    case ADC12IV_ADC12IFG14:  break;        // Vector 40:  ADC12MEM14
    case ADC12IV_ADC12IFG15:  break;        // Vector 42:  ADC12MEM15
    case ADC12IV_ADC12IFG16:  break;        // Vector 44:  ADC12MEM16
    case ADC12IV_ADC12IFG17:  break;        // Vector 46:  ADC12MEM17
    case ADC12IV_ADC12IFG18:  break;        // Vector 48:  ADC12MEM18
    case ADC12IV_ADC12IFG19:  break;        // Vector 50:  ADC12MEM19
    case ADC12IV_ADC12IFG20:  break;        // Vector 52:  ADC12MEM20
    case ADC12IV_ADC12IFG21:  break;        // Vector 54:  ADC12MEM21
    case ADC12IV_ADC12IFG22:  break;        // Vector 56:  ADC12MEM22
    case ADC12IV_ADC12IFG23:  break;        // Vector 58:  ADC12MEM23
    case ADC12IV_ADC12IFG24:  break;        // Vector 60:  ADC12MEM24
    case ADC12IV_ADC12IFG25:  break;        // Vector 62:  ADC12MEM25
    case ADC12IV_ADC12IFG26:  break;        // Vector 64:  ADC12MEM26
    case ADC12IV_ADC12IFG27:  break;        // Vector 66:  ADC12MEM27
    case ADC12IV_ADC12IFG28:  break;        // Vector 68:  ADC12MEM28
    case ADC12IV_ADC12IFG29:  break;        // Vector 70:  ADC12MEM29
    case ADC12IV_ADC12IFG30:  break;        // Vector 72:  ADC12MEM30
    case ADC12IV_ADC12IFG31:  break;        // Vector 74:  ADC12MEM31
    case ADC12IV_ADC12RDYIFG: break;        // Vector 76:  ADC12RDY
    default: break;
  }
}
