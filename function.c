#include <msp430.h>
#include "function.h"
#include "spi.h"
#include "sx1276Regs-LoRa.h"
#include "sx1276Regs-Fsk.h"
extern OpMode OpMode_Struct;
extern int PayLoadLenghtSet_Value;

void Send_Data(volatile uint8_t RegisterAddr, volatile uint8_t Temp_Data) {
	spi_csl();
	__delay_cycles(20);
	UCA0TXBUF = mhr + RegisterAddr;
	while (UCA0STATW & UCBUSY)
		;
	UCA0TXBUF = Temp_Data;
	while (UCA0STATW & UCBUSY)
		;
	spi_csh();
}

unsigned char Recieve_RegisterData(volatile uint8_t RegisterAddr) {
	spi_csl();
	__delay_cycles(20);
	UCA0TXBUF = RegisterAddr;
	while (UCA0STATW & UCBUSY)
		;
	UCA0TXBUF = 0x00;
	while (UCA0STATW & UCBUSY)
		;
	spi_csh();
	return UCA0RXBUF;
}

void Modem_Config1_Init(volatile Modem_Config1* ModemConfig) {
	u8 Temp_Data = 0x0;
	Temp_Data = (ModemConfig->ImplicitHeaderModeOn + ModemConfig->CodingRate
			+ ModemConfig->BandWidth);
	Send_Data(RegModemConfig1, Temp_Data);
}

void Modem_Config1_Get(volatile Modem_Config1* ModemConfig) {
	u8 Temp_Data = 0x00;
	;
	Temp_Data = Recieve_RegisterData(RegModemConfig1);

	ModemConfig->ImplicitHeaderModeOn = Temp_Data & (~RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK);

	ModemConfig->CodingRate = Temp_Data & (~RFLR_MODEMCONFIG1_CODINGRATE_MASK);

	ModemConfig->BandWidth = Temp_Data & (~RFLR_MODEMCONFIG1_BW_MASK);
}

void Modem_Config2_Init(volatile Modem_Config2* ModemConfig) {
	u8 Temp_Data = 0x0;
	Temp_Data = (ModemConfig->RxPayLoadCrcOn + ModemConfig->SpreadingFactor
			+ ModemConfig->Tx_Continuos_Mode /*+ ModemConfig->SymbTSYMBTIMEOUTMSB*/);
	Send_Data(RegModemConfig2, Temp_Data);
}

void Modem_Config2_Get(volatile Modem_Config2* ModemConfig) {
	u8 Temp_Data = 0x00;
	;
	Temp_Data = Recieve_RegisterData(RegModemConfig2);

	ModemConfig->RxPayLoadCrcOn = Temp_Data & (~RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK);

	ModemConfig->SpreadingFactor = Temp_Data & (~RFLR_MODEMCONFIG2_SF_MASK);

	ModemConfig->Tx_Continuos_Mode = Temp_Data & (~RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK);
}

void Modem_Config3_Init(volatile Modem_Config3* ModemConfig) {
	u8 Temp_Data = 0x0;
	Temp_Data = (ModemConfig->AGC + ModemConfig->LowDataRateOptimize);
	Send_Data(RegModemConfig3, Temp_Data);
}

void Modem_Config3_Get(volatile Modem_Config3* ModemConfig) {
	u8 Temp_Data = 0x00;
	;
	Temp_Data = Recieve_RegisterData(RegModemConfig3);

	ModemConfig->AGC = Temp_Data & (~RFLR_MODEMCONFIG3_AGCAUTO_MASK);

	ModemConfig->LowDataRateOptimize = Temp_Data & (~RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK);
}

void OCP_Config_Init(volatile OCP_Config* OCPConfig) {
	u8 Temp_Data = 0x0;
	Temp_Data = ((OCPConfig->OCPTriming) + (OCPConfig->OCP_ON));
	Send_Data(RegOcp, Temp_Data);
}

void OCP_Config_Get(volatile OCP_Config *OCPConfig) {
	u8 Temp_Data = 0x00;
	;
	Temp_Data = Recieve_RegisterData(RegOcp);

	OCPConfig->OCPTriming = Temp_Data & (~RFLR_OCP_TRIM_MASK);

	OCPConfig->OCP_ON = Temp_Data & (~RFLR_OCP_MASK);
}

void Pa_Config_Init(volatile PaConfig* Pa_Config) {
	u8 Temp_Data = 0x0;

	Temp_Data = (((( uint8_t )(Pa_Config->MaxPower))<<4) + (( uint8_t )(Pa_Config->OutputPower))
			+ Pa_Config->PaSelect);
	Send_Data(RegPaConfig, Temp_Data);

}

void Pa_Config_Get(volatile PaConfig* Pa_Config) {
	u8 Temp_Data = 0x00;
	;
	Temp_Data = Recieve_RegisterData(RegPaConfig);

	Pa_Config->PaSelect = Temp_Data & (~RFLR_PACONFIG_PASELECT_MASK);

	Pa_Config->MaxPower = (Temp_Data & (~RFLR_PACONFIG_MAX_POWER_MASK)) >> 4;

	Pa_Config->OutputPower = Temp_Data & (~RFLR_PACONFIG_OUTPUTPOWER_MASK);
}

void LNAConfig_Init(volatile LNA_Config *LNAConfig) {
	u8 Temp_Data = 0x0;
	Temp_Data = (LNAConfig->LNA_BoostHf + LNAConfig->LNA_BoostLF
			+ LNAConfig->LNA_GAIN);
	Send_Data(RegLna, Temp_Data);

}

void LNAConfig_Get(volatile LNA_Config* LNAConfig) {
	u8 Temp_Data = 0x00;
	;
	Temp_Data = Recieve_RegisterData(RegLna);

	LNAConfig->LNA_BoostHf = Temp_Data & (~RFLR_LNA_BOOST_HF_MASK);

	LNAConfig->LNA_BoostLF = Temp_Data & (~RFLR_LNA_BOOST_LF_MASK);

	LNAConfig->LNA_GAIN = Temp_Data & (~RFLR_LNA_GAIN_MASK);
}

void Rx_TimeOut(volatile u8 Rx_TimeoutValue) {
	Send_Data(RegSymbTimeoutLsb, Rx_TimeoutValue);

}

void OPMode_Init(volatile OpMode *Op_Mode) {
	u8 Temp_Data = 0x0;
	;
	Temp_Data = (Op_Mode->LongRangeMode + Op_Mode->Low_FrequencyModeOn
			+ Op_Mode->Mode);
	Send_Data(RegOpMode, Temp_Data);

}

void OPMode_Get(volatile OpMode *Op_Mode) {
	u8 Temp_Data = 0x00;
	;
	Temp_Data = Recieve_RegisterData(RegOpMode);

	Op_Mode->LongRangeMode = Temp_Data & (~RFLR_OPMODE_LONGRANGEMODE_MASK);

	Op_Mode->Low_FrequencyModeOn = Temp_Data & (~RFLR_OPMODE_FREQMODE_ACCESS_MASK);

	Op_Mode->Mode = Temp_Data & (~RFLR_OPMODE_MASK);

}

void Packet_Set(u8 PreambleLenght, u8 PayloadLenght) {
	volatile u8 temp = 0;
	PreambleLenght -= 4;
	temp = (u8) (PreambleLenght) & 0xFF;
	Send_Data(RegPreambleLsb, temp);
	temp = (u8) (PreambleLenght >> 8) & 0xFF;
	Send_Data(RegPreambleMsb, temp);
	if (PayloadLenght <= 255) {
		Send_Data(RegPayloadLenght, PayloadLenght);
	}
	Send_Data(RegSymbTimeoutLsb, 0xff);
}

void Packet_Get(u8* PreambleLenght, u8* PayloadLenght) {
	u8 Temp_Data = 0x00;
	;
	*PreambleLenght=0;
	Temp_Data = Recieve_RegisterData(RegPreambleLsb);
	*PreambleLenght+=Temp_Data;
	Temp_Data = Recieve_RegisterData(RegPreambleMsb);
	*PreambleLenght+=(Temp_Data<<8);
	*PreambleLenght+=4;
	*PayloadLenght=Recieve_RegisterData(RegPayloadLenght);
}

void PLL_Set(u8 PLL_TypeDef) {
	Send_Data(RegPll, PLL_TypeDef);
}

void PLL_Get(u8* PLL_TypeDef) {
	*PLL_TypeDef= Recieve_RegisterData(RegPll);
}

//void Frequency_Set(volatile unsigned long int frequency)
void Frequency_Set(unsigned long long frequency) {
	/*
	 * Uso la formula data dal datasheet
	 * fRF=F(XOSC)*⋅ Frf/2^19
	 * trovata la risoluzione basta dividere la frequenza voluta per la risoluzione
	 * la risoluzione vale 61.035 Hz data con Frf=1Hz
	 */
	unsigned long long fRF = frequency / 61.035;
// devo convertitrlo in char e mandare i valori singoli
	u8 temp = 0;
	temp = (u8) (fRF) & 0xFF;
	Send_Data(RegFrLsb, temp);
	temp = (u8) (fRF >> 8) & 0xFF;
	Send_Data(RegFrMid, temp);
	temp = (u8) (fRF >> 16) & 0xFF;
	Send_Data(RegFrMsb, temp);
}

void Frequency_Get(unsigned long long* frequency) {
	unsigned long long fRF=0;
	u32 temp = 0;
	temp = Recieve_RegisterData(RegFrLsb);
	fRF+=temp;
	temp = Recieve_RegisterData(RegFrMid);
	fRF+=(temp<<8);
	temp = Recieve_RegisterData(RegFrMsb);
	fRF+=(temp<<16);
	*frequency=fRF*61.035;
}

int GetFlag_Status(uint8_t FLAG) {
	if (Recieve_RegisterData(RegIRQFlags)) {
		return 1;
	} else {
		return 0;
	};
}

void Clear_Flag(uint8_t FLAG) {
	Send_Data(RegIRQFlags, 0x0);
}

int Number_ValidHeader(u8 Register_Value) {
	unsigned int NumberOfValidHeader = 0;
	NumberOfValidHeader |= Recieve_RegisterData(RegRxHeaderCntValueLsb);
	NumberOfValidHeader |= Recieve_RegisterData(RegRxHeaderCntValueMsb);
	return NumberOfValidHeader;
}

int Number_ValidPackets(u8 Register_Value) {
	unsigned int NumberOfValidPackets = 0;
	NumberOfValidPackets |= Recieve_RegisterData(RegRxPacketCntValueLsb);
	NumberOfValidPackets |= Recieve_RegisterData(RegRxPacketCntValueMsb);
	return NumberOfValidPackets;
}

int Number_Payload(u8 Register_Value) {
	unsigned int NumberOfPayloadByte = 0;
	NumberOfPayloadByte = Recieve_RegisterData(RegRxNbBytes);
	return NumberOfPayloadByte;
}

u8 GetModem_Status() {
//u8 ModemState;
	u8 ModemState = Recieve_RegisterData(RegModemStat);
	return ModemState;
}

/*
 * Stima del SNR dell'Ultimo pacchetto.
 *  Il risultato è in dB
 *
 */
void PaRamp_Set(u8 value) {
	Send_Data(RegPaRamp, value);
}

void PaRamp_Get(u8* value) {
	*value = Recieve_RegisterData(RegPaRamp);
}

void Manual_Reset() {
	/* Il Modem si resetta ad ogni accensione
	 * laddove non è possibile staccare fisicamente
	 * il pin NReset dal VDD si può procedere con il reset manuale
	 * Il pin Nrest va portato allo 0 logico per un tempo maggiore i 100us
	 * e dopo va porato il alta impedenza
	 * bisogna aspettare almeno 5 ms prima di
	 * procedere con le impostazioni
	 *
	 */
}

void Packet_Tx(int BufferSize, char * DataTx) {
	int DataCounter = 0;
	if (OpMode_Struct.Low_FrequencyModeOn == Mode_LF) {
//P1OUT=!CPS;
	}
	OpMode_Struct.Mode = Mode_Tx;
	OPMode_Init(&OpMode_Struct);
	Send_Data(RegFifoAddrPtr, 0x80); // Settare il puntatore sulla locazione di memoria da trasmettere
	while ((DataTx[DataCounter] != '\0') && (DataCounter < BufferSize)) {
		Send_Data(RegFifo, DataTx[DataCounter]);
		DataCounter++;
	}
	OpMode_Struct.Mode = Mode_Sleep;
	OPMode_Init(&OpMode_Struct);
}

char ciccio[10];
u8 flag;
u8 mm;
void Packet_RxSingle(volatile int PayLoadLenghtSet_Value, char Rx_Data[]) {
	volatile int counter = 0;
	volatile int Modem_Status = 0;
	if (OpMode_Struct.Low_FrequencyModeOn == Mode_LF) {
//	P1OUT|=CPS;
	}
	OpMode_Struct.Mode = Mode_RxSingle;
	OPMode_Init(&OpMode_Struct);
	Send_Data(RegFifoRxByteAddr, PayLoadLenghtSet_Value);
	Send_Data(RegFifoAddrPtr, 0x00);
	Rx_TimeOut(0x1F);
	Modem_Status = GetModem_Status();
	if (Modem_Status
			& (ModemStat_SignalDetected + ModemStat_SignalSynchronized
					+ ModemStat_Header_Info_Valide) == 0xB) {
//	     P1OUT^=0x01;
		while (counter < PayLoadLenghtSet_Value) {
			Rx_Data[counter] = Recieve_RegisterData(RegFifo);
			counter++;
		}
	}
	flag = Recieve_RegisterData(RegIRQFlags);
	counter = 0;
//reset di tutti i flag
	Send_Data(RegIRQFlags, 0x00);
}

void Packet_RxContinuos(volatile int PayLoadLenghtSet_Value, char Rx_Data[]) {
	volatile int counter = 0;
	volatile int Modem_Status = 0;
	if (OpMode_Struct.Low_FrequencyModeOn == Mode_LF) {
//	P1OUT|=CPS;
	}
	OpMode_Struct.Mode = Mode_RxContinuous;
	OPMode_Init(&OpMode_Struct);
	Send_Data(RegFifoRxByteAddr, PayLoadLenghtSet_Value);
	Send_Data(RegFifoAddrPtr, 0x00);
	Rx_TimeOut(0x1F);
	Modem_Status = GetModem_Status();
	if (Modem_Status
			& (ModemStat_SignalDetected + ModemStat_SignalSynchronized
					+ ModemStat_Header_Info_Valide) == 0xB) {
//		P1OUT^=0x01;
		while (counter < PayLoadLenghtSet_Value) {
			Rx_Data[counter] = Recieve_RegisterData(RegFifo);
			counter++;
		}
	}
	flag = Recieve_RegisterData(RegIRQFlags);
	counter = 0;
//reset di tutti i flag
	Send_Data(RegIRQFlags, 0x00);
}

void Packet_Rx(volatile int PayLoadLenghtSet_Value, char Rx_Data[]) {
	volatile int counter = 0;
	volatile int Modem_Status = 0;
	volatile u8 AddrLastByteRx = 0;
	if (OpMode_Struct.Low_FrequencyModeOn == Mode_LF) {
//	P1OUT|=CPS;
	}
	OpMode_Struct.Mode = Mode_RxContinuous;
	OPMode_Init(&OpMode_Struct);
	AddrLastByteRx = Recieve_RegisterData(RegFifoRxCurrentAdrr);
	Send_Data(RegFifoAddrPtr, AddrLastByteRx);
	Rx_TimeOut(0x1F);
	Modem_Status = GetModem_Status();
	if (Modem_Status
			& (ModemStat_SignalDetected + ModemStat_SignalSynchronized
					+ ModemStat_Header_Info_Valide) == 0xB) {
//		P1OUT^=0x01;
		while (counter < PayLoadLenghtSet_Value) {
			Rx_Data[counter] = Recieve_RegisterData(RegFifo);
			counter++;
		}
	}
	flag = Recieve_RegisterData(RegIRQFlags);
	counter = 0;
//reset di tutti i flag
	Send_Data(RegIRQFlags, 0x00);
}

void DIO_Configuratio_00_RXDONE (){

	Send_Data( REG_LR_IRQFLAGSMASK, 0x00);

	Send_Data( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                  //RFLR_IRQFLAGS_RXDONE |
                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                  RFLR_IRQFLAGS_VALIDHEADER |
                                  RFLR_IRQFLAGS_TXDONE |
                                  RFLR_IRQFLAGS_CADDONE |
                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                  RFLR_IRQFLAGS_CADDETECTED );

	Send_Data( REG_DIOMAPPING1, ( Recieve_RegisterData( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_00 );
}

void DIO_Configuratio_01_TXDONE (){

	Send_Data( REG_LR_IRQFLAGSMASK, 0x00);

	Send_Data( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
                                  RFLR_IRQFLAGS_RXDONE |
                                  RFLR_IRQFLAGS_PAYLOADCRCERROR |
                                  RFLR_IRQFLAGS_VALIDHEADER |
								  //RFLR_IRQFLAGS_TXDONE |
                                  RFLR_IRQFLAGS_CADDONE |
                                  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
                                  RFLR_IRQFLAGS_CADDETECTED );

	Send_Data( REG_DIOMAPPING1, ( Recieve_RegisterData( REG_DIOMAPPING1 ) & RFLR_DIOMAPPING1_DIO0_MASK ) | RFLR_DIOMAPPING1_DIO0_01 );
}
