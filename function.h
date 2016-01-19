#ifndef function_H_
#define function_H_
#ifdef __cplusplus
extern "C" {
#endif
#define RegFifo 0x00 //LoRaTM base-band FIFO data input/output. FIFO is cleared an not accessible when device is in SLEEP mode
#define RegOpMode 0x01
#define RegBitrateMsb 0x02
#define RegBitrateLsb 0x03
#define RegFdevMsb 0x04
#define RegFdevLsb 0x05
#define RegFrMsb 0x06 //MSB of RF carrier frequency Register values must be modified only when device is in SLEEP or STAND-BY mode
#define RegFrMid 0x07 //MSB of RF carrier frequency Register values must be modified only when device is in SLEEP or STAND-BY mode
#define RegFrLsb 0x08 //LSB of RF carrier frequency Register values must be modified only when device is in SLEEP or STAND-BY mode
#define RegPaConfig 0x09
#define RegPaRamp 0x0A
#define RegOcp 0x0B
#define RegLna 0x0C
#define RegFifoAddrPtr 0x0D //SPI interface address pointer in FIFO data buffer.
#define RegFifoTxBaseAdrr 0x0E //write base address in FIFO data buffer for TX modulator
#define RegFifoRxBaseAdrr 0x0F //read base address in FIFO data buffer for RX demodulator
#define RegFifoRxCurrentAdrr 0x10 //Start address (in data buffer) of last packet received
#define RegIRQFlagMask 0x11
#define RegIRQFlags 0x12
#define RegRxNbBytes 0x13 //Number of payload bytes of latest packet received
#define RegRxHeaderCntValueMsb 0x14 //Number of valid headers received since last transition into Rx modeMSB(15:8) Header and packet counters are reseted in Sleep mode
#define RegRxHeaderCntValueLsb 0x15  //Number of valid headers received since last transition into Rx modeMSB(7:0) Header and packet counters are reseted in Sleep mode
#define RegRxPacketCntValueMsb 0x16  //Number of valid packets received since last transition into Rx mode MSB(15:8) Header and packet counters are reseted in Sleep mode
#define RegRxPacketCntValueLsb 0x17  //Number of valid packets received since last transition into Rx mode MSB(7:0) Header and packet counters are reseted in Sleep mode
#define RegModemStat 0x18
#define RegPktSnrValue 0x19
#define RegPktRssiValue 0x1A
#define RegRssiValue 0x1B
#define RegHopChannel 0x1C
#define RegModemConfig1 0x1D
#define RegModemConfig2 0x1E
#define RegSymbTimeoutLsb 0x1F
#define RegPreambleMsb 0x20
#define RegPreambleLsb 0x21
#define RegPayloadLenght 0x22
#define RegMaxPayloadLenght 0x23
#define RegHopPeriod 0x24
#define RegFifoRxByteAddr 0x25
#define RegModemConfig3 0x26
#define RegDioMapping1 0x40 //Mapping of pins DIO0 to DIO3
#define RegDioMapping2 0x41 //Mapping of pins DIO4 and DIO5, ClkOut frequency
#define RegVersion //Semtech ID relating the silicon reision
#define RegTcxo 0x4B
#define RegPaDac 0x4D
#define RegFormerTemp 0x5B
#define RegAgcRef 0x61
#define RegAgcThershold1 0x61
#define RegAgcThershold2 0x62
#define RegAgcThershold3 0x63
#define RegPll 0x70
#define ClearFlag 0x0
#define FXOSC 32000000
#define uint8_t unsigned short int
#define uint16_t unsigned int
#define uint32_t unsigned long
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define mhr 0x80

typedef enum {
	Rx_Done = 0x00, Tx_Done = 0x01
} DIO_0_Mode;

typedef enum {
	Mode_Sleep = 0x00,
	Mode_STDBY = 0x01,
	Mode_FSTX = 0x02,
	Mode_Tx = 0x03,
	Mode_FSRX = 0x04,
	Mode_RxContinuous = 0x05,
	Mode_RxSingle = 0x06,
	Mode_Cad = 0x07
} Mode_TypeDef;

typedef enum {
	Mode_HF = 0x00, Mode_LF = 0x08
} Low_FrequencyModeOn_TypeDef;

typedef enum {
	Mode_FSK = 0x00, Mode_Lora = 0x80
} LongRangeMode_TypeDef;

typedef enum {
	LNAGAIN_NOTUSED = 0x00,
	LNAGAIN_G1 = 0x20,
	LNAGAIN_G2 = 0x40,
	LNAGAIN_G3 = 0x60,
	LNAGAIN_G4 = 0x80,
	LNAGAIN_G5 = 0xA0,
	LNAGAIN_G6 = 0xC0
} LNA_GAIN_TypeDef;

typedef enum {
	LNABOOSTHF_Default = 0x00, LNABOOSTHF_BOOSTON = 0x03
} LNA_BoostHf_TypeDef;

typedef enum {
	LNA_BOOSTLF_Default = 0x00 /*ONLY DEFAULT*/
} LNA_BoostLF_TypeDef;

/*FLAG_MASK
 *
 */

#define RxTimeoutMASK 0x7
#define RxDoneMask 0x6
#define PayloadCrcErrorMask 0x5
#define ValidHeaderMask 0x4
#define TxDoneMask 0x3
#define CadDoneMask 0x2
#define FhssChangeChannelMask 0x1
#define CadDectedMask 0x0

/* FLAG
 *
 *
 */
#define RxTimeout 0x80 //TimeoutInterrupt
#define RxDone 0x40 //Packet Reception Complete Interrupt
#define PayloadCrcError 0x20 // Payload CRC error Interrupt
#define ValidHeader 0x10 // Valid Header received in RX
#define TxDone 0x8 // Fifo Payload transmission complete interrupt
#define CadDone 0x4 // CAD Complete
#define FhssChangeChannel 0x2 // FHSS change channel interrupt
#define CadDected 0x1 // valid Lora signal detected during cad operation

/*
 * per cancellare i vari flag basta mandare 0x0
 *
 *
 */
typedef enum {
	ModemStat_SignalDetected = 0x1,
	ModemStat_SignalSynchronized = 0x2,
	ModemStat_RxOn_Goingg = 0x4,
	ModemStat_Header_Info_Valide = 0x8,
	ModemSta_Modem_Clear = 0x10,
	ModemStat_RxCodingRate = 0x20
} Modem_State_TypeDef;

typedef enum {
	Bandwidth_7_8KHz = 0x0,
	Bandwidth_10_4KHz = 0x10,
	Bandwidth_15_6KHz = 0x20,
	Bandwidth_20_8KHz = 0x30,
	Bandwidth_31_25KHz = 0x40,
	Bandwidth_41_7KHz = 0x50,
	Bandwidth_62_5KHz = 0x60,
	Bandwidth_125KHz = 0x70,
	Bandwidth_250KHz = 0x80,
	Bandwidth_500KHz = 0x90
} BandWidth_TypeDef;
typedef enum {
	CodingRate_4_5 = 0x02,
	CodingRate_4_6 = 0x04,
	CodingRate_4_7 = 0x06,
	CodingRate_4_8 = 0x08
} CodingRate_TypeDef;
typedef enum {
	ExplicitHeaderMode = 0x0, ImplicitHeaderMode = 0x1
} ImplicitHeaderModeOn_TypeDef;
typedef enum {
	SpreadingFactor_6 = 0x60,
	SpreadingFactor_7 = 0x70,
	SpreadingFactor_8 = 0x80,
	SpreadingFactor_9 = 0x90,
	SpreadingFactor_10 = 0xA0,
	SpreadingFactor_11 = 0xB0,
	SpreadingFactor_12 = 0xC0
} SpreadingFactor_TypeDef;
typedef enum {
	Tx_ContinuosMode_NormalMode = 0x0, Tx_ContinuosMode_ContinuosMode = 0x8
} Tx_Continuos_Mode_TypeDef;

typedef enum {
	Header_Indicates_CRC_OFF = 0x0, Header_Indicates_CRC_ON = 0x4
} RxPayLoadCrcOn_TypeDef;

typedef enum {
	Paramp_3_4_ms = 0x0,
	Paramp_2ms = 0x1,
	Paramp__1ms = 0x2,
	Paramp_500us = 0x3,
	Paramp_250us = 0x4,
	Paramp_125us = 0x5,
	Paramp_100us = 0x6,
	Paramp_62s = 0x7,
	Paramp_50us = 0x8,
	Paramp_40us = 0x9,
	Paramp_31us = 0xA,
	Paramp_25us = 0xB,
	Paramp_20us = 0xC,
	Paramp_15us = 0xD,
	Paramp_12us = 0xE,
	Paramp_10us = 0xF
} Pa_Ramp_TypeDef;

typedef enum {
	PaSelect_RFO_Pin = 0x00, //RFO pin. Output power is limited to +14 dBm.
	PaSelect_PA_Boost_Pin = 0x80 // PA_BOOST pin. Output power is limited to +20 dBm
} PaSelect_TypeDef;

typedef enum {
	OCP_ENABLE = 0x20, /*OverLoad Current Protection */
	OCP_DISBALE = 0x00
} OCP_ON_TypeDef;

typedef enum {
	LowDataRateOpimize_Disable = 0x0, LowDataRateOptimize_Enable = 0x8
} LowDataRateOptimize_TypeDef;
typedef enum {
	AGCAUTO_ON = 0x4,/*LNA GAIN set by the internal AGC loop*/
	AGCAUTO_OFF = 0x0/*LNA GAIN set by register LNAGAIN*/
} AGC_TypeDef;
typedef enum {
	PLL_300MHz = 0xD0, PLL_225MHz = 0x90, PLL_150MHz = 0x50, PLL_75MHz = 0x10
} PLL_TypeDef;
typedef struct {
	SpreadingFactor_TypeDef SpreadingFactor;
	Tx_Continuos_Mode_TypeDef Tx_Continuos_Mode;
	RxPayLoadCrcOn_TypeDef RxPayLoadCrcOn;
} Modem_Config2;
typedef struct {
	BandWidth_TypeDef BandWidth;
	CodingRate_TypeDef CodingRate;
	ImplicitHeaderModeOn_TypeDef ImplicitHeaderModeOn;
} Modem_Config1;
typedef struct {
	LongRangeMode_TypeDef LongRangeMode;
	Mode_TypeDef Mode;
	Low_FrequencyModeOn_TypeDef Low_FrequencyModeOn;
} OpMode;
typedef struct {
	LNA_GAIN_TypeDef LNA_GAIN;
	LNA_BoostHf_TypeDef LNA_BoostHf;
	LNA_BoostLF_TypeDef LNA_BoostLF;
} LNA_Config;

typedef struct {
	PaSelect_TypeDef PaSelect;
	/*
	 *
	 * Pout=Pmax-(15-OutputPower) if PaSelect = 0 (RFO pin)
	 * Pout=17-(15-OutputPower) if PaSelect = 1 (PA_BOOST pin)
	 *
	 *
	 */
	int MaxPower; /* Il valore massimo che può assumere è 3 */
	int OutputPower; /* Il valore massimo che può assumbere è 15*/
} PaConfig;

typedef struct {
	OCP_ON_TypeDef OCP_ON;
	int OCPTriming; /* il massimo valore che può assumere è 31*/
	/*
	 *
	 *
	 * Default Imax=100mA
	 * Imax=45+5*OCPTriming[mA] if OcpTrim <= 15(120mA)
	 * else Imax=-30+10*OCPTriming[mA] if 15<OcpTrim <= 27(130 to 240 mA)
	 * Imax=240mA
	 */
} OCP_Config;

typedef struct {
	LowDataRateOptimize_TypeDef LowDataRateOptimize;
	AGC_TypeDef AGC;
} Modem_Config3;

void Modem_Config1_Init(volatile Modem_Config1 *ModemConfig);
void Modem_Config1_Get(volatile Modem_Config1 *ModemConfig);
void Modem_Config2_Init(volatile Modem_Config2 *ModemConfig);
void Modem_Config2_Get(volatile Modem_Config2 *ModemConfig);
void Modem_Config3_Init(volatile Modem_Config3 *ModemConfig);
void Modem_Config3_Get(volatile Modem_Config3 *ModemConfig);
void Rx_TimeOut(u8 Rx_TimeoutValue);
void Paket_Set(u8 PreambleLenght, u8 PayloadLenght);
void OCP_Config_Init(volatile OCP_Config *OCPConfig);
void OCP_Config_Get(volatile OCP_Config *OCPConfig);
void Pa_Config_Init(volatile PaConfig*Pa_Config);
void Pa_Config_Get(volatile PaConfig* Pa_Config);
void LNAConfig_Init(volatile LNA_Config *LNAConfig);
void LNAConfig_Get(volatile LNA_Config* LNAConfig);
void Packet_Set(u8 PreambleLenght, u8 PayloadLenght);
void Packet_Get(u8* PreambleLenght, u8* PayloadLenght);
void Packet_Tx(int BufferSize, char * DataTx);
void Packet_RxSingle(int PayLoadLenghtSet_Value, char* Rx_Data);
void Packet_RxContinuos(volatile int PayLoadLenghtSet_Value, char Rx_Data[]);
void Packet_Rx(volatile int PayLoadLenghtSet_Value, char Rx_Data[]);
void OPMode_Init(volatile OpMode *Op_Mode);
void OPMode_Get(volatile OpMode *Op_Mode);
void PaRamp_Set(u8 value);
void PaRamp_Get(u8* value);
void PLL_Set(u8 PLL_TypeDef);
void PLL_Get(u8* PLL_TypeDef);
void Struct_Init(uint8_t *Structure);
void Send_Data(uint8_t RegisterAddr, uint8_t Temp_Data);
unsigned char Recieve_Data();
unsigned char Recieve_RegisterData(uint8_t RegisterAddr);
//void Frequency_Set(unsigned long int frequency);
void Frequency_Set(unsigned long long frequency);
void Frequency_Get(unsigned long long* frequency);
int GetFlag_Status(uint8_t FLAG);
void Clear_Flag(uint8_t FLAG);
int Number_Payload(u8 Register_Value);
int Number_ValidHeader(u8 Register_Value);
int Number_ValidePackets(u8 Register_Value);
u8 GetModem_Status();
void PowerSetup();
double PacketSNR();
double PacketRssi();
double Rssi();
void Manual_Reset();
void DIO_0_Configuratio (u8 Register_Value);
void DIO_Configuratio_00_RXDONE ();
void DIO_Configuratio_01_TXDONE ();

#endif
