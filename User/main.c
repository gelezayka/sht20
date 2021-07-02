#include <string.h>

#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "Common.h"
#include "Delay.h"

#include "PetitModbusPort.h"
#include "PetitModbus.h"

#define ADDR_BASE 0x4700

#define I2C_CLOCK 														2
#define SHT20_ADDR 														0x80
#define SHT20_READ 														0x01
#define SHT20_WRITE 													0x00
#define SHIFTED_DIVISOR                       0x988000
#define ERROR_I2C                    					0x03E6
#define ERROR_BAD_CRC                         0x03E7
#define TRIGGER_TEMP_MEASURE_HOLD             0xE3
#define TRIGGER_HUMD_MEASURE_HOLD             0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD           0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD           0xF5

bit I2C_Reset_Flag;
volatile unsigned char xdata page_buffer[128];
volatile UINT16 xdata TimerCounter=0;   
UINT8 riflag = 0, pflag = 0;
UINT8 xdata BUF[16];
UINT8 in_c;

#define MAGIC 0x34982315

typedef struct {
		UINT32 magic;
		unsigned char address;
		short baudRate;
		short tc;
		short hc;
} config_t;

config_t xdata config;

UINT8 Read_APROM_BYTE(UINT16 code *u16_addr)
{
        UINT8 rdata;
        rdata = *u16_addr>>8;
        return rdata;
}

void Write_DATAFLASH_BYTE(unsigned int u16EPAddr,unsigned char u8EPData)
{
		unsigned char looptmp=0;
		unsigned int u16_addrl_r;
		unsigned int RAMtmp;

		//Check page start address
		u16_addrl_r=(u16EPAddr/128)*128;
		//Save APROM data to XRAM0
		for(looptmp=0;looptmp<0x80;looptmp++)
		{
			RAMtmp = Read_APROM_BYTE((unsigned int code *)(u16_addrl_r+looptmp));
			page_buffer[looptmp]=RAMtmp;
		}
		// Modify customer data in XRAM
		page_buffer[u16EPAddr&0x7f] = u8EPData;

		//Erase APROM DATAFLASH page
		IAPAL = u16_addrl_r&0xff;
		IAPAH = (u16_addrl_r>>8)&0xff;
		IAPFD = 0xFF;
		set_IAPEN;
		set_APUEN;
		IAPCN = 0x22;
		set_IAPGO;
		//Save changed RAM data to APROM DATAFLASH
		set_IAPEN;
		set_APUEN;
		IAPCN = 0x21;
		for(looptmp=0;looptmp<0x80;looptmp++)
		{
			IAPAL = (u16_addrl_r&0xff)+looptmp;
			IAPAH = (u16_addrl_r>>8)&0xff;
			IAPFD = page_buffer[looptmp];
			set_IAPGO;
		}
		clr_APUEN;
		clr_IAPEN;
}


void readConfig()
{
	unsigned int RAMtmp;
	UINT16 i;
	UINT16 code *u16_addr;
	for(i = 0; i < sizeof(config_t); i++) {
		u16_addr = ADDR_BASE+i;
		RAMtmp = *u16_addr>>8;
		page_buffer[i&0x7f] = RAMtmp;
	}
	memcpy(&config, page_buffer, sizeof(config_t));
}

void writeConfig()
{
	unsigned char looptmp=0;
	unsigned int u16_addrl_r = ADDR_BASE;
	memcpy(page_buffer, &config, sizeof(config_t));
	
	// Erase APROM DATAFLASH page
	set_IAPEN;
	IAPFD = 0xFF;
	IAPCN = 0x22;
	set_APUEN;
	IAPAL = u16_addrl_r&0xff;
	IAPAH = (u16_addrl_r>>8)&0xff;	
	set_IAPGO;
	
	//Save changed RAM data to APROM DATAFLASH
	set_IAPEN;
	set_APUEN;
	IAPCN = 0x21;
	for(looptmp=0;looptmp<0x80;looptmp++)
	{
		IAPAL = (u16_addrl_r&0xff)+looptmp;
		IAPAH = (u16_addrl_r>>8)&0xff;
		IAPFD = page_buffer[looptmp];
		set_IAPGO;
	}
	clr_APUEN;
	clr_IAPEN;
}

void I2C_SI_Check(void)
{
  if (I2STAT == 0x00)
  {
     I2C_Reset_Flag = 1;
     set_STO;
     SI = 0;
     if(SI)
     {
       clr_I2CEN;
       set_I2CEN;
       clr_SI;
       clr_I2CEN;
    }
  }
}
 
UINT8 checkCRC(UINT16 message_from_sensor, UINT8 check_value_from_sensor)
{
	UINT8 i;
	UINT32 remainder = (UINT32)message_from_sensor << 8;
	UINT32 divsor = (UINT32)SHIFTED_DIVISOR;
	remainder |= check_value_from_sensor;
	for(i = 0 ; i < 16 ; i++){
    if(remainder & (UINT32)1 << (23 - i)){
			remainder ^= divsor;
    }
		divsor >>= 1;
  }
	return (UINT8)remainder;
}

UINT16 readValue(UINT8 cmd)
{
	UINT8 msb, lsb, checksum;
	UINT16 rawValue = ERROR_I2C;
	
	set_STA;
	clr_SI;
	while (!SI);
	if (I2STAT != 0x08) {
		I2C_Reset_Flag = 1;
		goto Read_Error_Stop;
	}
	
	// send write
	I2DAT = (SHT20_ADDR | SHT20_WRITE);
	clr_STA;
	clr_SI;
	while (!SI);
	if (I2STAT != 0x18) {
		I2C_Reset_Flag = 1;
		goto Read_Error_Stop;
	}
	
	
	// send command
	I2DAT = cmd; // Trigger T measurement
	clr_SI;
	while (!SI);
	if (I2STAT != 0x28) {
		I2C_Reset_Flag = 1;
		goto Read_Error_Stop;
	}
	
	//clr_STA;
	set_STA;
	clr_SI;
	while (!SI);
	if (I2STAT != 0x10) {
		I2C_Reset_Flag = 1;
		goto Read_Error_Stop;
	}
	
	// send read
	//set_AA;                             /* Set Assert Acknowledge Control Bit */
	clr_STA;                              /* Clear STA and Keep SI value in I2CON */
	I2DAT = (SHT20_ADDR | SHT20_READ);	
	clr_SI;
  while (!SI);
	if (I2STAT != 0x40) {
		I2C_Reset_Flag = 1;
		goto Read_Error_Stop;
	}
	
	Timer0_Delay1ms(100);
	
	set_AA;                             /* Set Assert Acknowledge Control Bit */
	clr_SI;
  while (!SI);
	if (I2STAT != 0x50) {
		I2C_Reset_Flag = 1;
		goto Read_Error_Stop;
	}
	msb = I2DAT;

	set_AA;                             /* Set Assert Acknowledge Control Bit */
	clr_SI;
  while (!SI);
	if (I2STAT != 0x50) {
		I2C_Reset_Flag = 1;
		goto Read_Error_Stop;
	}
	lsb = I2DAT;

	set_AA;                             /* Set Assert Acknowledge Control Bit */
	clr_SI;
  while (!SI);
	if (I2STAT != 0x50) {
		I2C_Reset_Flag = 1;
		goto Read_Error_Stop;
	}
	checksum = I2DAT;

	clr_SI;
	set_STO;
	while (STO)                        /* Check STOP signal */
	{
		I2C_SI_Check();
	}
	
	rawValue = ((UINT16) msb << 8) | (UINT16) lsb;
	
	if (checkCRC(rawValue, checksum) != 0) {
			return (ERROR_BAD_CRC);
	}
	
Read_Error_Stop:
	if (I2C_Reset_Flag) {
		I2C_SI_Check();
		I2C_Reset_Flag = 0;
	}

	return rawValue & 0xFFFC;
}

float readHumidity()
{
	UINT16 rawHumidity;
	rawHumidity = readValue(TRIGGER_HUMD_MEASURE_HOLD);
	return (rawHumidity * (125.0 / 65536.0)) - 6.0;
}

float readTemperature()
{
	UINT16 rawTemperature;
	rawTemperature = readValue(TRIGGER_TEMP_MEASURE_HOLD);
	return (rawTemperature * (175.72 / 65536.0)) - 46.85;
}

void Timer2_ISR (void) interrupt 5
{
	clr_TF2;
	//TimerCounter++;
	PetitModBus_TimerValues();
}

void SerialPort0_ISR(void) interrupt 4
{
	if (RI == 1)
	{               		                           	
		ReceiveInterrupt(SBUF);
		RI = 0;
	}
  if(TI == 1)
  {
    clr_TI;                             
  }
}

void TM_ini(void)
{
	TIMER2_DIV_16;
	TIMER2_Auto_Reload_Delay_Mode;
	RCMP2L = TIMER_DIV16_VALUE_10ms;
	RCMP2H = TIMER_DIV16_VALUE_10ms>>8;
	TL2 = 0;
	TH2 = 0;
	set_ET2;                                    // Enable Timer2 interrupt
	set_EA;
	set_TR2;                                    // Timer2 run
}

void UART0_ini(UINT32 u32Baudrate)
{
  P06_Quasi_Mode;    //Setting UART pin as Quasi mode for transmit
  P07_Quasi_Mode;    //Setting UART pin as Quasi mode for transmit  
	SCON = 0x50;     //UART0 Mode1,REN=1,TI=1
  set_SMOD;        //UART0 Double Rate Enable
  T3CON &= 0xF8;   //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
  set_BRCK;        //UART0 baud rate clock source = Timer3
  RH3 = HIBYTE(65536 - (1000000/u32Baudrate)-1); 
	RL3 = LOBYTE(65536 - (1000000/u32Baudrate)-1);
	set_TR3;         //Trigger Timer3
	set_ES;
  set_EA;
}

void readSensors()
{
	float t, h;
	t = readTemperature() + config.tc/10;
	h = readHumidity() + config.hc/10;;	
	PetitRegisters[0].ActValue = t * 10;
	PetitRegisters[1].ActValue = h * 10;
}

void checkConfig()
{
	if ((config.tc != PetitRegisters[5].ActValue) ||
					(config.hc != PetitRegisters[6].ActValue) ||
					(config.address != PetitRegisters[3].ActValue) ||
					(config.baudRate != PetitRegisters[4].ActValue)
	) {
		config.address = PetitRegisters[3].ActValue;
		config.baudRate = PetitRegisters[4].ActValue;
		config.tc = PetitRegisters[5].ActValue;
		config.hc = PetitRegisters[6].ActValue;
		writeConfig();
		InitPetitModbus(config.address);
	}
}

void main (void)
{
	UINT16 code *u16_addr;
	Set_All_GPIO_Quasi_Mode;
	I2CLK = I2C_CLOCK;
	set_I2CEN;
	P15_PushPull_Mode;
	
	Write_DATAFLASH_BYTE (0x3802, 0x34);
	readConfig();
	if (config.magic != MAGIC) {
		config.magic = MAGIC;
		config.address = 1;
		config.baudRate = 9600;
		config.tc = 0;
		config.hc = 0;
		writeConfig();
	}
	
	InitPetitModbus(config.address);
	
	UART0_ini(9600);
	TM_ini();

	// Temperature
	PetitRegisters[0].Addr = 0x001;
	PetitRegisters[0].ActValue = 0;
	// Humidity
	PetitRegisters[1].Addr = 0x002;
	PetitRegisters[1].ActValue = 0;
	
	// Timeout
	PetitRegisters[2].Addr = 0x003;
	PetitRegisters[2].ActValue = 0;
	
	// Device Address
	PetitRegisters[3].Addr = 0x0101;
	PetitRegisters[3].ActValue = config.address;
	
	// Baud Rate 0:9600 1:14400 2:19200
	PetitRegisters[4].Addr = 0x0102;
	PetitRegisters[4].ActValue = config.baudRate;
	// Temperature correction(/10) -10.0~10.0
	PetitRegisters[5].Addr = 0x0103;
	PetitRegisters[5].ActValue = config.tc;
	// Humidity correction(/10) -10.0~10.0
	PetitRegisters[6].Addr = 0x0104;
	PetitRegisters[6].ActValue = config.hc;
	
	// enable watchdog timer
	TA = 0xAA;
  TA = 0x55;
  WDCON = 0x07;                       
  set_WDCLR;                                                  
  while((WDCON | ~SET_BIT6) == 0xFF);         
  EA = 1;
  set_WDTR;                                                       
	
	P15 = 0; // enable RX
	while(1)  {
		ProcessPetitModbus();
	}
}