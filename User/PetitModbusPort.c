//#include "General.h"
#include "PetitModbus.h"
#include "PetitModbusPort.h"
#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"

// This port file for PIC microcontrollers!

// Modbus RTU Variables
volatile unsigned char xdata PetitReceiveBuffer[PETITMODBUS_RECEIVE_BUFFER_SIZE];   // Buffer to collect data from hardware
volatile unsigned char xdata PetitReceiveCounter=0;                                 // Collected data number
extern volatile unsigned char xdata PETITMODBUS_SLAVE_ADDRESS;
extern void readSensors(void);
extern void checkConfig(void);

unsigned short msgCount = 0;
	
// UART Initialize for Microconrollers, yes you can use another phsycal layer!
void PetitModBus_UART_Initialise(void)
{
// Insert UART Init Code Here
//    InitUART();
}

// Timer Initialize for Petit Modbus, 1ms Timer will be good for us!
void PetitModBus_TIMER_Initialise(void)
{
// Insert TMR Init Code Here
//    InitTMR1();
}

// This is used for send one character
void PetitModBus_UART_Putch(unsigned char c)
{
		TI=0;
		SBUF = c;
		set_WDCLR;
		while(TI==0);
}

// This is used for send string, better to use DMA for it ;)
unsigned char PetitModBus_UART_String(unsigned char *s, unsigned int Length)
{
    unsigned short  DummyCounter;
		P15 = 1; // enable TX
    for(DummyCounter=0;DummyCounter<Length;DummyCounter++) {
        PetitModBus_UART_Putch(s[DummyCounter]);
		}
		msgCount++;
		if (msgCount == 60) {
				msgCount = 0;
				SW_Reset();
		}
    P15 = 0; // enable RX
    return TRUE;
}

/*************************Interrupt Fonction Slave*****************************/
// Call this function into your UART Interrupt. Collect data from it!
// Better to use DMA
void ReceiveInterrupt(unsigned char Data)
{
    PetitReceiveBuffer[PetitReceiveCounter]   =Data;
    PetitReceiveCounter++;

    if(PetitReceiveCounter>PETITMODBUS_RECEIVE_BUFFER_SIZE)  
        PetitReceiveCounter=0;

		PetitModbusTimerValue=0;
}

// Call this function into 1ms Interrupt or Event!
void PetitModBus_TimerValues(void)
{
    PetitModbusTimerValue++;
}
/******************************************************************************/

void ProcessPreRead()
{
		readSensors();
}

void ProcessPostWrite()
{
		checkConfig();
}
