/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2016 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

//***********************************************************************************************************
//  Nuvoton Technoledge Corp. 
//  Website: http://www.nuvoton.com
//  E-Mail : MicroC-8bit@nuvoton.com
//  Date   : Apr/21/2016
//***********************************************************************************************************

#include "N76E003.h"
#include "Function_define.h"


void InitialUART0_Timer3(UINT32 u32Baudrate)
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

/*==========================================================================*/
//#ifdef SW_Reset
void SW_Reset(void)
{
    TA = 0xAA;
    TA = 0x55;
    set_SWRST;
}
//#endif
/*==========================================================================*/
