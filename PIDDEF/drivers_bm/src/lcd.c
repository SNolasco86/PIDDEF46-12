/* Copyright 2014, 2015 Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Bare Metal example source file
 **
 ** This is a mini example of the CIAA Firmware.
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/
#include "lcd.h"       /* <= own header */

#ifndef CPU
#error CPU shall be defined
#endif
#if (lpc4337 == CPU)
#include "chip.h"
#elif (mk60fx512vlq15 == CPU)
#else
#endif

/*==================[macros and definitions]=================================*/


/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
void InicializarLcd(void)
{

	delay();
	/********************************/
	Chip_GPIO_Init(LPC_GPIO_PORT);
	/*******************************/
	/**Configurar la System Control Unit (SCU), para indicarle las
	características eléctricas de cada pin empleado y remapearlos como puertos
	GPIO**/
		 Chip_SCU_PinMux(4,4,MD_PUP,FUNC0); /* remapea P4_4 en GPIO2[4], LCD1 Y habilita el
		 pull up*/
		 Chip_SCU_PinMux(4,5,MD_PUP,FUNC0); /* remapea P4_5 en GPIO2[5], LCD2 Y habilita
		 el pull up */
		 Chip_SCU_PinMux(4,6,MD_PUP,FUNC0); /* remapea P4_6 en GPIO2[6], LCD3 Y habilita el
		 pull up */
		 Chip_SCU_PinMux(4,10,MD_PUP,FUNC4); /* remapea P4_10 en GPIO5[14], LCD4 Y habilita
		 el pull up */
		 Chip_SCU_PinMux(4,8,MD_PUP,FUNC4); /* remapea P4_8 en GPIO5[12], LCD_RS y habilita
		 el pull up */
		 Chip_SCU_PinMux(4,9,MD_PUP,FUNC4); /* remapea P4_9 en GPIO5[13], LCD_EN y habilita
		 el pull up */
		/*Set up the I/O ports that the LCD module is connected to*/
	    /*Chip_GPIO_SetDir 	( 	LPC_GPIO_T *  	pGPIO,uint8_t  	portNum,uint32_t  	bitValue,uint8_t  	out	) */

		Chip_GPIO_SetDir(LPC_GPIO_PORT, 2, (1<<4),1);	//salida LCD1
		Chip_GPIO_SetDir(LPC_GPIO_PORT, 2, (1<<5),1);	//salida LCD2
		Chip_GPIO_SetDir(LPC_GPIO_PORT, 2, (1<<6),1);	//salida LCD3
		Chip_GPIO_SetDir(LPC_GPIO_PORT, 5, (1<<14),1);	//salida LCD4
		Chip_GPIO_SetDir(LPC_GPIO_PORT, 5, (1<<12),1);	//salida RS
		Chip_GPIO_SetDir(LPC_GPIO_PORT, 5, (1<<13),1);  //salida Enable

		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,13);//Enable=Low

		/* Configure the LCD module for 4 bit input*/

		LCD_EscribirInstruccion(0x33);	// Reset the module
		LCD_EscribirInstruccion(0x32);	// Reset the module
		LCD_EscribirInstruccion(0x2C);	// Set 2 lines, small font, 4 bit mode
		LCD_EscribirInstruccion(0x06);	// Set it to "Cursor move" mode
		LCD_EscribirInstruccion(0x0C);	// Turn display on, turn cursor off
		LCD_EscribirInstruccion(0x02);	// Return cursor to "Home"
		LCD_EscribirInstruccion(0x01);	// Clear the display
		LCD_EscribirInstruccion(0x80);	// Set display to first line

		/****************************/


}

void HabilitarLcd(void)
{
	delay();
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,5,13);//Enable=High
	delay();
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,13);//Enable=Low
	delay();
}



void LCD_EscribirString(char * string)
{
	int c=0;
	while (string[c]!='\0')
	{
		LCD_EscribirChar(string[c]);
		c++;
	}
}

void LCD_EscribirChar(char cmd)
{
		//Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,13);//Enable=Low
	    /*Habilito Rs=1 como dato*/
		Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,5,12);
		/*Configuro la palabra de 8 bits en nibles de 4*/

		uint8_t upperNibble = cmd >> 4;
		uint8_t lowerNibble = cmd & 0x0F;

		LCD_Data(upperNibble);

		HabilitarLcd();

		LCD_Data(lowerNibble);

		HabilitarLcd();

	/**************************/

}
void LCD_EscribirInstruccion(uint8_t cmd)
{

	    /*Habilito Rs=0 como instruccion*/
	    Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,12);
		/*Configuro la palabra de 8 bits en nibles de 4*/

		uint8_t upperNibble = cmd >> 4;
		uint8_t lowerNibble = cmd & 0x0F;

		LCD_Data(upperNibble);

		HabilitarLcd();

		LCD_Data(lowerNibble);

		HabilitarLcd();

	/**************************/

}

void delay(void)
{
	int i=0,x=0;
	for(i=0; i<20000; i++){ x++; }
}

//*******************************************
void LCD_Data(uint8_t ch)
{

	delay();
	Chip_GPIO_SetPinOutHigh(LPC_GPIO_PORT,5,14);
	/*****Escribir Dato************/
	if ((ch&0x08)==0){
		Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT,5,14);
	}
	Chip_GPIO_SetMaskedPortValue(LPC_GPIO_PORT,2,(ch<<4));

	delay();

}
//*******************************************




/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

