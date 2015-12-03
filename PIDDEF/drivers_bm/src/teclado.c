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
#include "teclado.h"       /* <= own header */

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
/**************funciones teclado*************/

void InicializarTeclas(void){
	/*Funcion inicializa GPIO */
	Chip_GPIO_Init(LPC_GPIO_PORT);

	/**Configurar la System Control Unit (SCU), para indicarle las
	características eléctricas de cada pin empleado y remapearlos como puertos
	GPIO**/
		 Chip_SCU_PinMux(1,0,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* remapea P1_0 en GPIO0[4], TEC1
		 */
		 Chip_SCU_PinMux(1,1,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* remapea P1_1 en GPIO0[8], TEC2
		  */
		 Chip_SCU_PinMux(1,2,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* remapea P1_2 en GPIO0[9], TEC3
		 */
		 Chip_SCU_PinMux(1,6,MD_PUP|MD_EZI|MD_ZI,FUNC0); /* remapea P1_6 en GPIO1[9], TEC4

		 /********************************************************************/
		 /*Seleccionar el modo (entrada) de cada pin con la función "chip_GPIO_setDIr"*/
		 /*******************************************************************/
		 /*Defino entrada TEcla1(4), tecla2(8) y tecla(9)*/
		 Chip_GPIO_SetDir(LPC_GPIO_PORT,0,(((1<<4)|(1<<8))|(1<<9)),0);
		 /*Defino entrada tecla4(9)*/
		 Chip_GPIO_SetDir(LPC_GPIO_PORT,1,(1<<9),0);


}

int LeerTecla(int NroTecla){
	int puerto,devuelve=0,comparador=0,bit=0;
	int valor;

	/*Funcion devuelve el valor de entrada 0 ó 1 del puerto */
	switch(NroTecla){
		     case 1 :
		    	bit=4;
		    	comparador=0x0008;/*bit 4*/
		    	puerto=0;
		        break;
		     case 2 :
		    	bit=8;
		    	comparador=0x0080;/*bit 8*/
		    	puerto=0;
		    	break;
		     case 3 :
		    	bit=9;
		    	comparador=0x0100;
		        puerto=0;
		        break;
		     case 4 :
		    	bit=9;
		    	comparador=0x0100;
		        puerto=1;
		        break;
		     default : /* Optional */
		    	 break;

		 }

	/*Devuelve 32 bit con lectura del puerto*/
	valor=Chip_GPIO_GetPinState(LPC_GPIO_PORT,puerto,bit);
	devuelve=valor;
	return devuelve;

}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

