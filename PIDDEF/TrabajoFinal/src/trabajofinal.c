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
#include "trabajofinal.h"       /* <= own header */



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
/**************************/
/******Variables Globales******/

int LED1=14,LED2=11,LED3=12,LED_R=0,LED_G=1,LED_B=2;
int valor_Analogico,contador;
int ValorTecla;
float escalador;
/***********************/
extern void InterrupRitISR(void){
	/*Contador*/
	contador++;
	/*Pasaron ? 100mseg(10Hz)*/
	if(contador>100){
	/*Toma valor analogico cada 10Hz*/
	valor_Analogico= EntradaADC();
	/*Multipplica la salida*/

	valor_Analogico=((int)(escalador*valor_Analogico));

	/*imprimo la salida del DAC*/
	SalidaDAC(valor_Analogico);

	/*enciende LED azul cada 100mseg*/
	InvertirLed(LED_B);
	contador=0;
	}

    /*Enciende LED3 cada 1mseg*/
	InvertirLed(LED_R);
	/*anti_Saturador*/
	/*if(valor_Analogico>1018){
	valor_Analogico=1018;
	}*/
	/*Reseteo contador*/
	Chip_RIT_ClearInt(LPC_RITIMER);
}

int main(void)
{


   /* perform the needed initialization here */
   /*Inicializa los Leds*/
   InicializarLeds();
   /*Inicaliza Teclado*/
   InicializarTeclas();
   /*Inicaliza Timer*/
   InicializarTimer();
   /*Inicializa convDAC*/
   InicializarDAC();
   /*Inicializa conADC*/
   InicializarADC();
   /*Configurar tiempo 1mseg*/
   ConfigurarTiempo(1);
   /*Habilita interrupcion*/
   SetInterrup();
/***************Funciones UART*********************/
   /*void UART_Init();
   unsigned char UART_Read();
   void UART_Send(char dato);*/
/***************************************************/

   while(1) {


	   ValorTecla=LeerTecla(1);
	  	   if (ValorTecla==1)
	  		       ApagarLed(LED1);

	  	   else{
	  		       PrenderLed(LED1);
	  		       escalador=1.5;
	  	   }
	  	   /*Tecla 2*/
	  	   ValorTecla=LeerTecla(2);
	  	   if (ValorTecla==1)
	  	  		   ApagarLed(LED2);
	  	   else{
	  	  		   PrenderLed(LED2);
	  	  		   escalador=0.5;
	  	   }
	  	   /*Tecla 3*/
	  	    ValorTecla=LeerTecla(3);
	  	  	 if (ValorTecla==1)
	  	  	  		   ApagarLed(LED3);
	  	   else{
	  	  	  		   PrenderLed(LED3);
	  	  	  		escalador=0;
	  	   }
	  	  	 /*Tecla 4*/
	  	  	   ValorTecla=LeerTecla(4);
	  	   if (ValorTecla==1)
	  	  		  		   ApagarLed(LED_G);
	  	    else{
	  	  		  		   PrenderLed(LED_G);
	  	  		  		   escalador=1.0;
	  	    }


   }

   return 0;
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

