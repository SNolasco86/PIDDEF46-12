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
#include "pwm.h"       /* <= own header */

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
void InicializarPWM(void){

	/*Funcion inicializa PWM */
	Chip_SCTPWM_Init(LPC_SCT);


}

 /*Configurar PinMUX, PinOUT, Frecuencia*/
void ConfigurarPWM(int Frec){

	/*Seteamos Frecuencia*/
	Chip_SCTPWM_SetRate(LPC_SCT,Frec);

	/*Seteamos pinMux*/
	/*****************/
	/******Leds*******/
	/*****************/
/* remapea P2_10 en CTOUT_2, LED1 y habilita el pull up
	Chip_SCU_PinMux(2,10,MD_PUP,FUNC1);
 remapea P2_11 en CTOUT_5, LED2 y habilita el pull up
	Chip_SCU_PinMux(2,11,MD_PUP,FUNC1);
 remapea P2_12 en CTOUT_4, LED3 y habilita el pull up
	Chip_SCU_PinMux(2,12,MD_PUP,FUNC1);*/
	/*****************/
	/******Pines*******/
	/*****************/
/* remapea P4_1 en CTOUT_1, GPIO2(1) y habilita el pull up*/
	Chip_SCU_PinMux(4,1,MD_PLN,FUNC1);
/* remapea P4_2 en CTOUT_0, GPIO2(2) y habilita el pull up*/
	Chip_SCU_PinMux(4,2,MD_PLN,FUNC1);
/* remapea P4_3 en CTOUT_3, GPIO2(3) y habilita el pull up*/
	Chip_SCU_PinMux(4,3,MD_PLN,FUNC1);
	/*****************/
	/******Leds*******/
	/*****************/

	/*Seteamos pinOUT de LED1
	Chip_SCTPWM_SetOutPin(LPC_SCT,10,2);
	Seteamos pinOUT de LED2
	Chip_SCTPWM_SetOutPin(LPC_SCT,11,5);
	Seteamos pinOUT de LED3
	Chip_SCTPWM_SetOutPin(LPC_SCT,12,4);*/
	/*****************/
	/******Pines*******/
	/*****************/
	/*Seteamos pinOUT de CTOUT_1. T_FIL1*/
	Chip_SCTPWM_SetOutPin(LPC_SCT,1,1);
	/*Seteamos pinOUT de CTOUT_0, T_FIL2*/
	Chip_SCTPWM_SetOutPin(LPC_SCT,2,0);
	/*Seteamos pinOUT de CTOUT_3, T_FIL3*/
	Chip_SCTPWM_SetOutPin(LPC_SCT,3,3);

		//LPC_SCT->CONFIG        |= (1 << 17)|(1 << 18);               // split timers, auto limit
	/******************************/
	/*****************************/
	/*****04/11/2015**************/
	  //  LPC_SCT->CTRL_L        |= (1 << 4);                // configure SCT1 as BIDIR
	  //  LPC_SCT->MATCH[5].U    = 180;                 // match on (half) PWM period

/*******************************/
	/*****************************/
	    /***********************/
	   /* LPC_SCT->MATCH[1].L     = 0;                     // match on duty cycle 1
	    LPC_SCT->MATCHREL[1].L  = 0;
	    LPC_SCT->MATCH[2].L     = 180;                     // match on duty cycle 2
	    LPC_SCT->MATCHREL[2].L  = 180;*/

	   /* LPC_SCT->EVENT[0].STATE = 0xFFFFFFFF;              // event 0 happens in all states
	    LPC_SCT->EVENT[0].CTRL  = (2 << 10) | (2 << 12);   // IN_0 falling edge only condition*/

	   /* LPC_SCT->EVENT[1].STATE = 0xFFFFFFFF;              // event 1 happens in all states
	    LPC_SCT->EVENT[1].CTRL  = (1 << 10) | (2 << 12);   // IN_0 rising edge only condition*/

	    /************************************/

	   /* LPC_SCT->EVENT[1].STATE = 0xFFFFFFFF;              // event 2 happens in all states
	    LPC_SCT->EVENT[1].CTRL  = (1 << 0) | (1 << 12);    // match 1 (DC1) only condition

	    LPC_SCT->EVENT[2].STATE = 0xFFFFFFFF;              // event 3 happens in all states
	    LPC_SCT->EVENT[2].CTRL  = (2 << 0) | (1 << 12);    // match 2 (DC2) only condition*/

	   /* LPC_SCT->OUT[0].SET     = (1 << 0) | (1 << 2);     // event 0 and 2 set OUT0 (blue LED)
	    LPC_SCT->OUT[0].CLR     = (1 << 2);                // event 2 clears OUT0 (blue LED)

	    LPC_SCT->OUT[1].SET     = (1 << 1);                // event 3 sets OUT1 (red LED)
	    LPC_SCT->OUT[1].CLR     = (1 << 0) | (1 << 1);     // event 0 and 3 clear OUT1 (red LED)*/
/**********************************/
	/*******04/11/2015**********/

	   /* LPC_SCT->RES           |= 0x0000000F; */             // toggle OUT0 and OUT1 on conflict
/***********************************/
	    //LPC_SCT->OUTPUT        |= 1;                       // default set OUT0 and and clear OUT1*/

	   /*****************/
	    /*int ix = (int) index;
	    	pSCT->EVENT[ix].CTRL = index | (1 << 12);
	    	pSCT->EVENT[ix].STATE = 1;

	    	pSCT->OUT[pin].SET = 1;
	    	pSCT->OUT[pin].CLR = 1 << ix;

	    	pSCT->OUT[pin].SET = 1 | (1 << ix);
	    	pSCT->OUT[pin].CLR = 1 << ix;
	    */
	    /*********************/


    LPC_SCT->OUT[2].SET     = (1 << 0) ;     // event 0 and 2 set OUT0 (blue LED)
	LPC_SCT->OUT[2].CLR     = (1 << 2);                // event 2 clears OUT0 (blue LED)

    LPC_SCT->OUT[3].SET     = (1 << 1);                // event 3 sets OUT1 (red LED)

    LPC_SCT->OUT[3].CLR     = (1 << 3) | (1 << 0);     // event 0 and 3 clear OUT1 (red LED)

	//LPC_SCT->OUT[3].CLR     = (1 << 3);     // event 0 and 3 clear OUT1 (red LED)



 }
void AnchoPulsoPWM(int dutyccycle,int index){

	/*Funcio */
	  if((dutyccycle < 50)&&(index==3)){
		  //Ver esto funcion mayor 50%
		 LPC_SCT->OUT[3].CLR     = (1 << 3);     // event 0 and 3 clear OUT1 (red LED)
	  }
	  if((dutyccycle >= 50)&&(index==3)) {
		  LPC_SCT->OUT[3].CLR    = (1 << 3) | (1 << 0);     // event 0 and 3 clear OUT1 (red LED)
	  }




	/*Dutycycle en porcentaje*/
	 Chip_SCTPWM_SetDutyCycle(LPC_SCT,index,Chip_SCTPWM_PercentageToTicks(LPC_SCT, dutyccycle));
	 /***********04/11/2015************/

	 /*Para que nadie modifique el desfasaje 180*/
	 /*LPC_SCT->MATCH[0].L     = 180;                 // match on (half) PWM period
	 LPC_SCT->MATCHREL[0].L = 180;*/
}

void PulsoInicialPWM(int dutycycle, int index){
	Chip_SCT_SetMatchCount(LPC_SCT,(CHIP_SCT_MATCH_REG_T)index,Chip_SCTPWM_PercentageToTicks(LPC_SCT, dutycycle));
}

/*Comienza a funcionar el PWM*/
void ArranquePWM(void){
	Chip_SCTPWM_Start(LPC_SCT);
}

/*Para el funcionamiento PWM*/
void ParadaPWM(void){
	Chip_SCTPWM_Stop(LPC_SCT);
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

