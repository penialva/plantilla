
/* Copyright 2018, Osvaldo Marcos Zanet (tato)
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

/** \file d_utils.h
 ** \brief General Function and Utilities
 ** Functions and utilities related with the port configuration and interruptions.
 ** This file is deprecated and it will be deleted in near future
 **
 **
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */

/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Stepper Driver
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *  tato		Osvaldo Marcos Zanet
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20181122 v0.0.1 first documented version version
 */

/*==================[inclusions]=============================================*/

#include "../inc/d_utils.h"


/*==================[internal data definition]===============================*/


static action interrupt_function[4];


/*==================[internal functions definition]==========================*/

void GPIOInit_T(digitalIO port[]){
	/** Mapping pins*/
	uint8_t cant=sizeof(port)/sizeof(digitalIO);
	for(int i =0;i<cant;i=i+1)
	{
		Chip_SCU_PinMuxSet(port[i].hwPort,port[i].hwPin,port[i].gpiomod);
		if(port[i].inout==INPUT){
			Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT ,port[i].gpioPort,port[i].gpioPin);

		}
		else
		{
			Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT ,port[i].gpioPort,port[i].gpioPin);
			Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT ,port[i].gpioPort,port[i].gpioPin);
		}
	}
}


void InterruptFunction(uint8_t t,action function){
	interrupt_function[t]=function;
}
void InterruptFunctionAll(action fun0,action fun1,action fun2,action fun3)
{
	interrupt_function[0]=fun0;
	interrupt_function[1]=fun1;
	interrupt_function[2]=fun2;
	interrupt_function[3]=fun3;

}
void InterruptFunctionSameToAll(action function)
{
	interrupt_function[0]=function;
	interrupt_function[1]=function;
	interrupt_function[2]=function;
	interrupt_function[3]=function;

}
void GPIO0_IRQHandler( void )
{
	if(interrupt_function[0]!=NULL)
		interrupt_function[0]();
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(0));
}

void GPIO1_IRQHandler( void )
{

	if(interrupt_function[1]!=NULL)
		interrupt_function[1]();

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT,PININTCH(1));
}

void GPIO2_IRQHandler( void )
{
	if(interrupt_function[2]!=NULL)
		interrupt_function[2]();
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT,PININTCH(2));
}

void GPIO3_IRQHandler( void )
{
	if(interrupt_function[3]!=NULL)
		interrupt_function[3]();
	Chip_PININT_ClearIntStatus (LPC_GPIO_PIN_INT,PININTCH(3));
}



/*==================[external data definition]===============================*/



/*==================[external functions definition]==========================*/


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

