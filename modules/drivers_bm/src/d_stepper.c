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

/** \brief Stepper Drivers Functions and Ports assignment
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


#include "../inc/d_stepper.h"
#include "chip.h"

/*==================[macros and definitions]=================================*/


/*==================[internal data declaration]==============================*/


/*==================[internal data definition]===============================*/
/*==================[internal functions definition]==========================*/

void StepperInit(stepper* st)
{
//	GPIOInit(st->stp_conf);
//
//	Chip_SCU_PinMuxSet(port[i].hwPort,port[i].hwPin,port[i].gpiomod);
//	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT ,port[i].gpioPort,port[i].gpioPin);
//	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT ,port[i].gpioPort,port[i].gpioPin);
	Chip_GPIO_Init(LPC_GPIO_PORT);
	StepperPortInit(st->STP_port);
	StepperPortInit(st->DIR_port);
	StepperPortInit(st->SLP_port);
	StepperPortInit(st->M0_port);
	StepperPortInit(st->M1_port);
	StepperPortInit(st->M2_port);


	StepperSetMicrostepping(st,FULL_STEP);		// by default use full step for microstepping
}

void StepperPortInit(stepper_port_config* st_conf)
{
//	GPIOInit(st->stp_conf);

	Chip_SCU_PinMuxSet(st_conf->hwPort,st_conf->hwPin,SCU_MODE_INACT|st_conf->gpioFunc);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT ,st_conf->gpioPort,st_conf->gpioPin);
	Chip_GPIO_SetPinOutLow(LPC_GPIO_PORT ,st_conf->gpioPort,st_conf->gpioPin);
}


void StepperStepUp(stepper* st)
{
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, st->STP_port->gpioPort, st->STP_port->gpioPin, STP_UP);
}


void StepperStepDown(stepper* st)
{
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, st->STP_port->gpioPort, st->STP_port->gpioPin, STP_DOWN);
}


void StepperStepToggle(stepper* st)
{
	Chip_GPIO_SetPinToggle(LPC_GPIO_PORT, st->STP_port->gpioPort, st->STP_port->gpioPin);
}


void StepperSetDir(stepper* st,uint8_t dir)
{
	st->dir=dir;
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, st->DIR_port->gpioPort, st->DIR_port->gpioPin, dir);
}


void StepperSleep(stepper* st)
{
	st->sleep=SLEEPING;
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, st->SLP_port->gpioPort, st->SLP_port->gpioPin, SLEEPING);
}


void StepperWakeUp(stepper* st)
{
	st->sleep=AWAKE;
	Chip_GPIO_SetPinState(LPC_GPIO_PORT, st->SLP_port->gpioPort, st->SLP_port->gpioPin, AWAKE);
}

void StepperSetMicrostepping(stepper* st,uint8_t mi_st)
{
	st->microstep=mi_st;
	uint8_t mic=0;
	while(mi_st!=FULL_STEP)
	{
		mic=mic+1;
		mi_st=mi_st>>1;
	}
//
//	for(uint8_t i=0;mic!=0;i=i+1,mic>>1)
//	{
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, st->M0_port->gpioPort, st->M0_port->gpioPin, mic&1);
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, st->M1_port->gpioPort, st->M1_port->gpioPin, mic&10);
		Chip_GPIO_SetPinState(LPC_GPIO_PORT, st->M2_port->gpioPort, st->M2_port->gpioPin, mic&100);
//			}
}


/*==================[external data definition]===============================*/



/*==================[external functions definition]==========================*/


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

