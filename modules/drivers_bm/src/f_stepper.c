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

/** \file f_stepper.h
 ** \brief Stepper Drivers Functions and Ports assignment
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


#include "f_stepper.h"

/*==================[macros and definitions]=================================*/


/*==================[internal data declaration]==============================*/



static stepper* stepper_list[MAXQSTEPPERS];


const stepper_port_config stepper_0_STP=
{
	STEPPER_0_STEP_MUX_GROUP,
	STEPPER_0_STEP_MUX_PIN,
	STEPPER_0_STEP_GPIO_PORT,
	STEPPER_0_STEP_GPIO_PIN,
	STEPPER_0_STEP_FUNC
};
const stepper_port_config stepper_0_DIR=
{
	STEPPER_0_DIR_MUX_GROUP,
	STEPPER_0_DIR_MUX_PIN,
	STEPPER_0_DIR_GPIO_PORT,
	STEPPER_0_DIR_GPIO_PIN,
	STEPPER_0_DIR_FUNC
};
const stepper_port_config stepper_0_SLP=
{
	STEPPER_0_SLP_MUX_GROUP,
	STEPPER_0_SLP_MUX_PIN,
	STEPPER_0_SLP_GPIO_PORT,
	STEPPER_0_SLP_GPIO_PIN,
	STEPPER_0_SLP_FUNC
};
const stepper_port_config stepper_0_M0=
{
	STEPPER_0_M0_MUX_GROUP,
	STEPPER_0_M0_MUX_PIN,
	STEPPER_0_M0_GPIO_PORT,
	STEPPER_0_M0_GPIO_PIN,
	STEPPER_0_M0_FUNC
};
const stepper_port_config stepper_0_M1=
{
	STEPPER_0_M1_MUX_GROUP,
	STEPPER_0_M1_MUX_PIN,
	STEPPER_0_M1_GPIO_PORT,
	STEPPER_0_M1_GPIO_PIN,
	STEPPER_0_M1_FUNC
};
const stepper_port_config stepper_0_M2=
{
	STEPPER_0_M2_MUX_GROUP,
	STEPPER_0_M2_MUX_PIN,
	STEPPER_0_M2_GPIO_PORT,
	STEPPER_0_M2_GPIO_PIN,
	STEPPER_0_M2_FUNC
};
static stepper def_stepper={
		STOPPED,0,CLOCKWISE,FULL_STEP,SLEEPING,
		&stepper_0_STP,&stepper_0_DIR,&stepper_0_SLP,
		&stepper_0_M0,&stepper_0_M1,&stepper_0_M2
} ;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/


/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/




stepper* StepperAdd(stepper* st)
{
	uint8_t sz=0;

	while(stepper_list[sz]!=NULL)
	{
		sz=sz+1;
	}
	if(sz<MAXQSTEPPERS)
	{

		if(st==NULL)
		{
			if(sz==0)
			{
//				stepper newstepper;
//				newstepper.sleep=SLEEPING;
//				newstepper.dir=CLOCKWISE;
//				newstepper.microstep=FULL_STEP;
//				newstepper.speed=STOPPED;					// speed in RPM
//				newstepper.STP_port=&stepper_0_STP;
//				newstepper.DIR_port=&stepper_0_DIR;
//				newstepper.SLP_port=&stepper_0_SLP;
//				newstepper.M0_port=&stepper_0_M0;
//				newstepper.M1_port=&stepper_0_M1;
//				newstepper.M2_port=&stepper_0_M2;
				stepper_list[sz]=&def_stepper;
				StepperInit(stepper_list[sz]);
			}
		}
		else
		{
			stepper_list[sz]=st;
			StepperInit(stepper_list[sz]);
		}
	}
	return stepper_list[sz];
}

void StepperSetSpeed(stepper* st,int8_t speed)
{

	if (rpm2sps(speed)<=MAX_SPS*2)
	{
		if(speed<0)
		{
			speed=0-speed;
			StepperDirCounterClockwise(st);
		}

		st->speed=speed;
	}
}


void StepperDirClockwise(stepper* st)
{
	StepperSetDir(st,CLOCKWISE);
}

void StepperDirCounterClockwise(stepper* st)
{
	StepperSetDir(st,COUNTERCLOCKWISE);
}

void StepperHold(stepper* st)
{
	StepperWakeUp(st);
	StepperSetSpeed(st,STOPPED);
}


void StepperRoutine(void)
{

	for(uint8_t i=0;i<MAXQSTEPPERS;i=i+1)
	{
		if(stepper_list[i]->sleep==AWAKE&&stepper_list[i]->speed!=0)
		{
			stepper_list[i]->count=(stepper_list[i]->count+1)%(MAX_SPS*2/(stepper_list[i]->microstep*rpm2sps(stepper_list[i]->speed)));
			if(stepper_list[i]->count==0)
			{
				StepperStepToggle(stepper_list[i]);
			}
		}
	}
}






/*==================[external functions definition]==========================*/


/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

