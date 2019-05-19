/* Copyright 2018, Osvaldo Marcos Zanet
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

#ifndef INC_D_STEPPER_H_
#define INC_D_STEPPER_H_

/** \brief Header for Stepper Drivers
 **
 ** The header for the d_stepper.c file
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Baremetal Bare Metal example header file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * TATO			Osvaldo Marcos Zanet
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20181122 v0.0.1 first documented version
 */

/*==================[inclusions]=============================================*/
//#include "d_utils.h"
#include "stdint.h"

/*==================[macros and definitions]=================================*/



#define STP_UP				1
#define STP_DOWN			0

#define SLEEPING			0
#define AWAKE				1



#define FULL_STEP			1			// correspond to m0=0 m1=0 m2=0
#define HALF_STEP			2			// correspond to m0=1 m1=0 m2=0
#define QUAR_STEP			4			// correspond to m0=0 m1=1 m2=0
#define OCTA_STEP			8			// correspond to m0=1 m1=1 m2=0
#define OI16_STEP			16			// correspond to m0=0 m1=0 m2=1
#define OI32_STEP			32			// correspond to m0=1 m1=0 m2=1

#define CLOCKWISE				0
#define COUNTERCLOCKWISE		1	// speed negativa es antihoraria (esto no se sabra hasta no probarla)


 /*==================[typedef]================================================*/

 typedef struct
 {
	 uint8_t hwPort ;
	 uint8_t hwPin ;
	 uint8_t gpioPort ;
	 uint8_t gpioPin ;
	 uint8_t gpioFunc;
 } stepper_port_config;

 typedef struct {				/*!< Stepper State Struct*/
	 uint8_t speed;
	 uint8_t count;				/*Counter in 1/(MAXSPS * 2) frequency*/
	 uint8_t dir;
	 uint8_t microstep;
	 uint8_t sleep;
	 stepper_port_config *STP_port;
	 stepper_port_config *DIR_port;
	 stepper_port_config *SLP_port;
	 stepper_port_config *M0_port;
	 stepper_port_config *M1_port;
	 stepper_port_config *M2_port;
 } stepper;




/*==================[external functions declaration]=========================*/

/*! \brief Step Up
 * Change the STP output to high value.
 * \param[in] st The pointer to the stepper.
 */
void StepperStepUp(stepper* st);

/*! \brief Step Down
 * Change the STP output to low value.
 * \param[in] st The pointer to the stepper.
 */
void StepperStepDown(stepper* st);

/*! \brief Step Toggle
 * Change the STP output value.
 * \param[in] st The pointer to the stepper.
 */
void StepperStepToggle(stepper* st);

/*! \brief Set Direction
 * Change the DIR output to a "dir" value.
 *
 * \param[in] st The pointer to the stepper.
 * \param[in] dir Direction, it can be 0 or 1.
 */
void StepperSetDir(stepper* st,uint8_t dir);


/*! \brief Stepper Aslepp
 * Change the SLP output to SLEEP (0). The stepper axis doesn't move.
 * The main difference with speed 0 is that the stepper coils are de-energized.
 * \param[in] st The pointer to the stepper.
 */
void StepperSleep(stepper* st);

/*! \brief Stepper Wake Up
 * Change the SLP output to AWAKE (1).
 * \param[in] st The pointer to the stepper.
 */
void StepperWakeUp(stepper* st);



/*! \brief Initialize the stepper
 * Initialize the ports designed for the stepper.
 * \param[in] st The pointer to the stepper.
 */
void StepperInit(stepper* st);

/*! \brief Set Microstepping Configuration
 * Change the M0, M1, M2 outputs to set the fraction of step 1/ms
 *
 * \param[in] st The pointer to the stepper.
 * \param[in] ms MicroStep, The fraction of the step. IE for half step ms=2, for
 * one eighth step ms=8.
 */
void StepperSetMicrostepping(stepper* st,uint8_t ms);
#endif /* INC_D_STEPPER_H_ */
