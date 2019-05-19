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

#ifndef MODULES_LPC4337_M4_DRIVERS_BM_INC_F_STEPPER_H_
#define MODULES_LPC4337_M4_DRIVERS_BM_INC_F_STEPPER_H_

/** \brief Header for Stepper Functions
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
#include "d_stepper.h"
#include "d_utils.h"


/*==================[macros and definitions]=================================*/

#define STEPPER_0_STEP_MUX_GROUP	6	// pin 29
#define STEPPER_0_STEP_MUX_PIN 		1
#define STEPPER_0_STEP_GPIO_PORT 	3
#define STEPPER_0_STEP_GPIO_PIN 	0
#define STEPPER_0_STEP_FUNC 		0

#define STEPPER_0_DIR_MUX_GROUP		6	// pin 32
#define STEPPER_0_DIR_MUX_PIN 		4
#define STEPPER_0_DIR_GPIO_PORT 	3
#define STEPPER_0_DIR_GPIO_PIN 		3
#define STEPPER_0_DIR_FUNC 			0

#define STEPPER_0_SLP_MUX_GROUP		6	// pin 31
#define STEPPER_0_SLP_MUX_PIN 		5
#define STEPPER_0_SLP_GPIO_PORT 	3
#define STEPPER_0_SLP_GPIO_PIN 		4
#define STEPPER_0_SLP_FUNC 			0

#define STEPPER_0_M0_MUX_GROUP		6	// pin 34
#define STEPPER_0_M0_MUX_PIN 		7
#define STEPPER_0_M0_GPIO_PORT 		5
#define STEPPER_0_M0_GPIO_PIN 		15
#define STEPPER_0_M0_FUNC 			4

#define STEPPER_0_M1_MUX_GROUP		6	// pin 33
#define STEPPER_0_M1_MUX_PIN 		8
#define STEPPER_0_M1_GPIO_PORT 		5
#define STEPPER_0_M1_GPIO_PIN 		16
#define STEPPER_0_M1_FUNC 			4

#define STEPPER_0_M2_MUX_GROUP		6	// pin 36
#define STEPPER_0_M2_MUX_PIN 		9
#define STEPPER_0_M2_GPIO_PORT 		3
#define STEPPER_0_M2_GPIO_PIN 		5
#define STEPPER_0_M2_FUNC 			0

#define MAX_SPS				6000		// max step per second
#define STOPPED				0
#define DEFAULT_SPEED		60			// in RPM
#define SPR					400			// step per revolution

#define MAXQSTEPPERS		5


#define rpm2sps(rpm)((rpm*SPR)/60)

/*==================[external functions declaration]=========================*/



/*! \brief Add Stepper.
 * Add a stepper to the stepper list and return the pointer to the added stepper.
 * If the list is empty and the input is NULL, add the default stepper configuration and return its pointer.
 * If the list is full, the function does not do anything, and return NULL;
 * \param[in] st The pointer to the new stepper.
 * \return Return the pointer to the new stepper. If the stepper list is full return NULL
 */
stepper* StepperAdd(stepper* st);

/*! \brief Set Speed Function.
 * Change the stepper speed.
 * \param[in] speed The speed to set. Must be in RPM unit.
 * \param[in] st The pointer to the stepper.
 * \return none
 */
void StepperSetSpeed(stepper* st,int8_t speed);

/*! \brief Set Direction to Clockwise.
 * Change the direction of the stepper to clockwise.
 * \param[in] st The pointer to the stepper.
 * \return none
 */
void StepperDirClockwise(stepper* st);

/*! \brief Set Direction to Counter-Clockwise.
 * Change the direction of the stepper to counter-clockwise.
 * \param[in] st The pointer to the stepper.
 * \return none
 */
void StepperDirCounterClockwise(stepper* st);

/*! \brief Holding Position
 * Hold the stepper awake, immobile, with the coils energized.
 * \param[in] st The pointer to the stepper.
 * \return none
 */
void StepperHold(stepper* st);

/*!\brief Stepper routinary function
 *
 * This function at MAX_SPS*2 times per second toggled the STP output
 *
 * \param[in] st The pointer to the stepper.
 *
 */
void StepperRoutine(void);

/*! \brief RPM to Step per second
 * Convert the speed in RPM to step per second. It doesn't consider the microstepping,
 * so it must be multiplied later.
 * \param[in] rpm The speed to set. Must be in RPM unit.
 * \return the frequency in step per second, needed to reach that speed
 */


#endif /* MODULES_LPC4337_M4_DRIVERS_BM_INC_F_STEPPER_H_ */
