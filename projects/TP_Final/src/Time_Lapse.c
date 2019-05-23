/* Copyright 2017, Esteban Volentini - Facet UNT, Fi UNER
 * Copyright 2014, 2015 Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * All rights reserved.
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
 */

/** @file Time_Lapse.c
 **
 ** @brief Slider Motorizado para Time Lapse
 **
 ** Control de un sistema motorizado para la realización de videos "Time-Lapse"
 ** y "Live Motion". El sistema utiliza un LCD color de 2.2'' (ILI9341) y las
 ** cuatro teclas de la EDU-CIAA como interfaz con el usuario para la
 ** configuración del mismo. El control del motor paso-a-paso que genera el
 ** desplazamiento de la cámara se realiza utilizando el driver XXXX.
 ** 
 ** | RV | YYYY.MM.DD | Autores            | Descripción de los cambios        |
 ** |----|------------|--------------------|-----------------------------------|
 ** |  1 | 2019.05.16 | Peñalva - Cerrudo  | Version inicial del archivo       |
 ** 
 */

/* === Inclusiones de cabeceras ============================================ */
#include "../inc/main.h"

#include <stdint.h>
#include <string.h>
#include "gpio.h"
#include "led.h"
#include "switch.h"
#include "task.h"
#include "event_groups.h"
#include "semphr.h"
#include "ili9341.h"
#include "f_stepper.h"
#include "timer.h"

/* === Definiciones y Macros =============================================== */

#define EVENTO_COMPLETO   (1 << 0)

#define SPI_1 1			/*!< EDU-CIAA SPI port */

#define FOTO 0			/*!< Modo fotografía (Time-Lapse) */
#define VIDEO 1			/*!< Modo Video */
#define ACTIVADO 1		/*!< Estado activado */
#define DESACTIVADO 0	/*!< Estado desactivado */

#define TPO_OBT 200		/*!< Tpo (en ms) de obturación */
#define AVANCE 1		/*!< Dirección de motor para avance de carro */
#define RETROCESO 0		/*!< Dirección de motor para retroceso de carro */
#define MAX_VEL 6000	/*!< Velocidad máxima en pasos por segundo */
#define SPMM 160		/*!< Pasos por mm
						* srev*fm       200*32
						* ------- = d = ------
						*  p*Nt          2*20
						* srev is the number of steps per revolution for the motor
						* fm is the microstepping factor (1, 2, 4, 8 etc.)
						* p is the pitch (e.g. 2mm)
						* Nt is the number of teeth on the pulley attached to the motor shaft.
						*/

#define SCREEN_MARG 8	/*!< Margenes de pantalla */
#define MENU_MARG_X 20	/*!< Margen del Menú */
#define MENU_MARG_Y 63	/*!< Margen del Menú */
#define MENU_DIST_Y 3	/*!< Separación netre ítems del menú */

/* === Declaraciones de tipos de datos internos ============================ */

/** @brief Estructura de datos de configuración
 **
 ** Estructura que contiene los datos de configuración
 ** de la toma a realizarse.
 */
typedef struct {
   uint16_t distancia;  /** < Distancia total a recorrer en mm */
   uint32_t tiempo;     /** < Tiempo total de la sesión en ms */
   uint8_t fotos;    	/** < Cantidad de fotos a tomar */
   uint8_t modo;		/** < Foto o Video */
   uint8_t estado;		/** < Activado o Desactivado */
   uint8_t m_index;		/** < Posición actua del cursor en el menú de configuración */
} sesion_t;

/** @brief Estructura de ítem de menú
 **
 ** Estructura que contiene los parámetros asociados a cada ítem
 ** del menú de configuración.
 */
typedef struct {
   uint16_t value;     	/** < Valor actual del parámetro */
   uint8_t min_value;	/** < Valor mínimo del parámetro */
   uint8_t max_value;	/** < Valor máximo del parámetro */
   uint8_t * str;		/** < Etiqueta del parámetro */
} menu_t;

/* === Declaraciones de funciones internas ================================= */

/** @brief Tarea que escanea el teclado
 **
 ** Esta tarea lee el estado actual de las teclas de la EDU-CIAA y lo compara
 ** con el de la ultima activacion para detectar los cambios en las teclas y
 ** al detectar la pulsación de una tecla permitir tanto la navegación por el
 ** menú, como modificar los parámetros de conffiguración del sistema.
 ** Esta tarea se desactiva una vez iniciada una toma, permaneciendo en este
 ** estado hasta finalizarse la misma.
 */
void Teclado(void * parametros);

/** @brief Tarea que controla la graficación en el Display
 **
 ** Esta tarea se encarga de graficar en el Display el menú de configuración,
 ** los valores de los parámetros configurados y, una vez iniciada una toma,
 ** el estado de avance de la misma.
 */
void Pantalla(void * parametros);

/** @brief Tarea que controla el movimiento de la cámara y la toma de fotos
 **
 ** Esta tarea se activa al inicarse una toma (si la misma fue configurada en
 ** modo "Foto". La misma se encarga de reiniciar la posición de la cámara
 ** (llevarla al origen)y luego controlar tanto los movimientos de la misma
 ** como la captura de fotos, según la configuración realizada.
 */
void Foto(void * parametros);

/** @brief Tarea que controla el movimiento de la cámara durante filmación
 **
 ** Esta tarea se activa al inicarse una toma (si la misma fue configurada en
 ** modo "Video". La misma se encarga de reiniciar la posición de la cámara
 ** (llevarla al origen)y luego controlar los movimientos de la misma, según
 ** la configuración realizada.
 */
void Video(void * parametros);

/** @brief Función que comanda el movimiento del carro
 **
 **	@param[in]	stp Cantidad de pasos del motor a realizar
 **	@param[in]	tpo Tiempo total (en us) en que se deben realizar la cantidad de pasos
 **	@param[in]	dir Sentido de movivimiento (avance o retroceso)
 ** @return		NONE
 */
void MoverCarro(uint32_t stp, uint32_t tpo, uint8_t dir);

/** @brief Función que comanda la realización de los pasos del motor
 **
 ** Función que se ejecuta en la interrupción del Timer0 y en cada invocación
 ** comanda la realización de un paso del motor.
 */
void Pasos(void);

/* === Definiciones de variables internas ================================== */

/** @brief Menú de configuración
 *
 */
menu_t menu[]={
	{ FOTO, FOTO, VIDEO, "Modo     "},
	{ 200, 1, 250, "Tiempo   "},					/** < Tiempo en segundos */
	{ 40, 1, 100, "Fotos    "},						/** < Cantidad de Fotos */
	{ 80, 1, 85, "Distancia"},						/** < Distancia en cm */
	{ DESACTIVADO, DESACTIVADO, ACTIVADO, "INICIO"}
};

/** @brief Coniguración por default
 *
 */
sesion_t s = {
	FOTO,
	10,
	10,
	10,
	DESACTIVADO,
	0
};

/** @brief Configuración de pines para control de motor (GPIO0)
 *
 */
stepper_port_config step_0 = {
	STEPPER_0_STEP_MUX_GROUP,
	STEPPER_0_STEP_MUX_PIN,
	STEPPER_0_STEP_GPIO_PORT,
	STEPPER_0_STEP_GPIO_PIN,
	STEPPER_0_STEP_FUNC
};

/** @brief Configuración de pines para control de motor (GPIO1)
 *
 */
stepper_port_config dir_0 = {
	STEPPER_0_DIR_MUX_GROUP,
	STEPPER_0_DIR_MUX_PIN,
	STEPPER_0_DIR_GPIO_PORT,
	STEPPER_0_DIR_GPIO_PIN,
	STEPPER_0_DIR_FUNC
};

/** @brief Declaración de motor
 *
 */
stepper motor = {
	DEFAULT_SPEED,
	0,
	AVANCE,
	OI32_STEP,
	SLEEPING,
	&step_0,
	&dir_0,
	NULL,
	NULL,
	NULL,
	NULL
};

/** @brief Declaración de timer
 *
 */
timer_config temporizador = {
	TIMER_C,
	0,
	Pasos
};

/** @brief Declaración de GPIO para fin de carrera y obturador
 *
 */
gpioConf_t fin_de_carrera = {
		GPIO_5,
		INPUT,
		PULLUP
};
gpioConf_t obturador = {
		GPIO_6,
		OUTPUT,
		NONE_RES
};

uint8_t origen = FALSE;
uint8_t fotos_tomadas;
uint32_t pasos, pasos_realizados;

/** @brief Descriptor de las tareas de Teclado, Foto y Video */
TaskHandle_t xFotoHandle;
TaskHandle_t xVideoHandle;
TaskHandle_t xTecladoHandle;

/** @brief Descriptor del grupo de eventos */
EventGroupHandle_t eventos;

/* === Definiciones de variables externas ================================== */

/* === Definiciones de funciones internas ================================== */

void Pantalla(void * parametros) {
	uint16_t background[5];
	uint8_t i;
	/* Graficar Menú */
	ILI9341Fill(ILI9341_DARKGREY);
	ILI9341DrawRectangle(SCREEN_MARG, SCREEN_MARG, ILI9341_WIDTH - SCREEN_MARG,
			ILI9341_HEIGHT - SCREEN_MARG, ILI9341_BLACK);
	ILI9341DrawFilledRectangle(SCREEN_MARG, SCREEN_MARG,
			ILI9341_WIDTH - SCREEN_MARG,
			SCREEN_MARG + font_11x18.FontHeight + 8, ILI9341_BLACK);
	ILI9341DrawString(25, SCREEN_MARG + 5, "TIME LAPSE SLIDER", &font_11x18,
			ILI9341_LIGHTGREY, ILI9341_BLACK);
	ILI9341DrawString(MENU_MARG_X + 30,
			SCREEN_MARG + 10 + font_11x18.FontHeight + 2 * MENU_DIST_Y,
			"CONFIGURACION", &font_11x18, ILI9341_BLACK, ILI9341_DARKGREY);
	ILI9341DrawLine(MENU_MARG_X,
			SCREEN_MARG + 10 + 2 * font_11x18.FontHeight + 3 * MENU_DIST_Y,
			ILI9341_WIDTH - MENU_MARG_X,
			SCREEN_MARG + 10 + 2 * font_11x18.FontHeight + 3 * MENU_DIST_Y,
			ILI9341_LIGHTGREY);
	for (i = 0; i < sizeof(menu) / sizeof(menu_t); i++) {
			ILI9341DrawLine(MENU_MARG_X,
					MENU_MARG_Y + (2 * i + 2) * MENU_DIST_Y
					+ (i + 1) * font_11x18.FontHeight,
					ILI9341_WIDTH - MENU_MARG_X,
					MENU_MARG_Y + (2 * i + 2) * MENU_DIST_Y
					+ (i + 1) * font_11x18.FontHeight, ILI9341_LIGHTGREY);
	}

	while (1) {
		if (s.estado == DESACTIVADO) {
			/* Actualizar Menú */
			for (i = 0; i < sizeof(menu) / sizeof(menu_t); i++) {
				background[i] = ILI9341_DARKGREY;
			}
			background[s.m_index] = ILI9341_BLUE2;
			for (i = 0; i < sizeof(menu) / sizeof(menu_t) - 1; i++) {
				ILI9341DrawFilledRectangle(
						MENU_MARG_X + 9 * font_11x18.FontWidth,
						MENU_MARG_Y + (2 * i + 1) * MENU_DIST_Y
						+ i * font_11x18.FontHeight,
						ILI9341_WIDTH - font_11x18.FontWidth * 5 - MENU_MARG_X,
						MENU_MARG_Y + (2 * i + 1) * MENU_DIST_Y
						+ (i + 1) * font_11x18.FontHeight - 1,
						background[i]);
				ILI9341DrawString(MENU_MARG_X,
						MENU_MARG_Y + (2 * i + 1) * MENU_DIST_Y
						+ i * font_11x18.FontHeight, menu[i].str,
						&font_11x18, ILI9341_BLACK, background[i]);
				if (i == 0) {
					if (menu[0].value == FOTO) {
						ILI9341DrawString(
								ILI9341_WIDTH - font_11x18.FontWidth * 5- MENU_MARG_X,
								MENU_MARG_Y + MENU_DIST_Y, "Foto ", &font_11x18,
								ILI9341_BLACK, background[0]);
					} else {
						ILI9341DrawString(
								ILI9341_WIDTH - font_11x18.FontWidth * 5- MENU_MARG_X,
								MENU_MARG_Y + MENU_DIST_Y, "Video", &font_11x18,
								ILI9341_BLACK, background[0]);
					}
				} else {
					ILI9341DrawInt(
							ILI9341_WIDTH - font_11x18.FontWidth * 5- MENU_MARG_X,
							MENU_MARG_Y + (2 * i + 1) * MENU_DIST_Y
							+ i * font_11x18.FontHeight, menu[i].value,
							3, &font_11x18, ILI9341_BLACK, background[i]);
				}
			}
			ILI9341DrawString(
					ILI9341_WIDTH - font_11x18.FontWidth * 2 - MENU_MARG_X,
					MENU_MARG_Y + 3 * MENU_DIST_Y + 1 * font_11x18.FontHeight,
					"s ", &font_11x18, ILI9341_BLACK, background[1]);
			ILI9341DrawString(
					ILI9341_WIDTH - font_11x18.FontWidth * 2 - MENU_MARG_X,
					MENU_MARG_Y + 5 * MENU_DIST_Y + 2 * font_11x18.FontHeight,
					"  ", &font_11x18, ILI9341_BLACK, background[2]);
			ILI9341DrawString(
					ILI9341_WIDTH - font_11x18.FontWidth * 2 - MENU_MARG_X,
					MENU_MARG_Y + 7 * MENU_DIST_Y + 3 * font_11x18.FontHeight,
					"cm", &font_11x18, ILI9341_BLACK, background[3]);
			ILI9341DrawString(90,
					MENU_MARG_Y + (2 * 4 + 1) * MENU_DIST_Y
					+ 4 * font_11x18.FontHeight, menu[4].str,
					&font_11x18, ILI9341_BLACK, background[4]);
			/* Borrar pantalla */
			ILI9341DrawFilledRectangle(MENU_MARG_X,
					MENU_MARG_Y + (2 * 6) * MENU_DIST_Y
					+ 6 * font_11x18.FontHeight,
					ILI9341_WIDTH - MENU_MARG_X,
					ILI9341_HEIGHT - SCREEN_MARG - MENU_DIST_Y,
					ILI9341_DARKGREY);
		} else {
			/* Progress bar */
			ILI9341DrawRectangle(MENU_MARG_X,
					MENU_MARG_Y + (2 * 6) * MENU_DIST_Y
					+ 6 * font_11x18.FontHeight,
					ILI9341_WIDTH - MENU_MARG_X,
					MENU_MARG_Y + (2 * 6 + 2) * MENU_DIST_Y
							+ 7 * font_11x18.FontHeight, ILI9341_LIGHTGREY);
			/* Origen */
			if (origen){
				ILI9341DrawString(MENU_MARG_X,
						MENU_MARG_Y + (2 * 6 + 2) * MENU_DIST_Y
						+ 8 * font_11x18.FontHeight, "Retorno origen...",
						&font_11x18, ILI9341_BLACK, ILI9341_DARKGREY);
			} else {
				/* Progress bar */
				if(s.modo == FOTO){
					ILI9341DrawFilledRectangle(MENU_MARG_X + MENU_DIST_Y,
							MENU_MARG_Y + (2 * 6 + 1) * MENU_DIST_Y
							+ 6 * font_11x18.FontHeight,
							MENU_MARG_X + MENU_DIST_Y
							+ ((ILI9341_WIDTH - 2 * MENU_MARG_X
							- 2 * MENU_DIST_Y) * pasos_realizados) / (pasos * s.fotos),
							MENU_MARG_Y + (2 * 6 + 1) * MENU_DIST_Y
							+ 7 * font_11x18.FontHeight, ILI9341_BLUE2);

				} else{
					ILI9341DrawFilledRectangle(MENU_MARG_X + MENU_DIST_Y,
							MENU_MARG_Y + (2 * 6 + 1) * MENU_DIST_Y
							+ 6 * font_11x18.FontHeight,
							MENU_MARG_X + MENU_DIST_Y
							+ ((ILI9341_WIDTH - 2 * MENU_MARG_X
							- 2 * MENU_DIST_Y) * pasos_realizados) / pasos,
							MENU_MARG_Y + (2 * 6 + 1) * MENU_DIST_Y
							+ 7 * font_11x18.FontHeight, ILI9341_BLUE2);
				}
				/* Info */
				ILI9341DrawString(MENU_MARG_X,
						MENU_MARG_Y + (2 * 6 + 2) * MENU_DIST_Y
						+ 8 * font_11x18.FontHeight, "Distancia: ",
						&font_11x18, ILI9341_BLACK, ILI9341_DARKGREY);
				ILI9341DrawInt(MENU_MARG_X + 11 * font_11x18.FontWidth,
						MENU_MARG_Y + (2 * 6 + 2) * MENU_DIST_Y
						+ 8 * font_11x18.FontHeight, pasos_realizados / (10 * SPMM), 3,
						&font_11x18, ILI9341_BLACK, ILI9341_DARKGREY);
				ILI9341DrawString(MENU_MARG_X + 14 * font_11x18.FontWidth,
						MENU_MARG_Y + (2 * 6 + 2) * MENU_DIST_Y
						+ 8 * font_11x18.FontHeight, "cm",
						&font_11x18, ILI9341_BLACK, ILI9341_DARKGREY);
				if(s.modo == FOTO){
					ILI9341DrawString(MENU_MARG_X,
							MENU_MARG_Y + (2 * 7 + 2) * MENU_DIST_Y
							+ 9 * font_11x18.FontHeight, "Fotos: ",
							&font_11x18, ILI9341_BLACK, ILI9341_DARKGREY);
					ILI9341DrawInt(MENU_MARG_X + 7 * font_11x18.FontWidth,
							MENU_MARG_Y + (2 * 7 + 2) * MENU_DIST_Y
							+ 9 * font_11x18.FontHeight, fotos_tomadas, 3,
							&font_11x18, ILI9341_BLACK, ILI9341_DARKGREY);
				}
			}
		}
		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}

void Teclado(void * parametros) {
   uint8_t tecla;
   uint8_t anterior = 0;

   while(1) {
	  tecla = Read_Switches();
      if (tecla != anterior) {
         switch(tecla) {
            case TEC1:
                 if(s.m_index > 0){
                	 s.m_index -= 1;
                 }
               break;
            case TEC2:
            	if(s.m_index < 4){
            		s.m_index += 1;
            	}
               break;
            case TEC3:
            	if(menu[s.m_index].value > menu[s.m_index].min_value){
            		menu[s.m_index].value -= 1;
            	}
               break;
            case TEC4:
            	if(menu[s.m_index].value < menu[s.m_index].max_value){
            		menu[s.m_index].value += 1;
            	}
               break;
         }
         anterior = tecla;
      }
      s.distancia = menu[3].value * 10;	// para pasar a mm
      s.fotos = menu[2].value;
      s.tiempo = menu[1].value * 1000; // para pasar a ms
      s.modo = menu[0].value;
      s.estado = menu[4].value;
      if((s.modo == FOTO) && (s.estado == ACTIVADO) ){
    	  vTaskResume( xFotoHandle );
    	  vTaskSuspend(xTecladoHandle);
      }
      if((s.modo == VIDEO) && (s.estado == ACTIVADO) ){
    	  vTaskResume( xVideoHandle );
    	  vTaskSuspend(xTecladoHandle);
      }
      Led_Toggle(GREEN_LED);
      vTaskDelay(150 / portTICK_PERIOD_MS);
   }
}

void Foto(void * parametros) {
	uint8_t i;
	static int32_t pasos_totales, pasos_parciales, tiempo_espera;

	while (1) {
		vTaskSuspend(xFotoHandle);
		fotos_tomadas = 0;
		pasos_totales = s.distancia * SPMM; 			// Cantidad de pasos totales
		pasos_parciales = pasos_totales / s.fotos; 		// Cantidad de pasos parciales por foto
		tiempo_espera = s.tiempo / s.fotos - 1000 * (pasos_parciales / MAX_VEL) - TPO_OBT;	// Tiempo espera entre movimientos en ms
		/* Retorno a Origen */
		origen = TRUE;
		while(GPIORead(fin_de_carrera.pin)){
			/* retrocede el carro 5mm a la velocidad máxima y vuelve a preguntar si llegó al origen */
			MoverCarro(5 * SPMM, 1000000 / MAX_VEL, RETROCESO);
			xEventGroupWaitBits(eventos, EVENTO_COMPLETO, TRUE, FALSE, portMAX_DELAY);
		}
		origen = FALSE;
		pasos_realizados = 0;
		/* Movimiento Programado */
		for (i = 0; i < s.fotos; i++){
			MoverCarro(pasos_parciales, 1000000 / MAX_VEL, AVANCE);
			xEventGroupWaitBits(eventos, EVENTO_COMPLETO, TRUE, FALSE, portMAX_DELAY);
			/* Tomar foto */
			vTaskDelay(tiempo_espera / portTICK_PERIOD_MS);	// espera entre movimientos
			GPIOSetLow(obturador.pin);
			vTaskDelay(TPO_OBT / portTICK_PERIOD_MS); 	// retardo para el obturador
			GPIOSetHigh(obturador.pin);
			fotos_tomadas++;
		}
		menu[4].value = DESACTIVADO;
		vTaskResume(xTecladoHandle);
	}
}

void Video(void * parametros) {
	static uint32_t pasos_totales;

	while(1) {
		vTaskSuspend(xVideoHandle);
		pasos_totales = s.distancia * SPMM; 		// Cantidad de pasos totales
		/* Retorno a Origen */
		origen = TRUE;
		while(GPIORead(fin_de_carrera.pin)){
			/* retrocede el carro 5mm a la velocidad máxima y vuelve a preguntar si llegó al origen */
			MoverCarro(5 * SPMM, 1000000 / MAX_SPS, RETROCESO);
			xEventGroupWaitBits(eventos, EVENTO_COMPLETO, TRUE, FALSE, portMAX_DELAY);
		}
		origen = FALSE;
		pasos_realizados = 0;
		/* Movimiento Programado */
		MoverCarro(pasos_totales, s.tiempo * 1000 / pasos_totales, AVANCE);
		xEventGroupWaitBits(eventos, EVENTO_COMPLETO, TRUE, FALSE, portMAX_DELAY);
		menu[4].value = DESACTIVADO;
		vTaskResume(xTecladoHandle);
	}
}

void MoverCarro(uint32_t stp, uint32_t tpo_stp, uint8_t dir){
	StepperSetDir(&motor, dir);
	temporizador.period = tpo_stp; // tiempo en us entre pasos
	pasos = stp;
	TimerInit(&temporizador);
	TimerStart(temporizador.timer);
}

void Pasos(void){
	static uint32_t i = 0;
	if (i < pasos){
		StepperStepUp(&motor);
		//StepperStepToggle(&motor);
		Led_Toggle(YELLOW_LED);
		TimerStart(temporizador.timer);
		i++;
		pasos_realizados++;
		StepperStepDown(&motor);
	} else {
		i = 0;
		TimerStop(temporizador.timer);
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xEventGroupSetBitsFromISR(eventos, EVENTO_COMPLETO, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}
/* === Definiciones de funciones externas ================================== */

/** @brief Función principal del programa
 **
 ** @returns 0 La función nunca debería termina
 **
 ** \remarks En un sistema embebido la función main() nunca debe terminar.
 **          El valor de retorno 0 es para evitar un error en el compilador.
 */
int main(void) {

   /* Inicializaciones y configuraciones de dispositivos */
   Init_Leds();
   Init_Switches();
   ILI9341Init(SPI_1, GPIO_4, GPIO_2, GPIO_3);
   ILI9341Rotate(ILI9341_Portrait_2);
   StepperInit(&motor);
   GPIOInit(fin_de_carrera);
   GPIOInit(obturador);
   GPIOSetHigh(obturador.pin);	// Obturador inicia en alto

   /* Creación del grupo de eventos */
   eventos = xEventGroupCreate();

   /* Creación de las tareas */
	if (eventos != NULL) {
		xTaskCreate(Teclado, "Teclado", configMINIMAL_STACK_SIZE,
				NULL, tskIDLE_PRIORITY + 2, &xTecladoHandle);

		xTaskCreate(Pantalla, "Pantalla", configMINIMAL_STACK_SIZE*10,
				NULL, tskIDLE_PRIORITY + 2, NULL);

		xTaskCreate(Foto, "Foto", configMINIMAL_STACK_SIZE,
				NULL, tskIDLE_PRIORITY + 1, &xFotoHandle);

		xTaskCreate(Video, "Video", configMINIMAL_STACK_SIZE,
				NULL, tskIDLE_PRIORITY + 1, &xVideoHandle);

		/* Arranque del sistema operativo */
		vTaskStartScheduler();
	}
   
   /* vTaskStartScheduler solo retorna si se detiene el sistema operativo */
   while(1);

   /* El valor de retorno es solo para evitar errores en el compilador*/
   return 0;
}
/* === Ciere de documentacion ============================================== */

/** @} Final de la definición del modulo para doxygen */

