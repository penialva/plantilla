#ifndef INC_D_UTILS_H_
#define INC_D_UTILS_H_

#include "chip.h"
#include "stdint.h"



#define OUTPUT 1
#define INPUT 0


/*==================[typedef]================================================*/
typedef void (*action)( void );

typedef struct
{
uint8_t hwPort ;
uint8_t hwPin ;
uint8_t gpioPort ;
uint8_t gpioPin ;
uint16_t gpiomod ;
uint8_t inout;
} digitalIO ;

void GPIOInit_T(digitalIO port[]);


void InterruptFunction(uint8_t t,action function);
void InterruptFunctionAll(action fun0,action fun1,action fun2,action fun3);
void InterruptFunctionSameToAll(action function);
void GPIO0_IRQHandler( void );
void GPIO1_IRQHandler( void );
void GPIO2_IRQHandler( void );
void GPIO3_IRQHandler( void );

#endif
