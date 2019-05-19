#ifndef F_UTILS_H
#define F_UTILS_H




#include "d_utils.h"





inline uint32_t stackUsed(void)
__attribute__ (( always_inline ));


typedef void *(*accion)(void);


void Reset(void);
void MostrarStack(void);
void uint2char(char* pal,uint32_t inte);




#endif
