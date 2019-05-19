#include "f_utils.h"



extern uint8_t _sStack;





void Reset (void)
{
	Chip_RGU_TriggerReset(RGU_CORE_RST);
}
inline uint32_t stackUsed(void)
{
	return &_sStack-(uint8_t*) __get_MSP();
}

void MostrarStack(void){
	char word[20];
	uint32_t stackAddr = stackUsed();

	UARTWrite("\n\r\n\r");
	uint2char(word,stackAddr);
	UARTWrite(word);
	UARTWrite("\n\r\n\r");
}





void uint2char(char* pal,uint32_t inte)
{

	if(inte==0)
	{
		pal[0]='0';
		pal[1]='\0';
	}
	else
	{
		int j=inte,n=inte;
		int c=0;
		while(j!=0)
		{
			j=j/10;
			c=c+1;
		}
		while(c>j)
		{
			j=j+1;
			pal[c-j]=(char)(n%10)+'0';
			n=n/10;
		}

		pal[c]='\0';
	}

}

