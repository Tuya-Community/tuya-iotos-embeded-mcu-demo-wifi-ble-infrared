#include "sys.h"
#include "RCC.h"
#include "delay.h"
#include "IO.h"
#include "TIM.h"
#include "USART.h"
void System_Init(void)
{
	SystemClock_Config();//HSI16/4 -> PLL*8/2->16M	
	delay_init(16);
	TIM_Init();
	Configure_USART_MAIN(USART_MAIN_BOUND);
	Configure_USART_VICE(USART_VICE_BOUND);	
	IO_Init();
}


void System_Task(void)
{
	if(F_TASK_InfraRed)//红外调制发出该数据
	{
		F_TASK_InfraRed--;
		TASK_InfraRed();
	}
	if(F_TASK_USART_VICE)//上传该数据
	{
		F_TASK_USART_VICE--;
		TASK_USART_VICE();
	}	
}

