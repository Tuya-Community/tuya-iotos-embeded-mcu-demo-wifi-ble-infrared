#include "IO.h"
#include "delay.h"
#include "TIM.h"
#include "USART.h"

uint8_t InfraReddata;
uint8_t InfraRed_step=0;
uint8_t InfraRed_busy=0;
uint8_t F_TASK_InfraRed=0;
uint16_t InfraRed_count=0;

void InfraRed_IO_Init(void)
{
	InfraRedCon_OUT;
	InfraRedCon_RESET;
}

void InfraRed_Send(uint8_t *data)
{
  USART_VICE->CR1 &=~(1<<2);//停止接收
  InfraReddata = *data;
  InfraRed_step =0;
  TIM7_Start();
	InfraRed_busy=1;
}


void TASK_InfraRed(void)
{
	if(InfraRed_count<q_mian_buf[Q_Main.front].length)
	{
		F_TASK_InfraRed++;
		if(InfraRed_busy==0)
		{
			USART_VICE->CR1 &=~(1<<2);//禁止接收
			InfraReddata = q_mian_buf[Q_Main.front].data[InfraRed_count];
			InfraRed_step =0;
			InfraRed_busy=1;
			TIM7_Start();//开始调制	
			InfraRed_count++;
			if(InfraRed_count>q_mian_buf[Q_Main.front].length-1)//数据发完
			{
				F_TASK_InfraRed--;
				InfraRed_count=0;
				memset(q_mian_buf[Q_Main.front].data,0,q_side_buf[Q_Main.front].length);
				q_mian_buf[Q_Main.front].length=0;
				front_inc(&Q_Main);					
			}
		}
	}
}

void IO_Init(void)
{
	InfraRed_IO_Init();
}
