#include "TIM.h"
#include "IO.h"
#include "USART.h"

void TIM3_Init(uint16_t arr,uint16_t psc)//VICE串口超时
{
	RCC->APBENR1 |= 1<<1;
	TIM3->ARR = arr;
	TIM3->PSC = psc;
	TIM3->DIER |= 1<<0;//使能更新中断
	TIM3->CR1 |= 1<<0;//使能计数器
	NVIC_SetPriority(TIM3_IRQn, 1); 
	NVIC_EnableIRQ(TIM3_IRQn);
}
void TIM3_Start(void)
{
	TIM3->CR1 |=1<<0;//使能计数器
}
void TIM3_Stop(void)
{
	TIM3->CR1 &=~(1<<0);	//禁止计数器
	TIM3->CNT =0;//计数清零
}
void TIM3_Continue(void)
{
	TIM3->CR1 |=1<<0;//使能计数器
	TIM3->CNT =0;//计数清零
}
void TIM3_IRQHandler(void)
{
	if ((TIM3->SR & 1) == 1) /* Check ARR match */ 
	{	
		TIM3->SR &= ~(1<<0);//清除中断标志位
		TIM3_Stop();
		F_TASK_USART_VICE++;		
	}
}


void ConfigureTIM14_CH1_AsPWM_EdgeAligned(void)//T14_CH1 PWM PF0 AF2
{
  RCC->APBENR2 |= 1<<15;
  RCC->IOPENR |= 1<<5;
	GPIOF->MODER &=~(3<<0);
  GPIOF->MODER |=  2<<0; //复用功能
	GPIOF->AFR[0]&=0XFFFFFFF0;
  GPIOF->AFR[0] |= 0x02 << 0;
  
  TIM14->PSC = 0; //16M/(0+1)=16M, step=0.0625us
  TIM14->ARR = 420;//0.0625*（420+1）=26.3125us
  TIM14->CCR1 = 140; //CH_1（140+1）/(420+1)=33.5%
  TIM14->CCMR1 |=6<<4|1<<3;//CH_1 110模式,预装载使能（默认为输出模式）
  TIM14->CCER |= 1<<0;  //CH_1开启输出PWM
  TIM14->CR1 |= 1<<0; //使能计数器(边沿对齐 递增)
  TIM14->EGR |= 1<<0; //更新中断
	TIM14->BDTR |=1<<14;//自动输出使能AOE
}
void TIM14_PWM_Stop(void)
{
	TIM14->CCER &=~(1<<0);//关闭输出PWM
	TIM14->CR1 &=~(1<<0);	//禁止计数器
	TIM14->CNT =0;//计数清零
}
void TIM14_PWM_Start(void)
{
	TIM14->CCER |=1<<0;//开启输出PWM
	TIM14->CR1 |=1<<0;//使能计数器
}
void TIM14_PWM_Set(uint16_t CCR)
{
	TIM14->CCR1 = CCR;
}
void TIM7_Init(uint16_t arr,uint16_t psc)
{
	RCC->APBENR1 |= 1<<5;
	TIM7->ARR = arr;
	TIM7->PSC = psc;
	TIM7->DIER |= 1<<0;//使能更新中断
	TIM7->CR1 |= 1<<0;//使能计数器
	NVIC_SetPriority(TIM7_LPTIM2_IRQn, 1); 
	NVIC_EnableIRQ(TIM7_LPTIM2_IRQn);
}
void TIM7_Start(void)
{
	TIM7->CR1 |=1<<0;//使能计数器
}
void TIM7_Stop(void)
{
	TIM7->CR1 &=~(1<<0);	//禁止计数器
	TIM7->CNT =0;//计数清零
}

void TIM7_LPTIM2_IRQHandler(void)
{
	if ((TIM7->SR & 1) == 1) /* Check ARR match */ 
	{	
		TIM14_PWM_Stop();
		if(InfraRed_step==0)
		{
			TIM14_PWM_Start();//起始位，低电平
			InfraRed_step++;			
		}
		else if((InfraRed_step>0)&&(InfraRed_step<9))
		{
      InfraRed_step++; 
      if(InfraReddata &0x01)
      {
       TIM14_PWM_Stop(); 
      }
      else
      {
       TIM14_PWM_Start(); 
      }
      InfraReddata >>=1;
		}
		else if(InfraRed_step==9)
		{
			TIM14_PWM_Stop();//停止位
			InfraRed_step++;			
		}
		else
		{
			TIM14_PWM_Stop();//停止PWM
			InfraRed_step=0;	
      TIM7_Stop();
			InfraRed_busy=0;
		}
	
		
		TIM7->SR &= ~(1<<0);//清除中断标志位
	}
}

void TIM_Init(void)
{	
	TIM3_Init(10,15999);//串口10ms接接受超时
	TIM3_Stop();	
	TIM7_Init(1000000/USART_VICE_BOUND,15);//T=833us
	TIM7_Stop();
	ConfigureTIM14_CH1_AsPWM_EdgeAligned();
	TIM14_PWM_Stop();
}

