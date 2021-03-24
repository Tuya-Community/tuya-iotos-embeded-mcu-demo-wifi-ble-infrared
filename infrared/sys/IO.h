#ifndef __IO_H
#define __IO_H 		
#include "MY_ST_config.h"


//InfraRedCon   PB9 红外供电开关
#define InfraRedCon_OUT {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<18);GPIOB->MODER|=1<<18;}  
#define InfraRedCon_SET GPIOB->ODR|=1<<9
#define InfraRedCon_RESET GPIOB->ODR&=~(1<<9)



//InfraRedCon   PC6 红外PWM输出
#define InfraRedTX_OUT {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<12);GPIOC->MODER|=1<<12;}  
#define InfraRedTX_SET GPIOC->ODR|=1<<6
#define InfraRedTX_RESET GPIOC->ODR&=~(1<<6)






extern uint8_t InfraReddata;
extern uint8_t InfraRed_step;
extern uint8_t InfraRed_busy;
extern uint8_t F_TASK_InfraRed;

void TASK_InfraRed(void);
void IO_Init(void);
	
#endif

