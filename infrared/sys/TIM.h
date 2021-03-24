#ifndef __TIM_H
#define __TIM_H 		
#include "MY_ST_config.h"

void TIM14_PWM_Stop(void);
void TIM14_PWM_Start(void);
void TIM7_Start(void);
void TIM7_Stop(void);
void TIM3_Continue(void);

void TIM_Init(void);

#endif

