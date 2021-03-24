#ifndef __USART_H
#define __USART_H 		
#include "MY_ST_config.h"
#include "stdio.h"
#include "string.h"




typedef struct {
	uint8_t front;//队首指针
	uint8_t rear;//队尾指针
	uint8_t size;//队列容量
}Queue;

void front_inc(Queue* SQ);
void rear_inc(Queue* SQ);


#define USART_MAIN  USART1
#define USART_MAIN_BUF_LEN 40
#define USART_MAIN_BOUND 115200
struct quene_buf_type1
{
	uint8_t tab;
	uint16_t length;
	uint8_t data[USART_MAIN_BUF_LEN];
};	
#define quene_main_buf_total 4
extern struct quene_buf_type1 q_mian_buf[quene_main_buf_total];
extern Queue Q_Main;
extern uint8_t F_TASK_USART_MAIN;
void Configure_USART_MAIN(uint32_t bound);//TX PC4, RX PC5 USART1
void TASK_USART_MAIN(void);



#define USART_VICE  USART2
#define USART_VICE_BUF_LEN 40
#define USART_VICE_BOUND 1200

struct quene_buf_type2
{
	uint8_t tab;
	uint16_t length;
	uint8_t data[USART_VICE_BUF_LEN];
};	
#define quene_side_buf_total 4
extern struct quene_buf_type2 q_side_buf[quene_side_buf_total];
extern Queue Q_Side;
extern uint8_t F_TASK_USART_VICE;
void Configure_USART_VICE(uint32_t bound);//TX PD8, RX PD9 USART3
void TASK_USART_VICE(void);

#endif

