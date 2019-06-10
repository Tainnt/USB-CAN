#ifndef _QUEUE_H_
#define _QUEUE_H_

#include "stm32f0xx_hal.h"
#include "utilities.h"

#define QUEUE_LENGHT 25

typedef struct _Queue {
    Message  buffer[QUEUE_LENGHT];
    volatile uint16_t can_tail;
    volatile uint16_t can_head;
    volatile uint16_t overflow;
} Queue;

int Queue_Pop(Queue * q, Message *msg);
void Queue_Push(Queue * q, Message *msg);
#endif

