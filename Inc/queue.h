#ifndef _QUEUE_H_
#define _QUEUE_H_

#include "stm32f0xx_hal.h"
#include "utilities.h"

#define QUEUE_LENGHT 20

typedef struct _Queue {
    UsbMessage  buffer[QUEUE_LENGHT];
    volatile uint16_t can_tail;
    volatile uint16_t can_head;
    volatile uint16_t overflow;
} Queue;

int Queue_Pop(Queue * q, UsbMessage *msg);
void Queue_Push(Queue * q, UsbMessage *msg);
#endif

