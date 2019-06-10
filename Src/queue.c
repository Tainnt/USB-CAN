#include "queue.h"

int Queue_Pop(Queue *q, UsbMessage *msg) {

    uint16_t tail;

    if(q->can_tail != q->can_head) {
        tail = q->can_tail + 1;
        if(tail >= QUEUE_LENGHT) tail = 0;
        q->can_tail = tail;
				*msg = q->buffer[tail];
        return 1;
    }
    return 0;
}

void Queue_Push(Queue *q, UsbMessage *msg) {

    uint16_t head;

    head = (q->can_head + 1);
    if(head >= QUEUE_LENGHT) head = 0;
		q->buffer[head] = *msg;
    q->can_head = head;

    if ( head == q->can_tail) {
        // skip message
        q->overflow = 1;
    }

}
