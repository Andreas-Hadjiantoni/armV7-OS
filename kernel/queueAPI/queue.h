#ifndef __QUEUE_H
#define __QUEUE_H
#define length 50

#include <stdlib.h>
#include <stdbool.h>

typedef struct queue {
  char queue[length];
  char *back; //index where last item was pushed
  char *front;
  size_t s; //item size
} queue;

void initialiseQueue(queue *q, size_t itemSize);

bool full(queue *q);

bool empty(queue *q);

size_t size(queue *q);

char* peek_front(queue *q);

char* peek_back(queue *q);

//returns true on success and false otherwise
bool push(queue *q, void* item);

//returns a pcb_t object on success and exits otherwise
void pop(queue *q, void* returnAddress);

#endif
