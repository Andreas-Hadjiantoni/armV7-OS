#include "queue.h"
#include <stdbool.h>
#include "../hilevel.h"

void initialiseQueue(queue *q, size_t itemSize) {
  q->back = &(q->queue[length]);
  q->front = &(q->queue[length - itemSize]);
  q->s = itemSize;
}

bool full(queue *q) {
  return q->back - q->queue < q->s;
}

bool empty(queue *q) {
  return q->back > q->front;
}

size_t size(queue *q) {
  return q->s;
}

char* peek_front(queue *q) {
  return q->front;
}

char* peek_back(queue *q) {
  return q->back;
}

void moveItems(queue *q) {
  if (empty(q))
    return;

  for (char *element = q->front - 1; element != q->back - 1; element--)
    *(element + q->s) = *element;

  q->back += q->s;
}

//returns true on success and false otherwise
bool push(queue *q, void* item) {
  if (full(q))
    return false;

  q->back -= q->s;

  memcpy(q->back, item, q->s);

  return true;
}

void pop(queue *q, void* returnAddress) {
  if (empty(q)) return;

  memcpy(returnAddress, q->front, q->s);

  moveItems(q);
}
