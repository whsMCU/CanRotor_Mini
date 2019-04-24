#ifndef QUEUE_H_
#define QUEUE_H_

#define MAX 128

typedef struct queue {
  volatile uint8_t Ring_Buffer[MAX];
  volatile uint8_t head;
  volatile uint8_t tail;
  volatile uint8_t size;
  volatile uint8_t temp;
//uint8_t *p_buf;
  volatile int cnt;
} Queue_t;

void     Init_Q(Queue_t *Q);
void     QueueCreate(uint8_t channel);
bool     Init_Buffer(Queue_t *Q);
uint8_t  Q_full(Queue_t *Q);
uint8_t  Q_empty(Queue_t *Q);
uint8_t  write_Q(Queue_t *Q, char data);
uint8_t  read_Q(Queue_t *Q);
uint32_t QueueAvailable(Queue_t *Q);
void     QueueFlush(Queue_t *Q);

#endif
