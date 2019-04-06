#include "Board.h"

Queue_t Q_buffer[UART_MAX_CH];

void Init_Q(Queue_t *Q){
	uint8_t count;
	for(count = 0; count<MAX; count++){
		Q->Ring_Buffer[count]= 0;
	}
	Q->head = 0;
	Q->tail = 0;
	Q->size = 0;
	Q->temp = 0;
}

void QueueCreate(uint8_t channel){
	Queue_t *p_uart;
	p_uart = &Q_buffer[channel];
	Init_Buffer(p_uart);
}

bool Init_Buffer(Queue_t *Q){
	bool result = true;
		uint8_t count;
	for(count = 0; count<MAX; count++){
		Q->Ring_Buffer[count]= 0;
	}
	Q->head = 0;
	Q->tail = 0;
	Q->size = MAX;
	Q->temp = 0;
	
	return result;
}

uint8_t Q_full(Queue_t *Q){
	if((Q->head+1)%MAX == Q->tail){
	   return TRUE;
	} return FALSE;
}
uint8_t Q_empty(Queue_t *Q){
	if(Q->head == Q->tail){
		return TRUE;
	} return FALSE;
}

uint8_t write_Q(Queue_t *Q, char data){
	if(Q_full(Q)){
		return FALSE;
	}
  Q->Ring_Buffer[Q->head] = data;
  Q->head = (Q->head+1) % MAX;
	return TRUE;
}

uint8_t read_Q(Queue_t *Q){
	if(Q_empty(Q)){
		return FALSE;
	}
		Q->temp = Q->Ring_Buffer[Q->tail];
		Q->tail = (Q->tail+1) % MAX;
		if(Q->tail >= MAX-1)return FALSE;

	return Q->temp;
	//return TRUE;
}

uint32_t QueueAvailable(Queue_t *Q)
{
  uint32_t length;

  length = (Q->size + Q->head - Q->tail) % Q->size;

  return length;
}

void QueueFlush(Queue_t *Q)
{
  Q->head  = 0;
  Q->tail = 0;
}
