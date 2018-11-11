/*
 *	FIFO.c
 *
 *	Created: 05-06-2017 16:48:55
 *  Author: Gustav Vinther-Jørgensen
 *
 *	This module implements a FIFO buffer structure witch makes it easy to create FIFO buffers
 *	and contains a number of functions to make accessing the buffers as easy as possible.
 *
 */

#include "FIFO.h"

// This initializes the FIFO structure with the given buffer and size. If the FIFO was already initialized it clears the buffer.
void fifo_init(fifo_t* f, MoveSteps* buff, int size)
{
	f->head = 0;
	f->tail = 0;
	f->size = size;
	f->data = buff;
}

// This reads one byte from the FIFO
// If the FIFO is empty it returns 0. Otherwise it returns 1.
char fifo_read(fifo_t* f, MoveSteps * data)
{
	if(f->tail != f->head) { // see if any data is available
		*data = f->data[f->tail]; // grab a byte from the buffer
		f->tail++; // increment the tail
		if(f->tail == f->size) { // check for wrap-around
			f->tail = 0;
		}
		return 0;
	}
	else {
		return 1;
	}
}

// This writes one byte to the FIFO
// If the head runs in to the tail 1 is returned
// Otherwise the function returns 0
char fifo_write(fifo_t* f, MoveSteps data)
{
	// first check to see if there is space in the buffer
	if((f->head + 1 == f->tail) || ((f->head + 1 == f->size) && (f->tail == 0))) {
		return 1; // no more room
		} else {
		f->data[f->head] = data;
		f->head++; // increment the head
		if(f->head == f->size) { // check for wrap-around
			f->head = 0;
		}
		return 0;
	}
}

//This function returns the number of data packets currently stored in the FIFO
int fifo_size(fifo_t* f){
	int size = 0;
	if (f->head > f->tail){
		size =  (f->head - f->tail);
	}
	else if (f->head < f->tail){
		size = f->size - f->tail + f->head;
	}
	return size;
}
