/*
 *	FIFO.h
 *
 *	Created: 05-06-2017 16:48:55
 *  Author: Gustav Vinther-Jørgensen
 *
 *	This module implements a FIFO buffer structure witch makes it easy to create FIFO buffers
 *	and contains a number of functions to make accessing the buffers as easy as possible.
 *
 */


#ifndef FIFO_H_
#define FIFO_H_

#include "MotorControl.h"
/*
	A type to be used when addressing a FIFO buffer.
*/

typedef struct	{
	move_t * data;
	int head;
	int tail;
	int size;
} fifo_t;


// This initializes the FIFO structure with the given buffer and size. If the FIFO was already initialized it clears the buffer.
void fifo_init(fifo_t* f, move_t* buff, int size);

// This reads one byte from the FIFO
// If the FIFO is empty it returns 0. Otherwise it returns 1.
extern char fifo_read(fifo_t * f, move_t * data);

// This writes one byte to the FIFO
// If the head runs in to the tail 0 is returned
// Otherwise the function returns 1
extern char fifo_write(fifo_t * f, move_t data);

//This function returns the number of data packets currently stored in the FIFO
extern int fifo_size(fifo_t* f);


#endif /* FIFO_H_ */
