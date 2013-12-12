/*
 * RingBuffer.h
 *
 *  Created on: Jan 2, 2012
 *      Author: tero
 */

#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include "types.h"

class RingBuffer
{
public:
	RingBuffer( int bytes );
	RingBuffer();
	virtual ~RingBuffer();

	//same as constructor with param
	void allocate( int bytes );

	//insert byte in FIFO
	//return false if buffer full
	bool put(u8 byte);
	//sets buffer full =true if not enough space
	void put( u8 data, bool &bufferFull );

	//get byte from FIFO
	//sets error to true if buffer is empty
	u8 get( bool &empty );
	u8 get();

	//get oldest byte without removing it from FIFO
	u8 peek( bool &empty );

	//empty all data
	void clear();

	//number of bytes to read in FIFO
	u32 bytesAvailable() const;
	//number of bytes of free space in FIFO
	u32 bytesFree() const;
private:

	u8 *buf;
	u32 tail, head, size, bytesAvail;
};

#endif /* RINGBUFFER_H_ */
