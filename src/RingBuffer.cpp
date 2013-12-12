/*
 * RingBuffer.cpp
 *
 *  Created on: Jan 2, 2012
 *      Author: tero
 */

#include "RingBuffer.h"
#include <stdlib.h>
#include "stm32f2xx.h"

RingBuffer::RingBuffer( int bytes )
{
	allocate(bytes);
}

RingBuffer::RingBuffer()
{
	buf=NULL;
}

//same as constructor with a param
void RingBuffer::allocate( int bytes )
{
	buf=(u8*)malloc(bytes);
	tail=head=0;
	size=bytes;
	bytesAvail=0;
}

RingBuffer::~RingBuffer()
{
	free(buf);
}

void RingBuffer::clear()
{
	tail=head=bytesAvail=0;
}

bool RingBuffer::put( u8 data )
{
        u32 hd=head;
        hd++;
        if(hd>=size) hd=0;
        if(bytesAvail>=size)return false;//full
        buf[head]=data;
        head=hd;
        bytesAvail++;
        return true;
}

void RingBuffer::put( u8 data, bool &bufferFull )
{
	bool success=put(data);
	if(success==false)
		bufferFull=true;
	else
		bufferFull=false;
}

//get byte from FIFO
u8 RingBuffer::get()
{
	bool trash;
	return get(trash);
}


//get byte from FIFO
//sets error to true if buffer is empty
u8 RingBuffer::get( bool &empty )
{
		u8 ret;
        u32 tl=tail;
        if(bytesAvail<=0)
        {
        	//nodata
        	empty=true;
        	return 0;
        }
        bytesAvail--;
        ret=buf[tl];
        tl++;
        if(tl>=size)tl=0;
        tail=tl;
        return ret;
}

//get oldest byte without removing it from FIFO
u8 RingBuffer::peek( bool &empty )
{
    if(bytesAvail<=0)
    {
    	//nodata
    	empty=true;
    	return 0;
    }

    return buf[tail];
}

//get number of bytes
u32 RingBuffer::bytesAvailable() const
{
	return bytesAvail;
}

u32 RingBuffer::bytesFree() const
{
	return (size-bytesAvail);
}

//for testing
bool ringBufferTest()
{
	u32 size=33;
	RingBuffer b;
	b.allocate(size);
	u32 i;
	bool fail=false;

	if(b.bytesFree()!=size)fail=true;
	if(b.bytesAvailable()!=0)fail=true;

	for(i=0;i<size;i++)
		b.put(i,fail);

	if(b.bytesFree()!=0)fail=true;
	if(b.bytesAvailable()!=size)fail=true;

	for(i=0;i<size;i++)
		if(b.get(fail)!=i) fail=true;

	if(b.bytesFree()!=size)fail=true;
	if(b.bytesAvailable()!=0)fail=true;

	for(i=0;i<size;i++)
		b.put(i,fail);

	if(b.bytesFree()!=0)fail=true;
	if(b.bytesAvailable()!=size)fail=true;

	for(i=0;i<size;i++)
		if(b.get(fail)!=i) fail=true;

	b.get(fail);//should fail

	return fail;
}


