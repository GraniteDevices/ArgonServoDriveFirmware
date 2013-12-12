/*
 * Serial.h
 *
 *  Created on: Oct 21, 2011
 *      Author: tero
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#define RXBUFSIZE 256
#define TXBUFSIZE 256

#include "types.h"
#include "stm32f2xx_dma.h"

class System;

/**
\brief Serial communication driver.
Provides USART & DMA handling for serial interfaces to DSC and RS485. 
Limitations:
-TX buffer must be sent manually. If more than TXBUFSIZE bytes are put in buffer before send, the rest of data is lost
-Receive does not detect RX buffer overflow. If RX buffer is not emptied by user in time, data will be lost

*/
class Serial {
public:
	enum Port { RS485, DSC };

	Serial( Port port, System *parent );
	~Serial();

	void sendBuffer();
	//put byte in TX buffer, must send buffer with sendBuffer before buffer is full
	void putByte( u8 byte );
	//returns byte from RX buffer. sets emtpy to true if no data to be received
	void putBuffer( u8 *buf, int len );
	//read buffer, return bytes read
	int getBuffer( u8 *buf, int len );
	u8 getByte( bool &empty );
	bool transmitComplete();
	//return true if success
	bool armReceiveCompleteEvent( bool on, u16 rxlen );
	bool setBaudRate( s32 bps );//return false on failure
private:
	Port thisPort;
	int txBufPos;
	u16 rxBufPos;
	u16 rxPacketLen;//expected packet size for interrupt generation

	unsigned char *rxBuffer;
	unsigned char *txBuffer;
	DMA_InitTypeDef DMA_InitStructure;
};

void RS485_EnableTransmit(bool on);
//called from ISR
void RS485_TransmitCompleteEvent();

#endif /* SERIAL_H_ */
