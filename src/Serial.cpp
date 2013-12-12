/*
 * Serial.cpp
 *
 *  Created on: Oct 21, 2011
 *      Author: tero
 */
#include "stm32f2xx.h"
#include "stm32f2xx_rcc.h"
#include "stm32f2xx_usart.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_dma.h"
#include "misc.h"//=NVIC
#include "Serial.h"
#include <stdlib.h>
#include "FreeRTOSConfig.h"
#include "System.h"
#include "utils.h"

bool PortRS485Open = false;
bool PortDSCOpen = false;

Serial::Serial(Port port, System *parent) {
	thisPort = port;
	GPIO_InitTypeDef GPIO_InitStructure;
	txBufPos = rxBufPos = 0;

	if (port == RS485) {
		if (PortRS485Open)
		{
			parent->setFault(FLT_FIRMWARE,500101);
			return; //fail because already opened
		}
		PortRS485Open = true;

		rxBuffer=(u8*)malloc(RXBUFSIZE);
		txBuffer=(u8*)malloc(TXBUFSIZE);

		/* Enable USARTz Clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		/* Enable the DMA clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

		/* Connect USART pins to AF7 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

		//TX=PA9 AF7
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//RX=PA10 AF7
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//TXEN=PC10
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

		GPIO_Init(GPIOC, &GPIO_InitStructure);
		RS485_EnableTransmit(false);

		/* Enable the USART OverSampling by 8 */
		//by default oversampling=16 USART_OverSampling8Cmd(USART2, ENABLE);
		USART_InitTypeDef USART_InitStructure;
		USART_InitStructure.USART_BaudRate = 460800; //1.5mbps=max @24MHz (koska 24M/16=1.5M)
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		/* Configure USARTy */
		USART_Init(USART1, &USART_InitStructure);

		/* Enable USARTy DMA TX request */
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

		/* Enable USARTy */
		USART_Cmd(USART1, ENABLE);
		USART_ITConfig(USART1, USART_IT_TC, ENABLE);

		/* Enable the USARTx Interrupt, used for setting RS485 transmit enable=false after transmit complete */
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;//higher value=lower priority;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init_GD(&NVIC_InitStructure);

		//DMA2_Stream2==RX
		{

			//common?
			DMA_StructInit(&DMA_InitStructure);
			DMA_InitStructure.DMA_PeripheralBaseAddr =
					((uint32_t) USART1 + 0x04);
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize =
					DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
			//DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

			//tx
			DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) txBuffer;
			DMA_InitStructure.DMA_BufferSize = (uint16_t) TXBUFSIZE;
			DMA_Init(DMA2_Stream7, &DMA_InitStructure);

			//RX
			DMA_DeInit(DMA2_Stream2);
			DMA_InitStructure.DMA_PeripheralBaseAddr =
					((uint32_t) USART1 + 0x04);
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) rxBuffer;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
			DMA_InitStructure.DMA_BufferSize = RXBUFSIZE;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize =
					DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
			DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
			DMA_Init(DMA2_Stream2, &DMA_InitStructure);
			DMA_Cmd(DMA2_Stream2, ENABLE);
		}
	}

	//for DSC serial
	if (port == DSC) {
		if (PortDSCOpen)
		{
			parent->setFault(FLT_FIRMWARE,500102);
			return; //fail because already opened
		}
		PortDSCOpen = true;

		rxBuffer=(u8*)malloc(RXBUFSIZE);
		txBuffer=(u8*)malloc(TXBUFSIZE);


		/* Enable USARTz Clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		/* Enable the DMA clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

		/* Connect USART pins to AF7 */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

		//TX=PA2 AF7
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		//RX=PA3 AF7
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Enable the USART OverSampling by 8 */
		//by default oversampling=16 USART_OverSampling8Cmd(USART2, ENABLE);
		USART_InitTypeDef USART_InitStructure;

		//USART_InitStructure.USART_BaudRate = 468750; //1.5mbps=max @24MHz (koska 24M/16=1.5M)
		USART_InitStructure.USART_BaudRate = 416667; //1.5mbps=max @24MHz (koska 24M/16=1.5M)
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		/* Configure USARTy */
		USART_Init(USART2, &USART_InitStructure);

		/* Enable USARTy DMA TX request */
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

		/* Enable USARTy */
		USART_Cmd(USART2, ENABLE);

		//DMA1_Stream5==RX
		{

			//common?
			DMA_StructInit(&DMA_InitStructure);
			DMA_InitStructure.DMA_PeripheralBaseAddr =
					((uint32_t) USART2 + 0x04);
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize =
					DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
			//DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
			DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
			DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
			DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
			DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

			//TX
			DMA_InitStructure.DMA_Channel = DMA_Channel_4;
			DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) txBuffer;
			DMA_InitStructure.DMA_BufferSize = (uint16_t) TXBUFSIZE;
			DMA_Init(DMA1_Stream6, &DMA_InitStructure);

			DMA_DeInit(DMA1_Stream5);
			DMA_InitStructure.DMA_PeripheralBaseAddr =
					((uint32_t) USART2 + 0x04);
			DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) rxBuffer;
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
			DMA_InitStructure.DMA_BufferSize = RXBUFSIZE;
			rxPacketLen = RXBUFSIZE;
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
			DMA_InitStructure.DMA_PeripheralDataSize =
					DMA_PeripheralDataSize_Byte;
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
			//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
			DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
			DMA_Init(DMA1_Stream5, &DMA_InitStructure);
			DMA_Cmd(DMA1_Stream5, ENABLE);


			/* Enable the USARTx Interrupt */
			NVIC_InitTypeDef NVIC_InitStructure;
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;//higher value=lower priority
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init_GD(&NVIC_InitStructure);
		}
	}
}

Serial::~Serial() {
	//DMA's must be disabled to avoid overwriting unallocated buffer memories
	if (thisPort == RS485) {
		PortRS485Open = false;
		DMA_Cmd(DMA2_Stream7, DISABLE);
		DMA_Cmd(DMA2_Stream2, DISABLE);
	}

	if (thisPort == DSC) {
		PortDSCOpen = false;
		DMA_Cmd(DMA1_Stream6, DISABLE);
		DMA_Cmd(DMA1_Stream5, DISABLE);
	}
}

//return true if success
bool Serial::armReceiveCompleteEvent( bool on, u16 rxlen )
{
	//NA in this port
	if(thisPort==RS485) return false;
	if(rxlen>RXBUFSIZE) return false;

	if(on)
	{
		rxPacketLen = rxlen;
		rxBufPos=0;

		//Transmission Complete flag.
		DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, DISABLE);
		DMA_Cmd(DMA1_Stream5, DISABLE);
		DMA_ClearITPendingBit( DMA1_Stream5, DMA_IT_TCIF5 );
		DMA_SetCurrDataCounter(DMA1_Stream5, rxlen);
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
		DMA_Cmd(DMA1_Stream5, ENABLE);
		DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
	}
	else
	{
		DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, DISABLE);
	}
	return true;
}

#define RET_NODATA 254
//returns byte from RX buffer. sets emtpy to true if no data to be received
u8 Serial::getByte(bool &empty) {
	u16 dmaCounter;

	//circular buffer mode
	if (thisPort == RS485)
	{
		dmaCounter = 256 - DMA_GetCurrDataCounter(DMA2_Stream2); //counts downwards as data is received

		//if there is data
		if (rxBufPos != dmaCounter) {
			u8 ret = rxBuffer[rxBufPos];
			rxBufPos++;
			if (rxBufPos >= RXBUFSIZE)
				rxBufPos = 0;
			return ret;
		}
	}
	//normal mode because of transfer complete (TC) interrupt needed
	else if (thisPort == DSC) {
		dmaCounter = rxPacketLen - DMA_GetCurrDataCounter(DMA1_Stream5); //counts downwards as data is received
		//if there is data
		if (rxBufPos < dmaCounter) {
			u8 ret = rxBuffer[rxBufPos];
			rxBufPos++;
			return ret;
		}
	}
	empty=true;
	return RET_NODATA;
}

//read buffer, return bytes read
int Serial::getBuffer( u8 *buf, int len )
{
	int i;
	bool fail=false;
	for(i=0;i<len;i++)
	{
		buf[i]=getByte(fail);
		if(fail==true) break;
	}
	return i;
}

void Serial::sendBuffer() {

	//if no data
	if (txBufPos < 1)
		return;

	if (thisPort == RS485) {
		RS485_EnableTransmit(true); //will be set to false at transfer complete ISR

		//Transmission Complete flag.
		USART_ClearFlag(USART1, USART_FLAG_TC);

		DMA_Cmd(DMA2_Stream7, DISABLE);
		DMA_SetCurrDataCounter(DMA2_Stream7, txBufPos);
		DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
		DMA_Cmd(DMA2_Stream7, ENABLE);

	}

	else if (thisPort == DSC) {
		//Transmission Complete flag.
		USART_ClearFlag(USART2, USART_FLAG_TC);

		DMA_Cmd(DMA1_Stream6, DISABLE);
		DMA_SetCurrDataCounter(DMA1_Stream6, txBufPos);
		DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
		DMA_Cmd(DMA1_Stream6, ENABLE);
	}

	txBufPos = 0;
}



void Serial::putByte(u8 byte) {
	if (txBufPos < TXBUFSIZE) {
		txBuffer[txBufPos] = byte;
		txBufPos++;
	}
}

void Serial::putBuffer( u8 *buf, int len )
{
	int i;
	for(i=0;i<len;i++)
		putByte(buf[i]);
}

void RS485_EnableTransmit(bool on) {
	//proton purkka TTL232 kaapelille
	//GPIO_SetBits(GPIOC, GPIO_Pin_10);return;


	if (on)
		GPIO_SetBits(GPIOC, GPIO_Pin_10);
	else
		GPIO_ResetBits(GPIOC, GPIO_Pin_10);
}

//called from ISR
void RS485_TransmitCompleteEvent() {
	if (USART_GetITStatus(USART1, USART_IT_TC) == SET) {
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		RS485_EnableTransmit(false);
	}
}

//return true if transmitting of buffer is complete
bool Serial::transmitComplete()
{
	if (thisPort == RS485) {
		//Transmission Complete flag.
		if( USART_GetFlagStatus(USART1, USART_FLAG_TXE) == SET)
			return true;
		else
			return false;
	}
	else if (thisPort == DSC) {
		//Transmission Complete flag.
		if( USART_GetFlagStatus(USART2, USART_FLAG_TXE) == SET)
			return true;
		else
			return false;
	}

	return false;
}

bool Serial::setBaudRate(s32 bps)
{
	if (thisPort == RS485) {
		if(bps<1000 || bps > 2000000 ) return false;

		USART_InitTypeDef USART_InitStructure;
		USART_InitStructure.USART_BaudRate = bps; //1.5mbps=max @24MHz (koska 24M/16=1.5M)
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl =
				USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		/* Configure USARTy */
		USART_Init(USART1, &USART_InitStructure);

		return true;
	}
	return false;//wrong port
}


