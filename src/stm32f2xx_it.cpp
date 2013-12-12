/**
 ******************************************************************************
 * @file    Project/STM32F2xx_StdPeriph_Template/stm32f2xx_it.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    18-April-2011
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_it.h"
#include "main.h"
#include "Serial.h"
#include "stm32f2xx_exti.h"
#include "DigitalCounterInput.h"
#include "globals.h"


/** @addtogroup Template_Project
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler( void )
{
	sys.setFault(FLT_PROGRAM_OR_MEM,200101);
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler( void )
{
	sys.setFault(FLT_PROGRAM_OR_MEM,200201);
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler( void )
{
	sys.setFault(FLT_PROGRAM_OR_MEM,200301);
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler( void )
{
	sys.setFault(FLT_PROGRAM_OR_MEM,200401);
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler( void )
{
	sys.setFault(FLT_PROGRAM_OR_MEM,200501);
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler( void )
{
	sys.setFault(FLT_PROGRAM_OR_MEM,200601);
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler( void )
{
	sys.setFault(FLT_PROGRAM_OR_MEM,200701);
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler( void )
{
	sys.setFault(FLT_PROGRAM_OR_MEM,200801);
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
/*void SysTick_Handler(void)
 {
 TimingDelay_Decrement();
 }*/

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "globals.h"
#include "stm32f2xx_dma.h"
#include "ResolverIn.h"
#include "stm32f2xx_tim.h"

//interrupt on DMA buffer sent via USART1
void USART1_IRQHandler()
{
	RS485_TransmitCompleteEvent();
}

#include "utils.h"
#include "core_cm3.h"
//packet received from DSC complete interrupt -> send new packet to DSC
void DMA1_Stream5_IRQHandler()
{
	//check if this is correct interrupt
	if (DMA_GetITStatus( DMA1_Stream5, DMA_IT_TCIF5 ) == SET)
	{
		volatile int a,b;
		a=__get_BASEPRI();
		b=__get_PRIMASK();

		//clear interrupt flag to avoid repeating interrupts
		DMA_ClearITPendingBit( DMA1_Stream5, DMA_IT_TCIF5 );

		portBASE_TYPE xYieldRequired = pdFALSE;

		/* Unblock the task by releasing the semaphore. */
		xSemaphoreGiveFromISR( MCCommTaskSemaphore, &xYieldRequired );

		//must be called because we're at ISR
		portEND_SWITCHING_ISR( xYieldRequired );
	}
}

/**
 * @brief  This function handles External lines 15 to 10 interrupt request. This case BP11 which is direction pin in pulse train mode.
 * @param  None
 * @retval None
 */
void EXTI1_IRQHandler( void )
{
	//NVIC_InitTypeDef NVIC_InitStructure;

	if (EXTI_GetITStatus( PULSETRAIN_DIR_EXTI_LINE ) == SET)
	{

		//set counting direction according to DIR (HSIN2) pin state
		sys.digitalCounterInput.setCountingDirection( sys.physIO.getDirectionPinState() );

		/* Clear PULSETRAIN_DIR_EXTI_LINE pending bit */
		EXTI_ClearITPendingBit( PULSETRAIN_DIR_EXTI_LINE );
	}
}

//System class high frequency task isr
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(HIFREQ_TASK_TIMER, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(HIFREQ_TASK_TIMER, TIM_IT_CC1);
		//sys.physIO.doutDebug1.setState(1);
		sys.highFrequencyISRTask();
		//sys.physIO.doutDebug1.setState(1);
//		sys.physIO.doutDebug1.setState(0);
	}
}

extern bool adcread;
#include "stm32f2xx_adc.h"
void ADC_IRQHandler(void)
{
	ADC_ClearITPendingBit(ADC1,ADC_IT_EOC);
	adcread=true;

//	sys.physIO.doutDebug1.setState(1);
	sys.physIO.doutDebug1.setState(0);

}


/**
 * @}
 */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
