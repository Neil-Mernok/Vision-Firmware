/*
 * Global_Variables.c
 *
 *  Created on: Jun 13, 2013
 *      Author: KobusGoosen
 */

#include "Global_Variables.h"

RCC_ClocksTypeDef 			RCC_Clocks;
EXTI_InitTypeDef   			EXTI_InitStructure;
GPIO_InitTypeDef   			GPIO_InitStructure;
FSMC_NORSRAMInitTypeDef  	FSMC_NORSRAMInitStructure;
USART_InitTypeDef 			USART_InitStructure;
NVIC_InitTypeDef   			NVIC_InitStructure;
ADC_InitTypeDef       		ADC_InitStructure;
DMA_InitTypeDef       		DMA_InitStructure;
SPI_InitTypeDef  			SPI_InitStructure;

uint8_t uart_packet_received;
uint8_t uart_data[2048];
uint16_t uart_buf_counter;

CanTxMsg TxMessage;
CanRxMsg RxMessage;

#ifdef STM32F10X_CL
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;
#endif

//#define debug_print

///Used for printf etc

#ifdef debug_print
int prwrite(int fd, char *str, int len)
{
	int i;
	static _Q_MasterIF MIF;

#ifdef USE_USB
#ifdef STM32F10X_HD
	if (Ranger_Status.USB_Selected && bDeviceState == CONFIGURED)
	{
//		return CDC_Send_DATA(str, len);
		MIF.data = str;
		MIF.len = len;
		push_to_master(MIF);
	}
#endif
#ifdef STM32F10X_CL
	if (Vision_Status.USB_Active)
	{
		APP_FOPS.pIf_DataTx(str, len);
		return len;
	}
#endif
	else
#endif
	{
		//	USART_SendDMA(COM_1, str, len);
		MIF.data = str;
		MIF.len = len;
		push_to_master(MIF);
		return len;
	}
	return i;
}
#else

//#define DBGlen	4096
//uint8_t DBGbuff[DBGlen];

int prwrite(int fd, char *str, int len)
{
//	int i;
//	static int point = 0;
//
//	for (i = 0; i < len; i++)
//	{
//		DBGbuff[point++] = str[i];
//		if(point >= DBGlen) point = 0;
//	}
//	return i;

	return len;
}
#endif
