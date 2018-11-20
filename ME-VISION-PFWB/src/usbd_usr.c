/**
 ******************************************************************************
 * @file    usbd_usr.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    19-March-2012
 * @brief   This file includes the user application layer
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usbd_usr.h"
#include "usbd_ioreq.h"
#include "Global_Variables.h"

//#define USB_LOGGING
#ifdef USB_LOGGING

#define debug_log_max 2048
char USB_debug_log[debug_log_max];

void usb_print_to_log(char* data)
{
	static int pos = 0;

	if(pos > debug_log_max - 50)
		pos = 0;
	strcpy(&USB_debug_log[pos], data);
	pos += strlen(data);
}
#endif

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_USR 
 * @brief    This file includes the user application layer
 * @{
 */

/** @defgroup USBD_USR_Private_TypesDefinitions
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Defines
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Macros
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Variables
 * @{
 */

USBD_Usr_cb_TypeDef USR_cb =
{ USBD_USR_Init, USBD_USR_DeviceReset, USBD_USR_DeviceConfigured, USBD_USR_DeviceSuspended, USBD_USR_DeviceResumed,

USBD_USR_DeviceConnected, USBD_USR_DeviceDisconnected, };

/**
 * @}
 */

/** @defgroup USBD_USR_Private_Constants
 * @{
 */

/**
 * @}
 */

/** @defgroup USBD_USR_Private_FunctionPrototypes
 * @{
 */
/**
 * @}
 */

/** @defgroup USBD_USR_Private_Functions
 * @{
 */

/**
 * @brief  USBD_USR_Init 
 *         Displays the message on LCD for host lib initialization
 * @param  None
 * @retval None
 */
void USBD_USR_Init(void)
{
#ifdef USB_LOGGING
	usb_print_to_log("Init device\n");
#endif
}

/**
 * @brief  USBD_USR_DeviceReset 
 *         Displays the message on LCD on device Reset Event
 * @param  speed : device speed
 * @retval None
 */
void USBD_USR_DeviceReset(uint8_t speed)
{
#ifdef USB_LOGGING
	usb_print_to_log("reset device\n");
#endif
	// Configure the USB+ pullup enable pin
//	RCC_APB2PeriphClockCmd(USB_Pull_GPIO_CLK, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = USB_Pull_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//	GPIO_Init(USB_Pull_PORT, &GPIO_InitStructure);
//	GPIO_ResetBits(USB_Pull_PORT, USB_Pull_PIN);		// enable pull-up
}

/**
 * @brief  USBD_USR_DeviceConfigured
 *         Displays the message on LCD on device configuration Event
 * @param  None
 * @retval Staus
 */
void USBD_USR_DeviceConfigured(void)
{
#ifdef USB_LOGGING
	usb_print_to_log("device configued\n");
#endif
//	Vision_Status.USB_Active = 1;
//	Vision_Status.USB_Working = true;
}

/**
 * @brief  USBD_USR_DeviceSuspended 
 *         Displays the message on LCD on device suspend Event
 * @param  None
 * @retval None
 */
void USBD_USR_DeviceSuspended(void)
{
#ifdef USB_LOGGING
	usb_print_to_log("device suspended\n");
#endif
	/* Users can do their application actions here for the USB-Reset */
	
	// Configure the USB+ pullup enable pin
//	RCC_APB2PeriphClockCmd(USB_Pull_GPIO_CLK, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = USB_Pull_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(USB_Pull_PORT, &GPIO_InitStructure);
//	Vision_Status.USB_Active = 0;
}

/**
 * @brief  USBD_USR_DeviceResumed 
 *         Displays the message on LCD on device resume Event
 * @param  None
 * @retval None
 */
void USBD_USR_DeviceResumed(void)
{
#ifdef USB_LOGGING
	usb_print_to_log("device resumed\n");
#endif
	/* Users can do their application actions here for the USB-Reset */
	// Configure the USB+ pullup enable pin
//	RCC_APB2PeriphClockCmd(USB_Pull_GPIO_CLK, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = USB_Pull_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//	GPIO_Init(USB_Pull_PORT, &GPIO_InitStructure);
//	GPIO_ResetBits(USB_Pull_PORT, USB_Pull_PIN);		// enable pull-up
}

/**
 * @brief  USBD_USR_DeviceConnected
 *         Displays the message on LCD on device connection Event
 * @param  None
 * @retval Staus
 */
void USBD_USR_DeviceConnected(void)
{
#ifdef USB_LOGGING
	usb_print_to_log("device Connected\n");
#endif
	// Configure the USB+ pullup enable pin
//	RCC_APB2PeriphClockCmd(USB_Pull_GPIO_CLK, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = USB_Pull_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//	GPIO_Init(USB_Pull_PORT, &GPIO_InitStructure);
//	GPIO_ResetBits(USB_Pull_PORT, USB_Pull_PIN);		// enable pull-up
}

/**
 * @brief  USBD_USR_DeviceDisonnected
 *         Displays the message on LCD on device disconnection Event
 * @param  None
 * @retval Staus
 */
void USBD_USR_DeviceDisconnected(void)
{
#ifdef USB_LOGGING
	usb_print_to_log("device disconnected\n");
#endif
	// Configure the USB+ pullup enable pin
//	RCC_APB2PeriphClockCmd(USB_Pull_GPIO_CLK, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = USB_Pull_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_Init(USB_Pull_PORT, &GPIO_InitStructure);
}
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
