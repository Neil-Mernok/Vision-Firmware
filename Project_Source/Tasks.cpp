/*
 * Tasks2.cpp
 *
 *  Created on: May 11, 2015
 *      Author: Kobus
 */

#include "Tasks.h"

uint8_t LF_trnasmitter_Send = 0;
extern Zone_Info_Type LF_CAN_Sync_List[5];

/// need for the CC1101 initialisation steps.

/**
 * @ the purpose of this task is to wait for messages from the master and answer them when ready. 
 * @ it can also accept messages from the code intended for the master and forward them along. 
 * @param pvParameters
 */
uint32_t TimesinceSent = 0;
void Master_task(task* t, int* cant_sleep)
{
	static uint32_t lastHeartBeat = 0;
	static _Q_MasterIF MIF =
	{ };
	static uint32_t last_Sent = 0;

	if (t->state == 0)			// this is run the first time only. 
	{
		// create a pipe to hold messages we need to forward. max 50 messages 
		pipe_init(&(t->p), 280, sizeof(_Q_MasterIF));
		t->state = 1;						// don't run this again. 
	}

	if (MIF.len != 0) // not finished with current message, try sending again.
	{
		if ((TimesinceSent = time_since(last_Sent)) >= 1)	 // wait a few milliseconds between messages (Todo: Check why L4 DMA isnt working well...)
		{
			/// try sending some more data. make sure not to send non-boot related responses during boot bypass mode.
			if(Vision_Status.boot_mode_time == 0 || MIF.data[0] == rf_Boot_R)
				Send_to_Master( &MIF );
			else
				MIF.len = 0;

			if (MIF.len == 0)
			{
				last_Sent = time_now();
				//(*cant_sleep)++;
			}
		}
	}
	else
	{
		/// try to get a new packet for next time.
		pipe_get(&(t->p), &MIF);
	}

	if (time_now() > lastHeartBeat)
	{
		if (vision_settings.getActivities().heartbeat)
			send_heartbeat(Vision_Status.last_master_coms);
		lastHeartBeat = time_now() + (int)vision_settings.interval + 100;
	}
}

//#define report_debug_data
char pcWriteBuffer[400];				// collects the current runtime stats. 

void GetStatus(task* t)
{
	// check if the timeout period has elapsed...
	if (t->pause_until > time_now())
		return;	// this was setup to allow it to run the first time regardless. 

	// postpone task by 1s before running again
	task_delay(t, 1000);

	// take ADC readings
	measure_volts();

#if defined (Charging_PIN)
	Vision_Status.sts.Charging = (!GPIO_ReadInputDataBit(Charging_PORT, Charging_PIN));
#endif

#if defined (ANT_SENSE_PIN)		// if we have a detect pin	
	Vision_Status.sts.Antenna_connected = (!GPIO_ReadInputDataBit(ANT_SENSE_PORT, ANT_SENSE_PIN));
#endif

#ifdef CAN_term_PIN
#ifdef USE_HAL_DRIVER
	if (((vision_settings.getActivities().CAN_terminated) != 0) && (Vision_Status.sts.EXT_Power))
		HAL_GPIO_WritePin(CAN_term_PORT, CAN_term_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(CAN_term_PORT, CAN_term_PIN, GPIO_PIN_RESET);
#else
	if (((vision_settings.getActivities().CAN_terminated) != 0) && (Vision_Status.sts.EXT_Power))
		GPIO_WriteBit(CAN_term_PORT, CAN_term_PIN, Bit_SET);
	else
		GPIO_WriteBit(CAN_term_PORT, CAN_term_PIN, Bit_RESET);
#endif
#endif
}


void Refresh_Settings_task(task* t)
{

	if (t->state == -1)
		return;

	// check if the timeout period has elapsed...
	if ((t->pause_until > time_now()) && (Vision_Status.Setting_changed == 0))
		return;	// this was setup to allow it to run the first time regardless. 

	// This function needs to be run after a setting is changed, but not immediately, in case others are busy changing. 
	if ((Vision_Status.Setting_changed != 0) && (time_since(Vision_Status.Setting_changed) < 500))
		return;

	// dont modify the eeprom when the LF TX is active, or LF reception is underway.  
	if (LF_Params.state || AS3933_LF_indication())
	{
		task_delay(t, 20);
		return;
	}

	Vision_Status.Setting_changed = 0;

	// postpone task by 60s before running again
	task_delay(t, 60000);

	GPIO_SetBits(VRFID_PORT, VRFID_PIN);	// power the eeprom

#ifdef USE_HAL_DRIVER
	Delay(1);								// give some time for the eeprom to warm up. this is necessary!
	MX_I2C1_Init(400000);
#else
	Delay(1);
	F103_i2c_init();
#endif


	// get data from the eeprom and store in the settings struct
	SetLed(&LED1, White, 0);
#ifdef USE_HAL_DRIVER
	HAL_NVIC_DisableIRQ(LF_CLK_IRQ_Channel);
	HAL_NVIC_DisableIRQ(CC_GDO_IRQ_Channel);
#else
	__disable_irq();
#endif

	static int settings_error_count = 0;			// variable to keep track of eeprom failures. 


	if (GetSettings() == 0)
	{
#ifdef USE_HAL_DRIVER
		MX_I2C1_Init(100000);
		Delay(1);

#else
		F103_i2c_init();
		DelayUs(1000);
#endif

		if (GetSettings() == 0)
		{
#ifdef USE_HAL_DRIVER
			HAL_NVIC_EnableIRQ(LF_CLK_IRQ_Channel);
			HAL_NVIC_EnableIRQ(CC_GDO_IRQ_Channel);
			HAL_I2C_DeInit(&hi2c1);
#else
			__enable_irq();
#endif
			GPIO_ResetBits(VRFID_PORT, VRFID_PIN);			// power down eeprom.
			task_delay(t, 1400);
			if(settings_error_count++ > 10)
				NVIC_SystemReset();
			else
				return;
		}
	}
	settings_error_count = 0;
#ifdef USE_HAL_DRIVER
	HAL_NVIC_EnableIRQ(LF_CLK_IRQ_Channel);
	HAL_NVIC_EnableIRQ(CC_GDO_IRQ_Channel);
	GPIO_ResetBits(VRFID_PORT, VRFID_PIN);			// power down eeprom.
#else
	__enable_irq();
#endif

	SetLed(&LED1, LED1.last_color, 0);

#ifdef USE_HAL_DRIVER
	static uint32_t interval_last = 0;
	if(interval_last != (uint32_t) vision_settings.interval)
	{
		if((uint32_t) vision_settings.interval > 15000)
			HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 4000, RTC_WAKEUPCLOCK_RTCCLK_DIV16);		// set clock for 2s if a very long time is required. we will construct it from numerous sleeps
		else
			HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, (uint16_t) vision_settings.interval * 2, RTC_WAKEUPCLOCK_RTCCLK_DIV16);		// set RTC wake for <interval> milliseconds
	}
	interval_last = vision_settings.interval;
#endif

#ifdef USING_CAN
	if (Vision_Status.CAN_baud_last != (int) vision_settings.can_baud)
	{
		Vision_Status.CAN_baud_last = vision_settings.can_baud;
		if (Vision_Status.CAN_baud_last != 0)
			CAN_Config(vision_settings.can_baud);
	}
	if (Vision_Status.CAN_SID_last != (int) vision_settings.slave_id)
	{
		Vision_Status.CAN_SID_last = vision_settings.slave_id;
		CAN_setup_filter(CAN_Vision_Poll_ID | Vision_Status.CAN_SID_last, 1);		// reconfigure the specific CAN filter to our SID
		CAN_setup_filter(CAN_Vision_Poll_ID | 0x00, 3);								// reconfigure the generic CAN filter

		//Todo: Neil added for broadcast messages 0xFF
		CAN_setup_filter(CAN_Vision_Poll_ID | 0xFF, 4);								// reconfigure the generic CAN filter
	}
#endif
	if (Vision_Status.UART_baud_last != (int) vision_settings.uartBaud)
	{
		Vision_Status.UART_baud_last = vision_settings.uartBaud;
		if (Vision_Status.UART_baud_last != 0)
		{
#ifdef USE_HAL_DRIVER
			USART_Configure(vision_settings.uartBaud);
#else
			USART_Configure(Master_COM, vision_settings.uartBaud);
#endif
		}
	}

	Vision_Status.kind = get_TAG_kind();
	if(!vision_settings.getActivities().Always_on)
	{
		if (time_since(Vision_Status.LastRF) > 300000)					// reset if we haven't seen any RF activity in 5 minutes. also for non-receiver devices.
		{
			if ((Vision_Status.sts.USB_Active == false))
			{
				NVIC_SystemReset();
			}
		}
		else if (time_since(Vision_Status.last_master_coms) > 300000)			// reset if we haven't seen any master activity in the last 5 minutes
		{
			if (Vision_Status.sts.USB_Active == false)
			{
				NVIC_SystemReset();
			}
		}
	}
	else if (time_now() > 2000000000)								// reset if we're close to systic overflow
	{
		NVIC_SystemReset();
	}

	// tell the bootloader that we are running fine...
#ifdef USE_HAL_DRIVER
	HAL_RTCEx_BKUPWrite(&hrtc, boot_reason_loc, Boot_reason_none);
#else
	BKP_WriteBackupRegister(boot_reason_loc, Boot_reason_none);
#endif

#ifdef report_debug_data
#if (configGENERATE_RUN_TIME_STATS)
	vTaskGetRunTimeStats(pcWriteBuffer);
	puts(pcWriteBuffer);
	printf("error count: %u\n", error_count);
#endif

	// wait for 20s...
	vTaskDelay(10000);

	i = 0;
	while (transp_filter(i, 1000, 60, 0))
	{
		printf("transp: %d \t Dist: %d.%dm, \t last seen: %ds ago.\n", i, t.Dist / 10, t.Dist % 10, (xTaskGetTickCount() - t.last_seen) / 1000);
		i++;
	}
#endif
}

/**
 * @brief: this task starts the USB
 */
void USB_start(void)
{
	//	if (Vision_Status.USB_Active)
	//	{
#ifdef STM32F10X_HD
	USB_Set_System();
	Set_USBClock();
	USB_Interrupts_Config();
	USB_Init();
#endif
#ifdef STM32F10X_CL
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
#endif
	//	}
}

/**
 * @brief: Checks if the GPS capable flag is set or not, this determines the _kind
 * @return: The _kind of the TAG
 */
uint8_t get_TAG_kind(void)
{
	// ---- Check the GPS_capable flag ----
	if (vision_settings.getActivities().GPS_capable)
	{
		// ---- Return Pulse_GPS _kind ----
		return (uint8_t) Pulse_GPS;
	}
	else
	{
		// ---- Return Pulse _kind ----
		return (uint8_t) Pulse;
	}
}

// define the states the RF state machine can adopt
#define CC_none 2
#define CC_RX 2
#define CC_TX 3
#define CC_Idle 4

uint8_t  packets_send = 0;

/**
 * @brief: this task controls the CC1101 chip. 
 * @param t = task handle, rf_wake_flag is set to say thay it must send RF, cant_sleep gets set if it cant sleep yet.  
 */
void CC1101_Task(task* t, int* rf_wake_flag, int* cant_sleep)
{
	int timeout;
	static uint32_t last_ID_broadcast = 0;
	uint8_t PayloadLength = 0, RSSI = 0, status = 0, rev;
	RF_message to_send;//, send_ID;

	static uint8_t data[64];

	if (t->state == 0)			// this is run the first time only. 
	{
		// create a pipe to hold IDs we need to range to. 
		pipe_init(&(t->p), 50, sizeof(RF_message));

		//Delay(500);

		halRfResetChip();

		//Delay(500);

		rev = halRfGetChipVer();

		if ((rev != 4) && (rev != 20))
		{
			Vision_Status.sts.CC_SPI_working = 0;
			t->state = -1;
			return;
		}

		/// added to indicate Healthy SPI to CC chip.
		Vision_Status.sts.CC_SPI_working = 1;

		// write RF settings to chip
		if ((int) vision_settings.rf_power > 7)
			vision_settings.rf_power = 7;

		myPaTable[0] = PAsettings[(int) vision_settings.rf_power];
		halRfConfig(&myRfConfig_250, myPaTable, myPaTableLen);

#ifdef RF_DEBUG //keeps the channel the bootloader channel for debugging
		halRfWriteReg(CC1100_CHANNR, 3);     // Channel number 0;
		Vision_Status.boot_mode_time = 0;
#endif
		Config_EXTI_GD0();
		t->state = CC_none;
	}
	else if (t->state == -1)
		return;

	/////////////////////////////////////////////////////////////////////////////////////////////
	//// this is where the processing happens
	/////////////////////////////////////////////////////////////////////////////////////////////

	// reset the channel to 0 after 10s of sending a boot message. 
	if(time_now() > Vision_Status.boot_mode_time && Vision_Status.boot_mode_time != 0)
	{
		halRfWriteReg(CC1100_CHANNR, 0);     // Channel number 0;
		Vision_Status.boot_mode_time = 0;
	}

	if ((vision_settings.getActivities().tag_enable))
	{
		timeout = (int) vision_settings.interval;

		// this is an RF transmitter device

		// check if there's a message to be sent
		if (pipe_peek(&t->p, &to_send))
		{
			// insert a delay parameter in the packet to tell it to wait a certain amount before responding. creates RF slots. 
			if (to_send.time_to_respond <= time_now())
			{
				pipe_get(&t->p, &to_send);			// remove the message from the list as we're going to process it now. 

				// forcibly switch off boot mode for master messages
				if(Vision_Status.boot_mode_time != 0 && to_send.type == rf_MasterP)
				{
					halRfWriteReg(CC1100_CHANNR, 0);     // Channel number 0;
					Vision_Status.boot_mode_time = 0;
				}

				// to send boot messages, switch to RF channel 3, and leave it there for a few seconds
				if (to_send.type == rf_Boot_M)
				{
					halRfWriteReg(CC1100_CHANNR, 3);     // Channel number 3 for boot messages
					Vision_Status.boot_mode_time = time_now() + 10000;
					Apl_master_boot_message(to_send.buff, to_send.len);
				}
				else if(Vision_Status.boot_mode_time == 0)
				{										// only send other RF if we're not in boot-load mode.  

					switch (to_send.type)
					{
					case rf_LF_resp:
						Apl_report_LF(to_send.LF);
						// check when last we send tag info like VID, SID, etc. If its been too long, send name message. 
						//						if(time_since(last_ID_broadcast) > 2 * timeout)
						//						{
						//							send_ID.type = rf_ID_name;
						//							send_ID.time_to_respond = time_now()+5;
						//						}
						//						// make sure we wait a certain amount before sending another RF ping
						//						task_delay(t, timeout);
						break;
					case rf_LF_send:
						Apl_sync_LF();
						break;
					case rf_MasterP:
						Apl_send_master_message(to_send.buff, to_send.len);
						break;
					case rf_Data:
						Apl_send_data_message(to_send.buff, to_send.len);
						break;
					case rf_Respons:
						Apl_send_master_response(to_send.buff, to_send.len);
						break;
					case rf_ID_name:
						// forcibly send name massage.
						SetLed(&LED1, Green, 0);
						Apl_report_name_var(to_send.buff[0], (char*) &to_send.buff[1]);
						SetLed(&LED1, LED1.last_color, 0);
						last_ID_broadcast = time_now();
						task_delay(t, timeout);							// don't send another RF ping until necessary
						break;
					case rf_legcPDS:
						// forcibly send PDS massage.
						Apl_Send_PDS(to_send.buff[0]);
						if(vision_settings.getActivities().legacy_PDS)
							task_delay(t, timeout);							// don't send another RF ping until necessary
						break;
					case rf_ID_puls:
						SetLed(&LED1, Green, 0);
						Apl_report_ID(to_send.buff[0]);
						SetLed(&LED1, LED1.last_color, 0);
						last_ID_broadcast = time_now();
						task_delay(t, timeout);							// don't send another RF ping until necessary
						break;

					case rf_GPS_C:
						Apl_report_GPS_Coordinates();
						break;

//					case rf_Time:
//						Apl_broadcast_Time();
//						break;

					case rf_Distress:
						Apl_report_Distress(to_send.buff[0]);
						break;

					default:
						break;
					}
				}
			}
			(*cant_sleep)++; // cc chip still busy, so we cant sleep.
			t->state = CC_TX;
		}


		if (((*rf_wake_flag) != 0) || (time_now() > t->pause_until))// timer for the task has expired. need to send ID ping.
		{
			if (Vision_Status.boot_mode_time == 0)
			{
				(*rf_wake_flag) = 0;

				// ---- Output LED only when broadcast ID is enabled ----
				if (vision_settings.getActivities().broadcast_ID)
				{
#ifdef WARNING_OUTPUTS
					if(Vision_Status.sts.EXT_Power && Vision_Status.sts.Charging)
						SetLed(&LED1, Violet, 400);
					else
					{
						static int ID_i = 0;
						SetLed(&LED1, Green, 0);
						if((ID_i&1)==1 || vision_settings.getActivities().broadcast_time == 0)
							Apl_broadcast_ID();
						else if (vision_settings.getActivities().broadcast_time)
							Apl_broadcast_Time();
						SetLed(&LED1, LED1.last_color, 0);
						ID_i++;
					}
#else
//					SetLed(&LED1, Green, 0);
//					Apl_broadcast_ID();
//					SetLed(&LED1, LED1.last_color, 0);
					static int ID_i = 0;
					SetLed(&LED1, Green, 0);
					if((ID_i&1)==1 || vision_settings.getActivities().broadcast_time == 0)
						Apl_broadcast_ID();
					else if (vision_settings.getActivities().broadcast_time)
						Apl_broadcast_Time();
					SetLed(&LED1, LED1.last_color, 0);
					ID_i++;
#endif



					task_delay(t, timeout/2 + 100);			// make sure in low power mode, wake_flag causes RF.

//					task_delay(t, timeout + 100);			// make sure in low power mode, wake_flag causes RF.

					(*cant_sleep)++; 						// CC1101 chip still busy, so we can't sleep.
					t->state = CC_TX;
				}
			}
		}

		if ((vision_settings.getActivities().receive_RF) || ((Vision_Status.Force_RF != 0) && (time_now() < Vision_Status.Force_RF))) // this is an RF receiver device
		{
			// indicate RF receive status
			Vision_Status.sts.RF_Receiving = 1;

			memset(data, 0, 64);

			(*cant_sleep)++; // CC1101 chip is receiving, so we can't sleep.
			status = rxRecvReentrant(data, &PayloadLength, &RSSI, 4000);

			if (status == 0)		// packet received, no timeout
			{
				// ---- Check if packet received is valid ----
				if (check_RF_frame_crc(data, (PayloadLength - 4)))
				{

					// ---- Output LED only when forward RF is enabled ----
					if (vision_settings.getActivities().forward_RF)
						SetLed(&LED1, Blue, 2);

					Apl_Parse_message(data + 2, PayloadLength - 4, RSSI, Vision_Status.boot_mode_time != 0);
					Vision_Status.LastRF = time_now();
					(*cant_sleep)--;
					//					// ---- Output LED only when forward RF is enabled ----
					//					if (vision_settings.getActivities().forward_RF)
					//						SetLed(&LED1, Off, 0);
				}
			}
			else
			{
				t->state = CC_RX;
			}
		}
		else
			Vision_Status.sts.RF_Receiving = 0;
	}


	if (((*cant_sleep) == 0) && (t->state != CC_Idle))	////	this is where we know the device is idle and can sleep. 
	{
		halRfStrobe(CC1100_SPWD); 	// Put radio to sleep
		t->state = CC_Idle;
	}
}

/**
 * @brief: this task performs all necessary steps related to the AC3933 device. (SPI and GPIO pins need to be set up first)
 * @param pvParameters
 */
void LF_Task(task* t, int* cant_sleep)
{
	int i, LF_Q_message;										//, LF_TX_val;
	static int8_t rssiMax;
	static LF_message_type LF_RX;
	//	uint8_t LF_buf[20];

	/// this gets run the first time we enter the task
	if (t->state == 0)
	{
		//		SPI_change_mode(AS_SPI, false);
		DelayUs(20);

		if (as3933Initialize(vision_settings.lf_hertz) < 0)
		{
			t->state = -1;
			return;
		}

		// inform the system that the LF detector has been seen. 
		Vision_Status.sts.LF_SPI_working = true;
		// create a pipe 
		pipe_init(&(t->p), 10, sizeof(int));

		/* calibrate the AS3933 RCO using SPI */
#ifndef lf_crystal
		as3933CalibrateRCOViaSPI();
#else
#endif

		/* tune the antennas */
		i = as3933AntennaTuning(vision_settings.lf_hertz);
		if (i == 3)
			Vision_Status.sts.LF_tuned = true;	// only set the flag if all channels work. 

		t->state = 1;
	}

	if (t->state == -1)
	{
		return;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////
	//// this is where the processing happens
	/////////////////////////////////////////////////////////////////////////////////////////////

	/// we're awaiting the next LF message
	if (t->state == 1)
	{
		// try to get something from the LF interrupt message Q. 
		LF_Q_message = 0;
		if (pipe_get(&t->p, &LF_Q_message) /*|| (time_now() > t->pause_until)*/)
		{
			// Make sure we're not busy with LF_TX, so can't receive.
			if (LF_Params.state == 0)
			{
				if (LF_Q_message == 1)		// start of message, so read RSSI
				{
					as3933LfSampleActive = 0;
					rssiMax = as3933GetStrongestRssi();
					t->state = 2; 		// tell the system to wait for the end of message
					//Vision_Status.Force_RF = time_now() + 6;	// Keep the RF receiver on for a bit to catch LF alerts.
					Vision_Status.LF_alert.last_LF = 0;			// use the Last LF of the alert LF as indication of an alert.
					Vision_Status.LF_alert.RSSI = rssiMax;		// Store the LF RSSI so we can send a resonse later, even if LF was missed. 
				}
			}
			else
			{
				// busy with LF TX, so might be my own LF...
				as3933SendCommand(clear_wake);
			}
		}
		//		else
		//		{
		//			/// todo: check the logic behind this... not sure.
		//
		//		}
	}
	/// we've receive LF start, now wait for it to timeout. 
	if (t->state == 2)
	{
		(*cant_sleep)++; // as long as the LF is somewhat active, prevent sleep.

		// we're reached the end of the LF message, complete or not. 
		if (AS3933_LF_check_timout())
		{
			// tell the state-machine to go back to waiting. 
			t->state = 1;

			as3933SendCommand(clear_wake);

			int LF_success = AS3933_LF_get_data(&LF_RX, vision_settings.getActivities().disable_LF_CRC);
			if(LF_success == 0 && Vision_Status.LF_alert.last_LF != 0)
			{
				// LF failed, but we got a LF-alert, so use instead. 
				LF_RX.RSSI = rssiMax;
				LF_success = 1;
			}
			if(LF_success != 0)					
			{
				// if we got an LF-alert, use it in stead. 
				if (Vision_Status.LF_alert.last_LF != 0)
					LF_RX = Vision_Status.LF_alert;

				// decide what is done with the LF message. Can be exclusion marker or allow RF reception, or regular LF 
				// set a flag telling the device it is in an LF exclusion zone for the next 6s
				if(LF_RX.VehicleID >= 0xFFE0)
				{
					// set the tag in exclusion if we are allowed
					if(vision_settings.getActivities().disable_exclusion == 0)
						Vision_Status.exclusion = time_now() + 10000;
				}
				// set a flag indicating that we want this device to keep RF on for 30s
				else if (LF_RX.VehicleID >= 0xFFD0)
				{
					Vision_Status.Force_RF = time_now() + 30000;
				}
				else
					Apl_handle_RSSI(LF_RX);						// valid LF message, so log it to display LF LEDs/buzzer, etc

				// send the LF RX message to the RF task. 
				RF_message RF;
				RF.LF = LF_RX;
				RF.time_to_respond = time_now() + (rand() % 50);			/// divide the responses randomly into a 50ms response period
				RF.type = rf_LF_resp;

				// Send RF response with Exclusion data
				if((time_now() > Vision_Status.exclusion) || (LF_RX.VehicleID >= 0xFFE0))
				{
					// send a message to the RF process to send my LF response.
					if ((vision_settings.getActivities().LF_response))
						pipe_put(&cc1101.p, &RF);

					// send a master message reporting the LF.  
					if ((vision_settings.getActivities().forward_own_lf))
						forward_LF_report(LF_RX);
				}
			}
		}
	}
}

/**
 * this task periodically checks the RSSI values and updates the IO and LEDs.
 */
void LF_RSSI_watcher(task* t, int count, int* cant_sleep)
{
	static int count_last = 0;

	if (vision_settings.getActivities().output_critical != 0)
	{
#ifdef BUTTON_PIN
		if (GetButton())
			Apl_acknowledge_LF();
#endif 

		if (count != count_last)
		{

			// TODO: RF_Zone_Indication flag
			if(AS3933_LF_indication() == 0)
			{
				Apl_Checkzone_and_output();
				count_last = count;
			}
			//			else								// prevent buzzing outputs when LF is actively detecting.
			//				(*cant_sleep)++;
		}
	}
}

/**
 * this task periodically checks the RSSI values and updates the IO and LEDs.
 */
void Distress_watcher(task* t, int* cant_sleep)
{

#ifdef BUTTON_PIN
	if (GetTilt())
	{
		Vision_Status.Distress = 1;
	}
	else
	{
		Vision_Status.Distress = 0;
	}
#endif

	if(time_now() > t->pause_until)
	{
		uint8_t data[5];

		data[0] = rf_Distress;
		data[1] = Vision_Status.Distress;

		RF_message RF;
		RF.buff = buff_alloc(&data[1], 2, true);
		RF.len = 0;
		RF.type = rf_Distress;
		RF.time_to_respond = time_now();
		pipe_put(&cc1101.p, &RF);
		task_delay(t, 1500);
	}

}

//#define board_tester
#ifdef board_tester
uint8_t test_counter = 0;
#endif

#ifdef LF_TX_Capable
/**
 * @brief: this task is used to send TX messages over LF
 * @param pvParameters
 */

void LF_TX_Task(task* t)
{
	int queue_val = 0;

	if (t->state == 0)			// this is run the first time only. 
	{
		LF_form_packet(vision_settings.vehic_id, vision_settings.slave_id, true);
		if ((int) vision_settings.lfPeriod < 100)
			vision_settings.lfPeriod = 100;

		srand(Vision_Status.UID); // seed the random generator...
		t->state = 1;
		task_delay(t, 100);// make sure the LF doesn't transmit at the beginning 

		// create a pipe 
		pipe_init(&(t->p), 10, sizeof(int));
	}

	// LF is ready, so wait for either a sequence command or a timeout
	if (t->state == 1)
	{
		Clean_LF_Sync_list();
		bool forced = pipe_get(&t->p, &queue_val);
		if (forced || (time_now() > t->pause_until))
		{
			if ((vision_settings.getActivities().LF_TX) != 0)				// make sure the LF will be sent when forced, regardless of LF_TX enable.
			{
				Add_to_LF_Sync_list(vision_settings.slave_id._value);
				// create the LF TX packet.
				if(queue_val == 0) queue_val = (int)vision_settings.vehic_id;   // if the LF force commands specifies a VID, use it.
				LF_form_packet(queue_val, vision_settings.slave_id, true);

				// configure the LF PWM
				PWM_set_period(vision_settings.lf_power, vision_settings.lf_hertz);

				// start the LF transmitter interrupt driven process
				if((LF_trnasmitter_count<=1)||(!vision_settings.getActivities().CAN_sync))
				{
					if (vision_settings.getActivities().CAN_sync)
						send_LF_sync();
					LF_send();
					SetLed(&LED1, Violet, 50);
				}
				else
				{
					send_LF_sync();
					if(LF_CAN_Sync_List[0].SlaveID == (uint8_t)vision_settings.slave_id._value)
					{
						send_LF_sync_F(LF_CAN_Sync_List[LF_trnasmitter_Send].SlaveID);
					}

					if((forced)||(((LF_CAN_Sync_List[LF_trnasmitter_Send].SlaveID == (uint8_t)vision_settings.slave_id._value))&&(LF_CAN_Sync_List[0].SlaveID == (uint8_t)vision_settings.slave_id._value)))
					{
						LF_send();
						SetLed(&LED1, Violet, 50);
					}

				}
				if(LF_trnasmitter_Send<LF_trnasmitter_count-1)
				{
					LF_trnasmitter_Send++;
				}
				else
					LF_trnasmitter_Send = 0;


				// Schedule An RF message to be sent as soon as tags detecting this tag's LF wake (30ms from the start of LF).
				//send_LF_alert(30);
				// LF will be busy after this, tell it to wait
				t->state = 2;
				task_delay(t, (uint32_t)vision_settings.lfPeriod);

#ifdef board_tester
				switch (test_counter++)
				{
				case 0:
					SetLed(&LED1, Red, 200);
					break;
				case 1:
					SetLed(&LED1, Green, 200);
					break;
				case 2:
					SetLed(&LED1, Blue, 200);
					break;
				default:
					test_counter = 0;
				}

#endif
			}
		}
	}
	else
	{			// LF is busy, check if its done
		if (LF_Params.state == 0)
		{
			// LF TX is done, so start waiting for the next loop start
			t->state = 1;

			//			if (vision_settings.getActivities().CAN_sync)
			//				send_LF_sync();
		}
	}
}
#endif

#ifdef GFX

extern const IMAGE_INFO MERLogoINFO;
extern const IMAGE_INFO lightningeINFO;
extern const FONT_INFO _5x7MonospaceCE_11ptFontInfo;
extern const FONT_INFO _5x7MonospaceCE_6ptFontInfo;

void led_send_byte(uint8_t value)
{
	/*!< Loop while DR register in not empty */
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;

	/*!< Send byte through the SPI2 peripheral */
	SPI_I2S_SendData(SPI2, value);

}

void LED_Driver_Chip(RGB_State_TypeDef LED0, RGB_State_TypeDef LED1, RGB_State_TypeDef LED2, RGB_State_TypeDef LED3, uint8_t Intensity)
{
	uint8_t i;
	uint8_t Data[28];
	uint16_t count = 0;
	uint8_t TX_Buffer[28] =
	{	0};

	//Clear data
	for (i = 0; i < 28; i++)
		TX_Buffer[i] = 0;

	switch (LED0 & 0b00000111)
	{
	case Red:
		TX_Buffer[26] = 0;			//Blue
		TX_Buffer[24] = 0;//Green
		TX_Buffer[22] = Intensity;//Red
		break;
	case Green:
		TX_Buffer[26] = 0;
		TX_Buffer[24] = Intensity;
		TX_Buffer[22] = 0;
		break;
	case Blue:
		TX_Buffer[26] = Intensity;
		TX_Buffer[24] = 0;
		TX_Buffer[22] = 0;
		break;
	case Violet:
		TX_Buffer[26] = Intensity;
		TX_Buffer[24] = 0;
		TX_Buffer[22] = Intensity;
		break;
	case Yellow:
		TX_Buffer[26] = 0;
		TX_Buffer[24] = Intensity;
		TX_Buffer[22] = Intensity;
		break;
	case White:
		TX_Buffer[26] = Intensity;
		TX_Buffer[24] = Intensity;
		TX_Buffer[22] = Intensity;
		break;
	case Cyan:
		TX_Buffer[26] = Intensity;
		TX_Buffer[24] = Intensity;
		TX_Buffer[22] = 0;
		break;
	case Off:
		TX_Buffer[26] = 0;
		TX_Buffer[24] = 0;
		TX_Buffer[22] = 0;
		break;
	}

	switch (LED1 & 0b00000111)
	{
	case Red:
		TX_Buffer[20] = 0;			//Blue
		TX_Buffer[18] = 0;//Green
		TX_Buffer[16] = Intensity;//Red
		break;
	case Green:
		TX_Buffer[20] = 0;
		TX_Buffer[18] = Intensity;
		TX_Buffer[16] = 0;
		break;
	case Blue:
		TX_Buffer[20] = Intensity;
		TX_Buffer[18] = 0;
		TX_Buffer[16] = 0;
		break;
	case Violet:
		TX_Buffer[20] = Intensity;
		TX_Buffer[18] = 0;
		TX_Buffer[16] = Intensity;
		break;
	case Yellow:
		TX_Buffer[20] = 0;
		TX_Buffer[18] = Intensity;
		TX_Buffer[16] = Intensity;
		break;
	case White:
		TX_Buffer[20] = Intensity;
		TX_Buffer[18] = Intensity;
		TX_Buffer[16] = Intensity;
		break;
	case Cyan:
		TX_Buffer[20] = Intensity;
		TX_Buffer[18] = Intensity;
		TX_Buffer[16] = 0;
		break;
	case Off:
		TX_Buffer[20] = 0;
		TX_Buffer[18] = 0;
		TX_Buffer[16] = 0;
		break;
	}

	switch (LED2 & 0b00000111)
	{
	case Red:
		TX_Buffer[14] = 0;			//Blue
		TX_Buffer[12] = 0;//Green
		TX_Buffer[10] = Intensity;//Red
		break;
	case Green:
		TX_Buffer[14] = 0;
		TX_Buffer[12] = Intensity;
		TX_Buffer[10] = 0;
		break;
	case Blue:
		TX_Buffer[14] = Intensity;
		TX_Buffer[12] = 0;
		TX_Buffer[10] = 0;
		break;
	case Violet:
		TX_Buffer[14] = Intensity;
		TX_Buffer[12] = 0;
		TX_Buffer[10] = Intensity;
		break;
	case Yellow:
		TX_Buffer[14] = 0;
		TX_Buffer[12] = Intensity;
		TX_Buffer[10] = Intensity;
		break;
	case White:
		TX_Buffer[14] = Intensity;
		TX_Buffer[12] = Intensity;
		TX_Buffer[10] = Intensity;
		break;
	case Cyan:
		TX_Buffer[14] = Intensity;
		TX_Buffer[12] = Intensity;
		TX_Buffer[10] = 0;
		break;
	case Off:
		TX_Buffer[14] = 0;
		TX_Buffer[12] = 0;
		TX_Buffer[10] = 0;
		break;
	}

	switch (LED3 & 0b00000111)
	{
	case Red:
		TX_Buffer[8] = 0;			//Blue
		TX_Buffer[6] = 0;//Green
		TX_Buffer[4] = Intensity;//Red
		break;
	case Green:
		TX_Buffer[8] = 0;
		TX_Buffer[6] = Intensity;
		TX_Buffer[4] = 0;
		break;
	case Blue:
		TX_Buffer[8] = Intensity;
		TX_Buffer[6] = 0;
		TX_Buffer[4] = 0;
		break;
	case Violet:
		TX_Buffer[8] = Intensity;
		TX_Buffer[6] = 0;
		TX_Buffer[4] = Intensity;
		break;
	case Yellow:
		TX_Buffer[8] = 0;
		TX_Buffer[6] = Intensity;
		TX_Buffer[4] = Intensity;
		break;
	case White:
		TX_Buffer[8] = Intensity;
		TX_Buffer[6] = Intensity;
		TX_Buffer[4] = Intensity;
		break;
	case Cyan:
		TX_Buffer[8] = Intensity;
		TX_Buffer[6] = Intensity;
		TX_Buffer[4] = 0;
		break;
	case Off:
		TX_Buffer[8] = 0;
		TX_Buffer[6] = 0;
		TX_Buffer[4] = 0;
		break;
	}

	TX_Buffer[3] = 0b10111111;
	TX_Buffer[2] = 0b11011111;
	TX_Buffer[1] = 0b01001111;
	TX_Buffer[0] = 0b10010110;

	for (count = 0; count < 28; count++)
	{
		Data[count] = TX_Buffer[count];
	}

	//Send data over SPI Bus
	for (count = 0; count < 28; count++)
	{
		led_send_byte(Data[count]);
	}
}

void Test_task5(task* t)
{
	static zone current_zone, zone_last;
	int holder;

	if (t->state == 0)			// this is run the first time only. 
	{

		setRotation(2);
		SSD1306_begin(SSD1306_SWITCHCAPVCC, 0);
		display();
		Delay(100);
		setTextColorBG(WHITE, BLACK);

		Setfont(&_5x7MonospaceCE_6ptFontInfo);

		// create a pipe to hold RSSI values we need to catch. 
		pipe_init(&(t->p), 10, sizeof(int));
		t->state = 1;
	}

	////////////////////////////////////////////////////////////////
	else
	{
		holder = -1;
		if (pipe_get(&t->p, &holder) || (time_now() > t->pause_until))
		{
			if(holder != -1)
				holder &= 0xFF;		// crop to one byte.

			current_zone = (zone) holder;

			if (true)//current_zone != zone_last)
			{
				clearDisplay();

				/*switch (current_zone)
				 {
				 case zone_none:
				 //				setCursor(4, 25);
				 //				print_TextXY("PULSE 110");
				 //				setCursor(4, 45);
				 //				print_TextXY("Demo");
				 //				LED_Driver_Chip(Off, Off, Off, Off, 127);
				 //				setCursor(4, 25);
				 //				print_TextXY("PULSE 110");
				 //				setCursor(4, 45);
				 //				print_TextXY("Demo");
				 //				LED_Driver_Chip(Off, Off, Off, Off, 127);

				 v = 0;
				 break;
				 case zone_pres:
				 //			setCursor(4, 25);
				 //			print_TextXY("Vehicle");
				 //			setCursor(4, 45);
				 //			print_TextXY("Warning");
				 //			fillRoundRect(120, 40, 7, 20, 0, WHITE);
				 LED_Driver_Chip(Blue, Blue, Blue, Blue, 127);

				 if (v < 1)
				 {
				 Timer6_Restart();
				 GPIO_SetBits(GPIOC, GPIO_Pin_10);
				 Delay(200);
				 GPIO_ResetBits(GPIOC, GPIO_Pin_10);
				 Timer6_Stop();
				 }
				 v = 1;
				 break;
				 case zone_warn:
				 //			setCursor(4, 25);
				 //			print_TextXY("Vehicle");
				 //			setCursor(4, 45);
				 //			print_TextXY("Caution");
				 //			fillRoundRect(120, 20, 7, 40, 0, WHITE);
				 LED_Driver_Chip(Yellow, Yellow, Yellow, Yellow, 70);

				 if (v < 2)
				 {
				 Timer6_Restart();
				 GPIO_SetBits(GPIOC, GPIO_Pin_10);		//Vibrate
				 Delay(200);
				 Timer6_Stop();
				 Delay(100);
				 Timer6_Restart();
				 Delay(200);
				 GPIO_ResetBits(GPIOC, GPIO_Pin_10);
				 Timer6_Stop();
				 }
				 v = 2;

				 break;

				 case zone_crit:
				 //			setCursor(4, 25);
				 //			print_TextXY("Vehicle");
				 //			setCursor(4, 45);
				 //			print_TextXY("Critical");
				 //			fillRoundRect(120, 0, 7, 60, 0, WHITE);
				 LED_Driver_Chip(Red, Red, Red, Red, 127);

				 if (v < 3)
				 {
				 Timer6_Restart();
				 GPIO_SetBits(GPIOC, GPIO_Pin_10);		//Vibrate
				 Delay(800);
				 GPIO_ResetBits(GPIOC, GPIO_Pin_10);
				 Timer6_Stop();
				 }
				 v = 3;
				 break;
				 default:
				 v = 0;
				 break;
				 }*/

				//				if (as3933TuneResults[0].resonanceFrequencyTuned != 0)
				//				{
				//					setCursor(0, 0);
				//					sprintf(pcWriteBuffer, "X: %d -> %dHz\n", as3933TuneResults[0].resonanceFrequencyOrig, as3933TuneResults[0].resonanceFrequencyTuned);
				//					print_TextXY(pcWriteBuffer);
				//					sprintf(pcWriteBuffer, "Y: %d -> %dHz\n", as3933TuneResults[1].resonanceFrequencyOrig, as3933TuneResults[1].resonanceFrequencyTuned);
				//					print_TextXY(pcWriteBuffer);
				//					sprintf(pcWriteBuffer, "Z: %d -> %dHz\n", as3933TuneResults[2].resonanceFrequencyOrig, as3933TuneResults[2].resonanceFrequencyTuned);
				//					print_TextXY(pcWriteBuffer);
				//
				//					sprintf(pcWriteBuffer, "X: %ddB\n", rssiX);
				//					print_TextXY(pcWriteBuffer);
				//					sprintf(pcWriteBuffer, "Y: %ddB\n", rssiY);
				//					print_TextXY(pcWriteBuffer);
				//					sprintf(pcWriteBuffer, "Z: %ddB\n", rssiZ);
				//					print_TextXY(pcWriteBuffer);
				//				}
				//				else
				//				{
				setCursor(0, 0);
				if (holder == -1)
					sprintf(pcWriteBuffer, "none\n");
				else
					sprintf(pcWriteBuffer, "RX: %ddB\n", holder);

				print_TextXY(pcWriteBuffer);
				//				}

				display();
			}
			zone_last = current_zone;
			task_delay(t, 1000);
		}
	}
}

//void Test_task6(task* t)
//{
//	Delay(2000);
//	uint8_t i = 0, k = 0;
//	while (1)
//	{
//		Read_Inputs();
//		if ((k == 0) && (Input_Flags.CHRG_PG == 1))
//		{
//			Timer6_Restart();
//			Delay(100);
//			Timer6_Stop();
//			Delay(100);
//			Timer6_Restart();
//			Delay(100);
//			Timer6_Stop();
//			k = 1;
//		}
//		else if (Input_Flags.CHRG_PG == 0)
//			k = 0;
//
//		xSemaphoreTake(SPI_keeper, portMAX_DELAY);
//
//		if ((Input_Flags.CHRG_PG == 1))
//		{
//			if (i == 0)
//			{
//				fillRoundRect(5, 5, 15, 15, 6, WHITE);
//				drawImage(&lightningeINFO, 5, 5, BLACK);
//				i++;
//			}
//			else if (i == 1)
//			{
//				fillRoundRect(5, 5, 15, 15, 6, BLACK);
//				drawImage(&lightningeINFO, 5, 5, WHITE);
//				i = 0;
//			}
//		}
//		else
//			fillRoundRect(5, 5, 15, 15, 0, BLACK);
//		display();
//		xSemaphoreGive(SPI_keeper);   // we're done with the SPI for now.
//
//		Delay(2000);
//	}
//}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
