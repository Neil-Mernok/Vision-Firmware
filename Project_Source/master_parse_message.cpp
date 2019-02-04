/*
 * master_parse_message.cpp
 *
 *  Created on: Jul 18, 2016
 *      Author: Kobus
 */


#include "master_interface.h"
#include "about.h"
#include "as3933.h"

#include "RF_APL.h"
#include "GPS_APL.h"
#include "Time_APL.h"

uint8_t to_master[150];

/**
 * @brief This is the main 
 * @param MIF
 * @return
 */

uint8_t MIF_Parse(_Q_MasterIF MIF)
{
	uint8_t message_length = 0 ;
	uint8_t message_data[100] = { 0 };

	if (MIF.Master == MUSB)
		Vision_Status.sts.USB_Working = true;
	else if (MIF.Master == COM)
		Vision_Status.sts.Uart_Working = true;

	for (uint16_t MIF_index = 0; MIF_index < MIF.len; MIF_index++)
	{
		if (check_frame(&MIF.data[MIF_index]))
		{
			message_length = MIF.data[MIF_index + 2] ;
			memcpy(message_data, MIF.data + MIF_index + 3, message_length);

			if (parse_message(message_data, message_length, MIF.Master))
			{
				MIF_index += message_length + 5 ;
			}
		}
	}
	return 1;
}
uint8_t Zone_Time_Now;
zone Recieved_Zone = zone_none;
Zone_Alert_Type Received_Alert;
extern Zone_Info_Type RF_Zone;
/**
 * @brief This is the main
 * @param MIF
 * @return
 */
int parse_message(uint8_t parse_data[], uint8_t parse_length, Master_source MIF_Source)
{
	int send_mess = 0, command_valid = 0;
	int i;

	_Q_MasterIF Push_MIF;

	uint16_t dist;
	_Transpondert* t;

	switch (parse_data[0])
	{

	case 'B':	// ----	Force device into boot mode (command) ----
	{
		DelayUs(5000);		// wait for the USB to acknowledge the receipt before powering down. otherwise it stalls on PC side

#ifdef USE_HAL_DRIVER
		HAL_RTCEx_BKUPWrite(&hrtc, boot_reason_loc, Boot_reason_program);
		//PowerOff();
		DelayUs(5000);// wait for the USB again...

		i = HAL_RTCEx_BKUPRead(&hrtc, boot_reason_loc);
		NVIC_SystemReset();
#else
		BKP_WriteBackupRegister(boot_reason_loc, Boot_reason_program);
		//PowerOff();
		DelayUs(5000);				// wait for the USB again...
		IWDG_ReloadCounter();
		DelayUs(5000);
		i = BKP_ReadBackupRegister(boot_reason_loc);
		NVIC_SystemReset();
#endif
	}
	break;

	case 'C': 	// ----	Device's set setting - index, setting and size reply (command) ----
	case 'c': 	// ---- Device's set setting - size reply (command) ----
		// ---- (adr<2>)(data<4+>) more bytes are only acceptable in the case of strings. ----
	{
		int index = *(uint16_t*) (&parse_data[1]);

		GPIO_SetBits(VRFID_PORT, VRFID_PIN);		// ---- Power the EEPROM ----
#ifdef USE_HAL_DRIVER
		MX_I2C1_Init(400000);						// ---- If the RFID device was unpowered, we need to reset the I2C ----
		DelayUs(20);
#endif

		if (vision_settings[index])
		{
			int res = SaveSetting(index, &parse_data[3], parse_length - 3);
			// send response
			if(res && parse_data[0] == 'c')
			{
				command_valid = 1;
				send_mess = 2;

				to_master[0] = 'c';
				to_master[1] = vision_settings[index]->len();
			}
			else if(res && parse_data[0] == 'C')
			{
				// ---- Send the setting value back ----
				// ---- Send 5 bytes, or more for a string ----
				int len = MAX(4, vision_settings[index]->len());
				memset(&to_master[4], 0, len + 1);
				memcpy(&to_master[4], vision_settings[index]->ptr(), len);

				/// ---- Returned message is the size of the data plus 4. one for 's', 2 for index, one for type/byte count ----
				command_valid = 1;
				send_mess = len + 4;
				to_master[0] = 'C';
				to_master[1] = parse_data[1];	// ---- Index0 ----
				to_master[2] = parse_data[2];	// ---- Index1 ----
				to_master[3] = vision_settings[index]->len();
			}
			else
			{
				command_valid = 1;
				send_mess = 2;

				to_master[0] = parse_data[0];
				to_master[1] = 0xFF;		// ---- Indicates invalid command or failure to save ----
			}
		}
		else if (index == 0xFFFF)			// ---- This is a 'reset to default' command for activities ----
		{
			command_valid = 1;
			send_mess = 2;

			to_master[0] = parse_data[0];

			switch (parse_data[3])
			{
			case 1:
				vision_settings.set_defaults(ME_PCB_173_03);
				break;
			case 2:
				vision_settings.set_defaults(vision_reader);
				break;
			case 3:
				vision_settings.set_defaults(pulse300);
				break;
			case 4:
				vision_settings.set_defaults(pulse500);
				break;
			case 5:
				vision_settings.set_defaults(ME_PCB_138_03);
				break;
			case 6:
				vision_settings.set_defaults(ME_PCB_217_02);
				break;
			default:
				to_master[1] = 0;
				break;
			}
			if (to_master[1] != 0)
			{
				// Use the LSB of the UID for the slave ID by default.
				vision_settings.slave_id = (uint8_t)Vision_Status.UID;
				if(SaveSettings() != 0)
				{
					to_master[1] = 1;		// Indicates success
				}
				else
				{
					to_master[1] = 255;		// Indicates invalid command or failure to save.
				}
			}
		}
		else /// This is a command error, return that we accept it was a setting, but address is bad
		{
			send_mess = 2;
			to_master[0] = 'c';
			to_master[1] = 0;
		}
		Vision_Status.Setting_changed = time_now();		// Indicate that some parameter has changed
	}
	break;

	case 'D': 	// ----	Force data onto RF (command) ----
	{
		command_valid = 1;

		RF_message RF;
		RF.buff = buff_alloc(parse_data, parse_length, true);
		RF.len = parse_length;
		RF.type = rf_Data;
		RF.time_to_respond = time_now();
		pipe_put(&cc1101.p, &RF);
	}
	break;

	case 'F':	// ----	Device's status word (request) ----
	{
		int index = *(uint16_t*) (&parse_data[1]);

		if(index == 9)			// ---- Check that the address points to a setting ----
		{
			int len = 1;

			/// returned message is the size of the data plus 4. one for 'F', 2 for index, one for type/byte count.
			command_valid = 1;
			send_mess = len + 4;

			to_master[0] = 'F';
			to_master[1] = parse_data[1];		// ---- Index0 ----
			to_master[2] = parse_data[2];		// ---- Index1 ----
			to_master[3] = len;
			to_master[4] = Vision_Status.board_id;
		}
		else if ((index >= 10) && (index <= 12))
		{
			int len = 4;

			/// returned message is the size of the data plus 4. one for 'F', 2 for index, one for type/byte count.
			command_valid = 1;
			send_mess = len + 4;

			to_master[0] = 'F';
			to_master[1] = parse_data[1];		// index0
			to_master[2] = parse_data[2];		// index1
			to_master[3] = len;
			memcpy(&to_master[4], &(as3933TuneResults[index - 10].resonanceFrequencyTuned), len);
		}
		else /// this is a command error. return that we accept it was a setting, but address is bad.
		{
			send_mess = 2;

			to_master[0] = 'F';
			to_master[1] = 0;
		}
	}
	break;

	case 'f':	// ----	Device's status word (request) ----
	{
		to_master[0] = 'f';
		command_valid = 1;
		send_mess = 9;
		memcpy(&to_master[1], &(Vision_Status.sts.Word), sizeof(uint32_t));

		if(MIF_Source == COM)
		{
			Vision_Status.Last_COM = time_now();
		}
	}
	break;

	case 'G':	// ----	Device coordinates (request) ----
	{
		command_valid = 1;
		send_mess = 54;

		to_master[0] = 'G';
		memcpy(to_master + 1, &Vision_Status.GPS_Data.Longitude, 4);
		memcpy(to_master + 5, &Vision_Status.GPS_Data.Latitude, 4);
		memcpy(to_master + 9, &Vision_Status.GPS_Data.HorizontalAccuracy, 4);
		memcpy(to_master + 13, &Vision_Status.GPS_Data.VerticalAccuracy, 4);
		to_master[17] = Vision_Status.GPS_Data.FixType;
		to_master[18] = Vision_Status.GPS_Data.NumberOfSat;
		memcpy(to_master + 19, &Vision_Status.GPS_Data.Speed, 4);
		memcpy(to_master + 23, &Vision_Status.GPS_Data.SpeedAccuracy, 4);
		memcpy(to_master + 27, &Vision_Status.GPS_Data.HeadingVehicle, 4);
		memcpy(to_master + 31, &Vision_Status.GPS_Data.HeadingMotion, 4);
		memcpy(to_master + 35, &Vision_Status.GPS_Data.HeadingAccuracy, 4);
		memcpy(to_master + 39, &Vision_Status.GPS_Data.SeaLevel, 4);
		memcpy(to_master + 43, &Vision_Status.GPS_Data.flags, 2);
		memcpy(to_master + 45, &Vision_Status.GPS_Data._Date, 4);
		memcpy(to_master + 49, &Vision_Status.GPS_Data._Time, 4);
		to_master[53] = Vision_Status.GPS_Data.FixAge;
	}
	break;

	case 'g':	// ----	Coordinate share message (command) ----
	{
		command_valid = 1;
		send_mess = 1;

		to_master[0] = 'g';
		memcpy(&Vision_Status.GPS_Data.Longitude, parse_data + 1, 4);
		memcpy(&Vision_Status.GPS_Data.Latitude, parse_data + 5, 4);
		memcpy(&Vision_Status.GPS_Data.HorizontalAccuracy, parse_data + 9, 4);
		memcpy(&Vision_Status.GPS_Data.VerticalAccuracy, parse_data + 13, 4);
		memcpy(&Vision_Status.GPS_Data.FixType, parse_data + 17, 1);
		memcpy(&Vision_Status.GPS_Data.Speed, parse_data + 18, 4);
		memcpy(&Vision_Status.GPS_Data.SpeedAccuracy, parse_data + 22, 4);
		memcpy(&Vision_Status.GPS_Data.HeadingVehicle, parse_data + 26, 4);
		memcpy(&Vision_Status.GPS_Data.HeadingMotion, parse_data + 30, 4);
		memcpy(&Vision_Status.GPS_Data.HeadingAccuracy, parse_data + 34, 4);
		memcpy(&Vision_Status.GPS_Data.SeaLevel,parse_data + 38, 4);
		memcpy(&Vision_Status.GPS_Data.FixAge, parse_data + 42, 1);
	}
	break;

	case 'I':	//	---- Force RF command. Tells the RF chip to send a PDS message with a specific type ----
	case 'i':
	{
		command_valid = 1;

		if (parse_length == 2)
		{
			RF_message RF;
			RF.buff = buff_alloc(&parse_data[1], parse_length, true);
			RF.len = 0;
			RF.type = (parse_data[0] == 'i') ? rf_legcPDS : rf_ID_puls;
			RF.time_to_respond = time_now();
			pipe_put(&cc1101.p, &RF);
		}
		else if (parse_length <= 22)
		{
			RF_message RF;
			RF.buff = buff_alloc(&parse_data[1], parse_length, true);
			RF.buff[parse_length - 1] = 0;		// make sure the string is null terminated.
			RF.len = parse_length - 2;
			RF.type = rf_ID_name;
			RF.time_to_respond = time_now();
			pipe_put(&cc1101.p, &RF);
		}
	}
	break;

	case 'k':	// ----	Device's transponder log count : dist, age, type (request) ----
	{
		i = 0;
		dist = *(uint16_t*) (&parse_data[1]);
		while ((t = transp_filter(i, dist, parse_data[3], parse_data[4], 0, (_kind)(0) )) != NULL)
		{
			i++;
		}
		command_valid = 1;
		send_mess = 2;

		to_master[0] = 'k';
		to_master[1] = i;
	}
	break;

	case 'l':	// ----	Device's transponder log count : dist, age, type (request) ----
	{
		uint32_t LF_TransponderUID = 0 ;
		LF_TransponderUID = *((uint32_t*) (&parse_data[2]));
		if (LF_TransponderUID != Vision_Status.UID)
		{
			_Transpondert* LF_Transponder;

			LF_Transponder = Transp_RF_handler(parse_data, 60, parse_length);
			if (vision_settings.getActivities().forward_RF)
				Send_POD_toMaster(LF_Transponder, CODE, 2);

			Zone_Info_Type RF_Transponder;
			RF_Transponder.Technology = LF_Technology;
			RF_Transponder.Zone = zone_from_rssi(LF_Transponder->LF.RSSI);
			RF_Transponder.VehicleID = LF_Transponder->VehicleID;
			RF_Transponder.SlaveID = LF_Transponder->SlaveID;
			RF_Transponder.Last_Seen = time_now();
			Apl_Compare_Zone_and_Update(RF_Transponder);
		}
	}
	break;

	case 'L':	//  ---- This is a LF sync/force message. tells the system, that
		// 		 another device has output LF or that we should output
	{
		command_valid = 1;
		if ((uint8_t)vision_settings.slave_id._value == parse_data[1])
		{
			if (parse_length == 4)
				i = *(uint16_t*) (&parse_data[2]);	// in the extended message, grab the VID to be sent.
			else if (parse_length == 2)
			{
				i = (uint16_t)vision_settings.vehic_id._value;
			}
			else
				i = 0;
			pipe_put(&LF_TX.p, &i);					// this is a forced LF tx to my ID.
		}
		else
		{
			Vision_Status.LastLF_TX = time_now();	// this is an indication of some other LF device.
			Add_to_LF_Sync_list(parse_data[1]);
		}
	}
	break;

	case 'M':	// --- Master message to be sent to RF process (command) ----
	{
		command_valid = 1;
		RF_message RF;
		RF.buff = buff_alloc(parse_data, parse_length, true);
		RF.len = parse_length;
		RF.type = rf_MasterP;
		RF.time_to_respond = time_now();
		pipe_put(&cc1101.p, &RF);
	}
	break;

	case 'o':	// ----	ODO meter reading (request) ----
	{
		command_valid = 1;
		send_mess = 17;

		to_master[0] = 'o';
		memcpy(to_master + 1, &Vision_Status.GPS_Data.Speed, 4);
		memcpy(to_master + 2, &Vision_Status.GPS_Data.SpeedAccuracy, 4);
		memcpy(to_master + 9, &Vision_Status.GPS_Data.Distance, 4);
		memcpy(to_master + 13, &Vision_Status.GPS_Data.TotalDistance, 4);
	}
	break;

	case 'P':	// ----	Device's transponder log count : dist, age, type (CAN request) ----
	case 'p':	// ----	Device's transponder log count : dist, age, type (request) ----
	{
		i = 0;
		dist = *(uint16_t*) (&parse_data[1]);
		while ((t = transp_filter(i, dist, parse_data[3], parse_data[4], 0, (_kind)parse_data[5])) != NULL)
		{
			Send_POD_toMaster(t, MIF_Source, parse_data[0]);
			i++;
		}
		command_valid = 1;
	}
	break;

	case 'q':	// ----	GPS antenna status (request) ----
	{
		command_valid = 1;
		send_mess = 2;

		to_master[0] = 'q';
		to_master[1] = Vision_Status.GPS_Data.antenna_status;
	}
	break;

	case 'r':	// ---- Software device reset (command) ----
	{
		send_mess = 1;
		command_valid = 1;

		to_master[0] = 'r';
		NVIC_SystemReset();
	}
	break;

	case '@':	// ----	Vehicle information message (command) ----
	{
		command_valid = 1;
		send_mess = 1;

		to_master[0] = 'R';
		Vision_Status.Reverse = parse_data[1];
		Vision_Status.Stopping_dist = parse_data[2];
		Vision_Status.Speed = *(uint32_t*) (&parse_data[3]);
	}
	break;

	case 's':	// ----	Device's setting (request) ----
	{
		int index = *(uint16_t*) (&parse_data[1]);

		// check that the address points to a setting
		if (vision_settings[index])
		{
			// send 4 bytes, or more for a string
			int len = MAX(4, vision_settings[index]->len());
			memset(&to_master[4], 0, len + 1);
			memcpy(&to_master[4], vision_settings[index]->ptr(), len);

			/// returned message is the size of the data plus 4. one for 's', 2 for index, one for type/byte count.
			command_valid = 1;
			send_mess = len + 4;
			to_master[0] = 's';
			to_master[1] = parse_data[1];		// index0
			to_master[2] = parse_data[2];		// index1
			to_master[3] = vision_settings[index]->len();
		}
		else /// this is a command error. return that we accept it was a setting, but address is bad.
		{
			command_valid = 1;
			send_mess = 2;
			to_master[0] = 's';
			to_master[1] = 0;
		}
	}
	break;

	//	filter type for ranging => send a bunch of bytes, all will be set to true in the filter.
	/// TODO: Revise type filter to possibly include vision type-> PDS vs Pulse... or check filter function for LF dBm...
	case 't':
	{
		command_valid = 1;
		send_mess = 1;
		for (i = 1; i < parse_length; i++)
		{
			transp_type_add(parse_data[i], true);
		}

		to_master[0] = 't';

	}
	break;

	case 'u':	// ----	Device's UID (request) ----
	{
		command_valid = 1;
		send_mess = 6;

		to_master[0] = 'u';
		to_master[1] = Vision_Status.kind;
		memcpy(&to_master[2], &Vision_Status.UID, 4);
	}
	break;

	case 'v': 	// ----	Device's firmware revision (request) ----
	{
		command_valid = 1;
		send_mess = 3;

		to_master[0] = 'v';
		to_master[1] = Firmware_rev;
		to_master[2] = Firmware_subrev;
	}
	break;

	case 'w':	// ---- Device string identification - Who Am I (request) ----
	{
		command_valid = 1;
		memset(to_master, 0, sizeof(to_master));  // clear the buffer.
		char* whoAmI = (char*)to_master;
		strcpy(whoAmI, "w");
		if(Vision_Status.board_id == ME_PCB_138_03)
			strcat(whoAmI, "RANGING, ");
		else if ((Vision_Status.board_id == ME_PCB_217_01) || (Vision_Status.board_id == ME_PCB_217_02))
			strcat(whoAmI, "GPS, ");
		else
			strcat(whoAmI, "PULSE, ");
		strcat(whoAmI, Firmware_rev_str);
#ifdef STM32L1
		strcat(whoAmI, ", Mantag, LP, ");
#else
#ifdef STM32L4
		strcat(whoAmI, ", Module, LP, ");
#else
		strcat(whoAmI, ", Module, HP, ");
#endif
#endif
		switch (Vision_Status.board_id)
		{
		case ME_PCB_173_03:
			strcat(whoAmI, "PCB-173-03, ");
			break;
		case ME_PCB_173_04:
			strcat(whoAmI, "PCB-173-04, ");
			break;
		case ME_PCB_203_01:
			strcat(whoAmI, "PCB-203-01, ");
			break;
		case ME_PCB_182_03:
			strcat(whoAmI, "PCB-182-03, ");
			break;
		case ME_PCB_182_04:
			strcat(whoAmI, "PCB-182-04, ");
			break;
		case ME_PCB_182_06:
			strcat(whoAmI, "PCB-182-06, ");
			break;
		case ME_PCB_138_03:
			strcat(whoAmI, "PCB-138-03, ");
			break;
		case ME_PCB_217_01:
			strcat(whoAmI, "PCB-217-01, ");
			break;
		case ME_PCB_217_02:
			strcat(whoAmI, "PCB-217-02, ");
			break;
		case ME_PCB_203_02:
			strcat(whoAmI, "PCB-203-02, ");
			break;
		case ME_PCB_203_04:
			strcat(whoAmI, "PCB-203-04, ");
			break;
		default:
			break;
		}
		strcat(whoAmI, compile_date);
		send_mess = strnlen(whoAmI, 50);
	}
	break;

	case 'x':	// ----	Reset the device's transponder log (command) ----
	{
		command_valid = 1;
		send_mess = 1;

		to_master[0] = 'x';
		clear_transp_log();
	}
	break;

	case 'y':	// ----	Reset ODO meter (command) ----
	{
		command_valid = 1;
		send_mess = 1;

		to_master[0] = 'y';
#ifdef GPS_MODULE
		UBX_Reset_Odometer();
#endif
	}
	break;

	case 'Z':	//  ---- Master boot message to be sent to RF process ----
	{
		//int leni = parse_length, leno = 0;
		int leno = 0;
		RF_message RF;

		command_valid = 1;
		RF.type = rf_Boot_M;
		RF.time_to_respond = time_now();

		// split the message into <62 byte RF messages and send all of them.
		//					while(leni > 62)
		//					{
		//						RF.buff = buff_alloc(&parse_data[leno + 1], 62, true);
		//						RF.len = 61;
		//						pipe_put(&cc1101.p, &RF);
		//						leno += 62;
		//						leni -= 62;
		//					}

		RF.buff = buff_alloc(&parse_data[leno + 1], parse_length - leno, true);
		RF.len = parse_length-leno-1;
		pipe_put(&cc1101.p, &RF);
	}
	break;

	case 'z':	//	---- Force RF zone message (command) -----
	{
		command_valid = 1;

		// ---- Parse and handle zone message ----
		RF_Zone.Zone = (zone)parse_data[1];
		RF_Zone.VehicleID = *(uint32_t*) &parse_data[2];
		RF_Zone.SlaveID = parse_data[6];
		RF_Zone.Last_Seen = time_now();
		RF_Zone.Technology = RF_Technology;
		Apl_Compare_Zone_and_Update(RF_Zone);
	}
	break;

	case '#': //	---- Force RF time  message (command) -----
	{
		command_valid = 1;
		if(parse_length>1)
		{
			Set_RTCDateTime(&parse_data[1]);
//			Vision_Status.DateTime.Seconds = parse_data[1];
//			Vision_Status.DateTime.Minutes = parse_data[2];
//			Vision_Status.DateTime.Hours = parse_data[3];
//			Vision_Status.DateTime.Date = parse_data[4];
//			Vision_Status.DateTime.Month = parse_data[5];
//			Vision_Status.DateTime.Year = parse_data[6];
		}


		send_mess = 2;
		to_master[0] = '#';
		to_master[1] =  send_mess;

	}
	break;

	default:
		command_valid = 0;
		break;

	}

	if(command_valid)
	{
		if (MIF_Source != RF)
		{
			Master_last = MIF_Source;    // indicate that out last master was this source.
		}
		Vision_Status.last_master_coms = time_now();
		//todo: Neil
		if(MIF_Source == MCAN)
		{
			Vision_Status.Last_CAN = time_now();
		}
	}

	if (send_mess)
	{
		Push_MIF.Master = MIF_Source;
		Push_MIF.data = to_master;
		Push_MIF.len = send_mess;
		push_to_master(Push_MIF);
	}

	return command_valid;
}

