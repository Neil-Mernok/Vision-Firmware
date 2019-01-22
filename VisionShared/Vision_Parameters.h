/*
 * vision_settings.h
 *
 *  Created on: 7 May, 2015
 *      Author: Kobus Goosen
 */

#ifndef VISION_PARAMS_H_
#define VISION_PARAMS_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ParamValue.h" 
#include "LF_APL.h"
#include "GPS_APL.h"
#include "Time_APL.h"
#include "TagTypes/TagTypes.h"

#define Firmware_rev 14
#define Firmware_subrev 0
#define Firmware_rev_str "V14"

/// locations of the vision parameters. this should only be added to, even if the values are no longer in use. 
enum vision_param_adr
{
	padr_firmware = 0x08,      	// 03 - byte indicating the firmware revision. used as sanity check
	padr_tag_type = 0x0c,      	// 04 - byte specifying the tag type.
	padr_rf_power = 0x10,      	// 05 - byte specifying RF output power. 0-7.
	padr_slave_id = 0x14,		// 06 - byte CAN ID.only lower 4 bits are used for pod_ID in the LF tx message
	padr_vehic_id = 0x18,		// 07 - short vehicle ID.
	padr_uartBaud = 0x1c,		// 08 - int uart baud in hz
	padr_can_baud = 0x20,		// 09 - int baud rate in hz
	padr_interval = 0x24,		// 10 - int RF transmission interval (ms)
	padr_activity = 0x28,      	// 11 - binary values indicating the tag's activities: see _Transp_activities.
	padr_rf_chanl = 0x2c,		// 12 - byte adjusts the RF channel used.
	padr_lfPeriod = 0x30,		// 13 - int LF_TX interval to send LF packets.
	padr_lf_power = 0x34,      	// 14 - byte allowing LF power to be adjusted. 0-100
	padr_lf_hertz = 0x38,      	// 15 - allow the device to set the LF frequency. in hz
	padr_max_dist = 0x3c,      	// 16 - int max dist to pass to master. not used by Pulse
	padr_name_str = 0x40,		// 17 - string max 20 byte string name of the device.
	padr_ack_time = 0x54,		// 22 - short time (ms) to stop sending warning after ack button is pressed (mantag)
	padr_ack_intv = 0x58,		// 23 - byte interval between LF warnings once acknowledged
	padr_lf_filtr = 0x5c,		// 24 - int time constant of LF zone max rssi filter.
	padr_vchrgmin = 0x60,		// 25 - int minimum source voltage from which to charge.
	padr_rssiCrit = 0x64,		// 26 - byte LF rssi critical Threshold.
	padr_rssiWarn = 0x68,		// 27 - byte LF rssi warning Threshold.
	padr_rssiPres = 0x6c,		// 28 - byte LF presence threshold.
	padr_usrParam = 0x70,		// 29 - byte user specified parameter.
	padr_AntOfset = 0x74,		// 30 - byte antenna cable length offset parameter. used to calibrate the range received.
	padr_firm_sub = 0x78,		// 31 - byte indicating the firmware sub revision
	padr_CAN_heartbeat_time = 0x7c, //32 - byte for CAN heatbeat monitoring timeout
	padr_CAN_revert = 0x80,      	//33 - byte to change tag type when heartbeat monitoring times out
	padr_Asset_list_rev = 0x84,     //34 - byte used to show the version of the mernok asset file last read
	padr_length 	= 0x88,			//35- byte used to set the vehicle length
	padr_Width  	= 0x8C,			//36- byte used to set the vehicle width
	padr_product	= 0x90			//37- byte used to set the product type
};



//// settings that need special updating:
	// CAN baud,	(re-initise CAN system.) 
	// RF Power. 	(re-initialise CC chip)
	// UART baud	(re-initialise Uart)
	// LF hertz. 	(re-initialise LF receiver. not a good idea to change, as it needs hardware change too. 
//// settings that require range testing to ensure proper operation:
	// RF power (0-7)
	// uart baud	9600	-	1000000
	// CAN baud		10000 	-	500000
	// interval		250		-	120000
	// lf period	250		-	120000
	// lf power		10		-	100
	// lf hertz		50000	-	150000
	// lf filter  	500		-	60000
	// vchargemin	3000	-	20000

enum P_uint
{
	P_string = 0,
	P_uint8_t = 1,
	P_uint16_t = 2,
	P_uint32_t = 3
};

enum boards
{
	ME_PCB_182_01,
	ME_PCB_182_02,
	ME_PCB_182_03,
	ME_PCB_173_01,
	ME_PCB_173_02,
	ME_PCB_173_03,
	vision_reader,
	pulse300,
	pulse500,
	ME_PCB_182_04,
	ME_PCB_173_04,
	can_PDS_reader,
	ME_PCB_203_01,
	ME_PCB_138_03,
	ME_PCB_182_05,
	// ---- V12 - GPS Functionality ----
	ME_PCB_217_01,
	ME_PCB_203_02,
	ME_PCB_203_04,
	ME_PCB_182_06,
	ME_PCB_217_02,
};

typedef union StatusWord
{
	uint32_t Word;
	struct
	{
		uint32_t CC_SPI_working :1;
		uint32_t LF_SPI_working :1;
		uint32_t LF_tuned :1;
		uint32_t ACC_SPI_working :1;
		uint32_t RFID_Working :1;
		uint32_t RF_Working :1;
		uint32_t Uart_Working :1;
		uint32_t crystal_working :1;
		uint32_t USB_Working :1;
		uint32_t CAN_Working :1;
		uint32_t Charging :1;
		uint32_t EXT_Power :1;
		uint32_t low_power_mode :1;
		uint32_t USB_Active :1;
		uint32_t Antenna_connected :1;
		uint32_t Sleeping :1;
		uint32_t UART_capable :1;
		uint32_t USB_capable :1;
		uint32_t CAN_capable :1;
		uint32_t RF_Receiving :1;
		uint32_t NN_SPI_working :1;
		// ---- V12 Status Flags ----
		uint32_t Module_UART_working :1;
		uint32_t Module_RF_working :1;
	};
} StatusWord;

typedef struct
{
	StatusWord sts;
	int Vbat;
	int Vsys;
	uint32_t UID;
	uint32_t LastRF;
	uint32_t ranging_exclusion;
	uint32_t LastLF_TX;
	uint32_t last_master_coms;
	uint32_t Force_RF;
	uint32_t exclusion;
	int CAN_baud_last;
	int CAN_SID_last;
	int UART_baud_last;
	uint32_t Setting_changed;
	// board ID to store what board is detected. 
	uint8_t board_id;
	zone LF_current_zone;				// current LF zone
	zone LF_last_zone; 					// last detected LF zone
	uint32_t last_slave_id;				// this is the last device we sent an RF master message to.
	uint32_t last_master_id;			// this is the last device that sent an RF master message to me.
	uint32_t boot_mode_time;
	LF_message_type LF_alert;

	uint8_t 		kind;
	// ---- GPS functionality ----
	GPS_Data_Type 	GPS_Data;
	Zone_Alert_Type Zone_Alert;

	//Vision V14 additions
	uint32_t Last_CAN; 					// ---- CAN HeartBeat Monitoring ----- //
	uint8_t TagTypeHolder;
	uint32_t Last_COM;
	uint8_t ManTagAck;					// ---- Acknowledgement Button status to RF packet ----- //
	uint8_t Reverse;					// ---- Reverse bit, to be added to status byte 2 in RF packet
	uint8_t Stopping_dist;				//Stopping distance parameter to be added to settings on EEPROM
	uint8_t MernokAssetFile_Def;		// ---- Last mernok asset file loaded onto EEPROM ---- //
	uint8_t Group_status;
	uint16_t Speed;
	TimeDate_Data_Type DateTime;

} _Vision_Status;



//------- Global Variables --------//

extern _Vision_Status Vision_Status;

union activities
{
	uint32_t word;
	struct
	{
		uint32_t tag_enable :1;			// turn the device on. otherwise it sleeps.
		uint32_t broadcast_ID :1; 		// will send RF packet every <period> milliseconds                                                                                                                                                                         
		uint32_t heartbeat :1; 			// will send out a periodic heartbeat                                                                                                                                                                
		uint32_t LF_TX :1; 				// will transmit LF TX message at the given interval                                                                                                                                                 
		uint32_t LF_response :1; 		// will send RF packets when an LF packet is received                                                                                                                                                
		uint32_t receive_RF :1; 		// will receive RF packets from other sources                                                                                                                                                        
		uint32_t Always_on :1; 			// will never enter low power state (irrespective of being battery powered)                                                                                                                          
		uint32_t accept_data :1; 		// this device will pass incoming RF data-forward packets to its master.                                                                                                                                                           
		uint32_t output_critical :1; 	// this is a test mode setting which will output critical distances to a GPIO                                                                                                                        
		uint32_t CAN_terminated :1;
		uint32_t forward_RF :1; 		// this device will forward nay RF packet to master. for testing                                                                                                                                                        
		uint32_t CAN_sync :1; 			// this device will send a sync message over the can when done sending LF.                                                                                                                           
		uint32_t get_range_all :1; 		// not used by PULSE                                                                                                                                                   
		uint32_t get_range_select :1; 	// not used by PULSE                                                                                                                                            
		uint32_t forward_dists :1; 		// will pass applicable range results directly to master.                                                                                                                                            
		uint32_t use_shortened_fw :1; 	// use a shortened autoforward message for the vision messages. 
		uint32_t send_name :1; 			// this tag will send its name string with RF messages
		uint32_t legacy_PDS :1; 		// this tag will send the shortened, old PDS message when it sends a RF packet.	
		uint32_t forward_own_lf :1; 	// this device will send messages to its master reporting its own LF levels. 
		uint32_t disable_exclusion :1;	// this tells the device to ignore exclusion LF messages. useful for vehicle tags.  
		uint32_t disable_LF_CRC :1;		// Device will send LF response even if LF CRC failed.
		// ---- GPS Functionality ----
		uint32_t GPS_capable :1;		// Device will broadcast ID with coordinates
		// -----V14-------------------
		uint32_t CAN_Heartbeat_monitor :1;  //device will monitor heartbeat messaged over CAN									-bit 22
		uint32_t broadcast_time :1;
	};

#ifdef __cplusplus
	/// default constructor
	activities() :
			word(0)
	{
	}
	activities(int& val) :
			word(val)
	{
	}
	// implicit conversion of activities to int
	operator int()
	{
		return word;
	}

	activities& operator=(int rhs)
	{
		word = rhs;
		return *this;
	}

	static activities mantag_activities()
	{
		activities p;
		p.tag_enable = 1;
		p.broadcast_ID = 1;
//		p.heartbeat = 1;
//		p.send_LF_TX = 1;
		p.LF_response = 1;
//		p.receive_RF = 1; 
//		p.Always_on = 1; 
//		p.accept_data = 1; 
		p.output_critical = 1;
//		p.CAN_terminated = 1;   
//		p.forward_RF = 1; 
//		p.CAN_sync = 1;  
//		p.get_range_all = 1; 
//		p.get_range_select = 1; 
//		p.forward_dists = 1; 
//		p.use_shortened_fw = 1; 
		p.send_name = 1;
//		p.legacy_PDS = 1;
//		p.forward_own_lf = 1; 	
//		p.disable_exclusion = 1;	
//		p.disable_LF_CRC = 1;
//		p.GPS_capable = 1;

		return p;
	}

	static activities reader_activities()
	{
		activities p;
		p.tag_enable = 1;
//		p.broadcast_ID = 1;
		p.heartbeat = 1;
//		p.send_LF_TX = 1;
//		p.LF_response = 1; 
		p.receive_RF = 1;
		p.Always_on = 1;
//		p.accept_data = 1;
//		p.output_critical = 1;
//		p.CAN_terminated = 1;   
		p.forward_RF = 1;
//		p.CAN_sync = 1;  
//		p.get_range_all = 1; 
//		p.get_range_select = 1; 
		p.forward_dists = 1;
//		p.use_shortened_fw = 1;
//		p.legacy_PDS = 1;
//		p.forward_own_lf = 1; 	
//		p.disable_exclusion = 1;	
//		p.disable_LF_CRC = 1;
//		p.GPS_capable = 1;
		return p;
	}
	
	static activities can_reader_activities()
	{
		activities p;
		p.tag_enable = 1;
//		p.broadcast_ID = 1;
		p.heartbeat = 1;
//		p.send_LF_TX = 1;
//		p.LF_response = 1; 
		p.receive_RF = 1;
		p.Always_on = 1;
//		p.accept_data = 1;
//		p.output_critical = 1;
//		p.CAN_terminated = 1;   
		p.forward_RF = 1;
//		p.CAN_sync = 1;  
//		p.get_range_all = 1; 
//		p.get_range_select = 1; 
//		p.forward_dists = 1;
		p.use_shortened_fw = 1;
//		p.legacy_PDS = 1;
//		p.forward_own_lf = 1; 	
//		p.disable_exclusion = 1;	
//		p.disable_LF_CRC = 1;
//		p.GPS_capable = 1;
		return p;
	}

	static activities pulse300_activities()
	{
		activities p;
		p.tag_enable = 1;
		p.broadcast_ID = 1;
		p.heartbeat = 1;
//		p.send_LF_TX = 1;
		p.LF_response = 1;
//		p.receive_RF = 1; 
//		p.Always_on = 1; 
//		p.accept_data = 1; 
		p.output_critical = 1;
//		p.CAN_terminated = 1;   
//		p.forward_RF = 1; 
//		p.CAN_sync = 1;  
//		p.get_range_all = 1; 
//		p.get_range_select = 1; 
//		p.forward_dists = 1; 
		p.use_shortened_fw = 1;
//		p.legacy_PDS = 1;
//		p.forward_own_lf = 1; 	
		p.disable_exclusion = 1;	
//		p.disable_LF_CRC = 1;
//		p.GPS_capable = 1;

		return p;
	}
	
	static activities ranger_activities()
	{
		activities p;
		p.tag_enable = 1;
		p.broadcast_ID = 1;
		p.heartbeat = 1;
//		p.send_LF_TX = 1;
//		p.LF_response = 1;
		p.receive_RF = 1; 
//		p.Always_on = 1; 
		p.accept_data = 1; 
//		p.output_critical = 1;
//		p.CAN_terminated = 1;   
//		p.forward_RF = 1; 
//		p.CAN_sync = 1;  
		p.get_range_all = 1; 
//		p.get_range_select = 1; 
		p.forward_dists = 1; 
//		p.use_shortened_fw = 1;
//		p.legacy_PDS = 1;
//		p.forward_own_lf = 1; 	
//		p.disable_exclusion = 1;	
//		p.disable_LF_CRC = 1;
//		p.GPS_capable = 1;

		return p;
	}

	static activities gps_activities()
	{
		activities p;
		p.tag_enable = 1;
//		p.broadcast_ID = 1;
		p.heartbeat = 1;
//		p.send_LF_TX = 1;
//		p.LF_response = 1;
//		p.receive_RF = 1;
		p.Always_on = 1;
//		p.accept_data = 1;
//		p.output_critical = 1;
//		p.CAN_terminated = 1;
		p.forward_RF = 1;
//		p.CAN_sync = 1;
//		p.get_range_all = 1;
//		p.get_range_select = 1;
//		p.forward_dists = 1;
//		p.use_shortened_fw = 1;
//		p.legacy_PDS = 1;
//		p.forward_own_lf = 1;
//		p.disable_exclusion = 1;
//		p.disable_LF_CRC = 1;
		p.GPS_capable = 1;

		return p;
	}
	static activities pulse500_activities()
	{
		activities p;
		p.tag_enable = 1;
//		p.broadcast_ID = 1;
		p.heartbeat = 1;
		p.LF_TX = 1;
		p.LF_response = 1; 
//		p.receive_RF = 1; 
		p.Always_on = 1; 
//		p.accept_data = 1; 
//		p.output_critical = 1;
//		p.CAN_terminated = 1;   
//		p.forward_RF = 1; 
		p.CAN_sync = 1;
//		p.get_range_all = 1; 
//		p.get_range_select = 1; 
//		p.forward_dists = 1; 
		p.use_shortened_fw = 1;
//		p.legacy_PDS = 1;
//		p.forward_own_lf = 1; 	
		p.disable_exclusion = 1;	
//		p.disable_LF_CRC = 1;
//		p.GPS_capable = 1;

		return p;
	}
#endif
};

#ifdef __cplusplus
class _vision_settings
{
public:

	int MernokAsset_Groups[255] = { };

	ParamValue* list[max_params] =
		{ };
// array of parameter pointers initialised to null

	ParamValue firmware;
	ParamValue tag_type;
	ParamValue rf_power;
	ParamValue slave_id;
	ParamValue vehic_id;
	ParamValue uartBaud;
	ParamValue can_baud;
	ParamValue interval;
	ParamValue activity;
//	ParamValue rf_chanl;
//	ParamValue max_dist;
	ParamValue lfPeriod;
	ParamValue lf_power;
	ParamValue lf_hertz;
	ParamValue name_str;
	ParamValue ack_time;
	ParamValue ack_intv;
	ParamValue lf_filtr;
	ParamValue vchrgmin;
	ParamValue rssiCrit;
	ParamValue rssiWarn;
	ParamValue rssiPres;
	ParamValue usrParam;
	ParamValue antOfset;
	ParamValue firmware_sub;
	ParamValue CAN_Timeout;
	ParamValue Type_revert;
	ParamValue Mernok_Asset_rev;
	ParamValue V_length;
	ParamValue V_width;
	ParamValue Product_ID;


	/// access a parameter based on ID.
	ParamValue* operator[](int index)
	{
		for (int i = 0; i < max_params; i++)
		{
			if (list[i]->address == index)
				return list[i];
		}
		return NULL;
	}

	_vision_settings(boards board)
	{
		set_defaults(board);
	}

	void set_defaults(boards board)
	{
		firmware = ParamValue(padr_firmware, 0, Firmware_rev, 99, "Firmware rev");
		tag_type = ParamValue(padr_tag_type, 0, Loco, 255, "Tag type");
		rf_power = ParamValue(padr_rf_power, 0, 7, 7, "RF output power");
		slave_id = ParamValue(padr_slave_id, 0, 10, 255, "CAN slave ID");
		vehic_id = ParamValue(padr_vehic_id, 0, 0, 0x7FFFFFFF, "Vehicle ID"); //TODO: NEil Change to 4byte
		uartBaud = ParamValue(padr_uartBaud, 9600, 115200, 1000000, "Serial buadrate");
		can_baud = ParamValue(padr_can_baud, 10000, 250000, 1000000, "CAN baudrate");
		interval = ParamValue(padr_interval, 200, 2000, 120000, "RF ping interval");
		activity = ParamValue(padr_activity, 0, 0, 0x7FFFFFFF, "Tag functions");
	//	rf_chanl = ParamValue(padr_rf_chanl, 0, 0, 0, "RF channel select");
	//	max_dist = ParamValue(padr_max_dist, 100, 10000, 32767, "Max dist passed");
		lfPeriod = ParamValue(padr_lfPeriod, 200, 800, 1000, "LF ping interval");
		lf_power = ParamValue(padr_lf_power, 0, 100, 100, "LF output power");
		lf_hertz = ParamValue(padr_lf_hertz, 50000, 125000, 150000, "LF frequency");
		name_str = ParamValue(padr_name_str, "", "Tag name");
		ack_time = ParamValue(padr_ack_time, 5000, 60000, 50000000, "Ack timeout");
		ack_intv = ParamValue(padr_ack_intv, 1, 4, 100, "Ack'ed warn interval");
		lf_filtr = ParamValue(padr_lf_filtr, 0, 5500, 60000, "LF filter time constant");
		vchrgmin = ParamValue(padr_vchrgmin, 3000, 4000, 20000, "min V-in charge");
		rssiCrit = ParamValue(padr_rssiCrit, 0, 7, 31, "LF critical threshold");
		rssiWarn = ParamValue(padr_rssiWarn, 0, 3, 31, "LF warning threshold");
		rssiPres = ParamValue(padr_rssiPres, 0, 0, 31, "LF presence threshold");
		usrParam = ParamValue(padr_usrParam, 0, 0, 0x7FFFFFFF, "User parameter");
		antOfset = ParamValue(padr_AntOfset, 0, 0, 255, "antenna offset");
		firmware_sub = ParamValue(padr_firm_sub, 0, Firmware_subrev, 99, "Firmware sub rev");
		CAN_Timeout = ParamValue(padr_CAN_heartbeat_time, 0, 10, 255, "CAN heartbeat time");
		Type_revert = ParamValue(padr_CAN_revert,0,Loco, 255, "Tag Revert Type");
		Mernok_Asset_rev = ParamValue(padr_Asset_list_rev,0,0,255,"Mernok Asset list rev");
		V_length = ParamValue(padr_length,1,1,255,"Mernok Asset Lenght");
		V_width  = ParamValue(padr_Width,1,1,255, "Mernok Asset Width");
		Product_ID = ParamValue(padr_product,0,1,254,"Product code");
		

		switch (board)
		{
		case ME_PCB_173_01:
		case ME_PCB_173_02:
		case ME_PCB_173_03:
		case ME_PCB_173_04:
		case ME_PCB_203_01:
		case ME_PCB_203_02:
		case ME_PCB_203_04:
			activity = activities::mantag_activities();
			tag_type = Person;
			uartBaud = 9600;
			break;
		case ME_PCB_182_01:
		case ME_PCB_182_02:
		case ME_PCB_182_03:
		case ME_PCB_182_04:
		case ME_PCB_182_05:
		case ME_PCB_182_06:
		case vision_reader:
			activity = activities::reader_activities();
			break;
		case pulse300:
			activity = activities::pulse300_activities();
			break;
		case pulse500:
			activity = activities::pulse500_activities();
			break;
		case can_PDS_reader:
			activity = activities::can_reader_activities();
			tag_type = Traffic_system_faulty;
			interval = 10000;
			can_baud = 10000;
			break;
		case ME_PCB_138_03:
			activity = activities::ranger_activities();
			break;
		case ME_PCB_217_01:
		case ME_PCB_217_02:
			activity = activities::gps_activities();
			break;
		default:
			break;
		}
		
		for (int i = 0; i < max_params; i++)
			list[i] = NULL;
		firmware.add_to_list(list);
		tag_type.add_to_list(list);
		rf_power.add_to_list(list);
		slave_id.add_to_list(list);
		vehic_id.add_to_list(list);
		uartBaud.add_to_list(list);
		can_baud.add_to_list(list);
		interval.add_to_list(list);
		activity.add_to_list(list);
//		rf_chanl.add_to_list(list);
//		max_dist.add_to_list(list);
		lfPeriod.add_to_list(list);
		lf_power.add_to_list(list);
		lf_hertz.add_to_list(list);
		name_str.add_to_list(list);
		ack_time.add_to_list(list);
		ack_intv.add_to_list(list);
		lf_filtr.add_to_list(list);
		vchrgmin.add_to_list(list);
		rssiCrit.add_to_list(list);
		rssiWarn.add_to_list(list);
		rssiPres.add_to_list(list);
		usrParam.add_to_list(list);
		antOfset.add_to_list(list);
		firmware_sub.add_to_list(list);
		CAN_Timeout.add_to_list(list);
		Type_revert.add_to_list(list);
		Mernok_Asset_rev.add_to_list(list);
		V_length.add_to_list(list);
		V_width.add_to_list(list);
		Product_ID.add_to_list(list);
	}

	activities getActivities(void)
	{
		return (activities)(activity._value);
	}
};

extern _vision_settings vision_settings;
#endif 

#ifdef __cplusplus
extern "C"
{
#endif

//------- Public functions --------//
int SaveSetting(int index, uint8_t* data, uint8_t len);
int SaveSettings(void);
int Get_MernokAsset_GroupValues(void);
int Set_Default_MernokAsset_GroupValues(void);

int GetSettings();
uint16_t GetStatusWord(void);
//_Vision_Status GetStatusfromWord(uint16_t data);
void USB_plugged(int in);
void DetermineTagType(void);


#ifdef __cplusplus
}
#endif

#endif /* RANGER_SETTINGS_H_ */

