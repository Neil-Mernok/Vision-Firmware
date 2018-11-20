/*
 * LF_APL.h
 *
 *  Created on: Jul 19, 2015
 *      Author: Kobus
 */

#ifndef LF_APL_H_
#define LF_APL_H_

#include "Delay.h"

#ifdef __cplusplus
extern "C" {
#endif

#define Zone_Alert_Snooze_Time 3000



//static uint8_t MantagAck;
typedef enum zone
{
	zone_none,
	zone_pres,
	zone_warn,
	zone_crit
} zone;

typedef enum Zone_Tech
{
	No_Technology = 0,
	LF_Technology = 1,
	RF_Technology = 2
}Zone_Tech;

typedef struct LF_message_type
{
    int8_t RSSI;
    uint8_t  SlaveID;
    uint32_t VehicleID; //TOdo: Neil, change this to a 32bit value
    uint32_t last_LF;
} LF_message_type;

typedef struct LF_acknowledge
{
	int zone_last;
	uint32_t block_till;
	LF_message_type LF;
} LF_acknowledge;

// ---- V12 Alert Update ----
typedef struct _Zone_Info_Type
{
	Zone_Tech 	Technology;
	uint8_t  	SlaveID;
	uint32_t 	VehicleID;
    uint32_t	Last_Seen;
    zone 		Zone;

} Zone_Info_Type;

typedef struct _Zone_Alert_Type
{
	Zone_Info_Type	Zone_Active;
	Zone_Info_Type 	Zone_ACK;
	uint32_t 		Alert_Snooze;
} Zone_Alert_Type;

extern uint8_t LF_trnasmitter_count;
extern uint8_t LF_trnasmitter_Send;

void Add_to_LF_Sync_list(uint8_t SlaveID);
void Clean_LF_Sync_list(void);
zone zone_from_rssi(int RSSI);
void Apl_acknowledge_LF(void);
void Apl_handle_RSSI(LF_message_type LFM);
void Apl_Checkzone_and_output(void);
void Output_zone(zone c_zone);
void Apl_Compare_Zone_and_Update(Zone_Info_Type Compare_Zone);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief This function keeps a rolling max of the RSSI value, with a specified timeout. 
 */
struct RssiTracker
{
public:
	RssiTracker() :
			timeout(2000)
	{
		latest.RSSI = -1;
		other.RSSI = -1;
		biggest.RSSI = -1;
	}
	RssiTracker(int time_out) :
			timeout(time_out)
	{
		latest.RSSI = -1;
		other.RSSI = -1;
		biggest.RSSI = -1;
	}

	int get_highest_RSSI(int time_out)
	{
		timeout = time_out;
		update();
		return biggest.RSSI;
	}

	void add_RSSI(LF_message_type current)
	{
		latest = current;
//		update();
	}

protected:
	uint timeout;
	LF_message_type latest, biggest, other;
	uint8_t ptr = 0;

	void update()
	{
		if (time_since(latest.last_LF) > timeout)
			latest.RSSI = -1;
		if (time_since(other.last_LF) > timeout)
			other.RSSI = -1;
		if (time_since(biggest.last_LF) > timeout)
			biggest.RSSI = -1;
		if (other.RSSI < latest.RSSI)
			other = latest;
		if (biggest.RSSI < other.RSSI)
			biggest = other;
	}
};
#endif

#endif /* LF_APL_H_ */
