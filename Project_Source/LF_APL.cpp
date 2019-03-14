/*
 * LF_APL.c
 *
 *  Created on: Sep 12, 2014
 *      Author: KobusGoosen
 */

#include "RF_APL.h"
#include "Global_Variables.h"
#include "Vision_Parameters.h"

#define MAX_LF_Transmitter_count 255

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
RssiTracker local_RSSI;
LF_acknowledge LF_ack, LF_last;
Zone_Info_Type RF_Zone;
Zone_Info_Type LF_Zone;
Zone_Info_Type LF_CAN_Sync_List[MAX_LF_Transmitter_count];

uint8_t LF_trnasmitter_count = 0;
uint8_t LF_trnasmitter_index = 0;

void swap(Zone_Info_Type *xp, Zone_Info_Type *yp)
{
	Zone_Info_Type temp = *xp;
    *xp = *yp;
    *yp = temp;
}

//Sort the list of transmitters according to SIDs
void selectionSort(Zone_Info_Type arr[], int n)
{
    int i, j, min_idx;

    // One by one move boundary of unsorted subarray
    for (i = 0; i < n-1; i++)
    {
        // Find the minimum element in unsorted array
        min_idx = i;
        for (j = i+1; j < n; j++)
          if (arr[j].SlaveID > arr[min_idx].SlaveID)
            min_idx = j;

        // Swap the found minimum element with the first element
        swap(&arr[min_idx], &arr[i]);
    }
}

//Add all the transmitters on the CAN Bus to the List to make them send sequentially
void Add_to_LF_Sync_list(uint8_t SlaveID)
{
	uint8_t LF_trnasmitter_count_temp = 0;
	bool present = false;
	uint8_t Last_index = 0;
	for(int i = 0 ; i<MAX_LF_Transmitter_count;i++)
	{
		if(LF_CAN_Sync_List[i].SlaveID == SlaveID)
		{
			present = true;
			LF_CAN_Sync_List[i].Last_Seen = time_now();
		}
		if(LF_CAN_Sync_List[i].SlaveID != 0)
		{
			Last_index = i;
			LF_trnasmitter_count_temp++;
		}
	}

	if(!present)
	{
		LF_CAN_Sync_List[Last_index+1].SlaveID = SlaveID;
		LF_CAN_Sync_List[Last_index+1].Last_Seen = time_now();
	}

	selectionSort(LF_CAN_Sync_List,5);
	LF_trnasmitter_count = LF_trnasmitter_count_temp;
}

//Remove the transmitter if not seen for a while so that a different SID can become the master
void Clean_LF_Sync_list(void)
{
	for(int i = 0; i < LF_trnasmitter_count; i++)
	{
		if(time_since(LF_CAN_Sync_List[i].Last_Seen)>((uint32_t)(1000 + 2*vision_settings.lfPeriod._value)))
		{
			LF_CAN_Sync_List[i].SlaveID = 0;
			LF_CAN_Sync_List[i].Last_Seen = 0;
		}
	}
	selectionSort(LF_CAN_Sync_List,LF_trnasmitter_count);
}

zone zone_from_rssi(int RSSI)
{
	zone z = zone_none;
	if (RSSI >= (int) vision_settings.rssiCrit)
		z = zone_crit;
	else if (RSSI >= (int) vision_settings.rssiWarn)
		z = zone_warn;
	else if (RSSI >= (int) vision_settings.rssiPres)
		z = zone_pres;
	return z;
}

/**
 * @brief generates warning outputs based on the zone.
 * @param c_zone
 */
void Output_zone(zone c_zone)
{
	int time = 100;
#ifdef WARNING_OUTPUTS
	bool buz = false, vib = false, led = false;
	int lamp = 0;
#endif

	if (vision_settings.getActivities().output_critical)
	{
		// if the tag has not been excluded, output LF 
		if(time_now() > Vision_Status.exclusion)
		{
			switch (c_zone)
			{
				case zone_crit:
					SetLed(&LED1, Red, time);
#ifdef WARNING_OUTPUTS
					led = true;
					buz = true;
					vib = true;
					lamp = 10;
#endif
					break;
				case zone_warn:
					SetLed(&LED1, Yellow, time+50);
#ifdef WARNING_OUTPUTS
					led = true;
					buz = false;
					vib = true;
					lamp = 5;// TODO: NEil tune this value for yellow on the new alert module firmware
#endif
					break;
				case zone_pres:
					SetLed(&LED1, Blue, time);
#ifdef WARNING_OUTPUTS
					led = true;
					buz = false;
					vib = false;
					lamp = 2;// TODO: NEil tune this value for yellow on the new alert module firmware
#endif
					break;
				default:
					break;
			}
		}
		// If excluded, output Cyan colour LEDs.
		else
		{
			SetLed(&LED1, Cyan, time);
#ifdef WARNING_OUTPUTS
			led = false;
			buz = false;
			vib = false;
			lamp = 0;
#endif
		}

#ifdef WARNING_OUTPUTS
		if (lamp)
			SetGPO(&LAMP_OUT, lamp);
		if (buz)
			SetGPO(&BUZ_OUT, time);
		if (vib)
			SetGPO(&VIB_OUT, time);
#ifdef NOTTIliT
		if (led)
			SetGPO(&LED_OUT, time);
#endif
#endif 
	}
}

void Apl_handle_RSSI(LF_message_type LFM)
{
	/// log the RSSI and timestamp it. 
	LFM.last_LF = time_now();
	local_RSSI.add_RSSI(LFM);

	int RSSI = local_RSSI.get_highest_RSSI(vision_settings.lf_filtr);

	Vision_Status.LF_current_zone = zone_from_rssi(RSSI);

	// keep track of the most recent LF here. in case the button gets pressed
	LF_last.LF = LFM;
	LF_last.zone_last = Vision_Status.LF_current_zone;

	LF_Zone.Zone = Vision_Status.LF_current_zone;
	LF_Zone.VehicleID = LFM.VehicleID;
	LF_Zone.SlaveID = LFM.SlaveID;
	LF_Zone.Last_Seen = LFM.last_LF;
	LF_Zone.Technology = LF_Technology;
	Apl_Compare_Zone_and_Update(LF_Zone);

}


///**
// * @brief this function most be called in regular intervals.
// * checks the filtered RSSI and outputs it.
// */
void Apl_Checkzone_and_output(void)
{
	static int count = 0;

	// ---- Check if the current zone received are different from the previous received ----
	if (Vision_Status.Zone_Alert.Zone_ACK.Zone != Vision_Status.Zone_Alert.Zone_Active.Zone)
	{
		Vision_Status.Zone_Alert.Alert_Snooze = 0;
		Vision_Status.Zone_Alert.Zone_ACK.Last_Seen = 0;
		Vision_Status.Zone_Alert.Zone_ACK.SlaveID = 0;
		Vision_Status.Zone_Alert.Zone_ACK.VehicleID = 0;
		Vision_Status.Zone_Alert.Zone_ACK.Zone = Vision_Status.Zone_Alert.Zone_Active.Zone;


	}
	// ---- If the current zone is not none output the zone ----
	if (Vision_Status.Zone_Alert.Zone_Active.Zone != zone_none)
	{
		if ((time_now() > Vision_Status.Zone_Alert.Alert_Snooze) || (Vision_Status.Zone_Alert.Zone_Active.VehicleID != Vision_Status.Zone_Alert.Zone_ACK.VehicleID))
		{
			// ---- Output Zone ----
			Output_zone(Vision_Status.Zone_Alert.Zone_Active.Zone);
			Vision_Status.ManTagAck = 0;
		}
		// ---- Reset current zone if update as not been received ----
		if (time_since(Vision_Status.Zone_Alert.Zone_Active.Last_Seen) > Zone_Alert_Snooze_Time)
		{
			Vision_Status.Zone_Alert.Zone_Active.Zone = zone_none;

		}
	}
	else
	{
		Vision_Status.ManTagAck = 0;
		// no LF, but we can display the charge status... but only if we're a mantag.
#ifdef WARNING_OUTPUTS
		if(time_now() < Vision_Status.exclusion)
			SetLed(&LED1, Cyan, 100);
		else if(Vision_Status.sts.EXT_Power && Vision_Status.sts.Charging)
			SetLed(&LED1, Violet, 400);
#endif
	}
	// ---- Set previous zone to current zone ----
	Vision_Status.LF_last_zone = Vision_Status.LF_current_zone;

	count++;
}

/**
 * @brief this function most be called in regular intervals.
 * checks the filtered RSSI and outputs it.
 */
void Apl_Compare_Zone_and_Update(Zone_Info_Type Compare_Zone)
{
	// ---- Check which of zone's priority is the bigger ----
	if (Vision_Status.Zone_Alert.Zone_Active.Zone < Compare_Zone.Zone)
	{
		Vision_Status.Zone_Alert.Zone_Active = Compare_Zone;
	}
	// ---- Check which of zone's priority is the same ----
	else if (Vision_Status.Zone_Alert.Zone_Active.Zone == Compare_Zone.Zone)
	{
		// ---- Check which of zone's VID is the same and update last seen ----
		if (Vision_Status.Zone_Alert.Zone_Active.VehicleID == Compare_Zone.VehicleID)
		{
			Vision_Status.Zone_Alert.Zone_Active.Last_Seen = Compare_Zone.Last_Seen;
		}
	}
}



void Apl_acknowledge_LF(void)
{

	if ((Vision_Status.Zone_Alert.Alert_Snooze < time_now()) && (Vision_Status.Zone_Alert.Zone_Active.VehicleID == Vision_Status.Zone_Alert.Zone_ACK.VehicleID))
	{
		Vision_Status.Zone_Alert.Zone_ACK.VehicleID = 0xFFFF;
		Vision_Status.ManTagAck = 1;
	}
	else if (Vision_Status.Zone_Alert.Zone_Active.VehicleID != Vision_Status.Zone_Alert.Zone_ACK.VehicleID)
	{
		Vision_Status.Zone_Alert.Zone_ACK.Technology = Vision_Status.Zone_Alert.Zone_Active.Technology;
		Vision_Status.Zone_Alert.Zone_ACK.SlaveID = Vision_Status.Zone_Alert.Zone_Active.SlaveID;
		Vision_Status.Zone_Alert.Zone_ACK.VehicleID = Vision_Status.Zone_Alert.Zone_Active.VehicleID;
		Vision_Status.Zone_Alert.Alert_Snooze = time_now() + (int) vision_settings.ack_time;
		Vision_Status.ManTagAck = 1;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

