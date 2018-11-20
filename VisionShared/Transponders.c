/*
 * Transponders.c
 *
 *  Created on: Nov 3, 2013
 *      Author: Kobus Goosen
 */

#include "Transponders.h"
#include "Vision_Parameters.h"
#include "Vision_HAL.h"

//#include "Global_Variables.h"


#ifndef num_transp_total
#define num_transp_total 256
#endif

_Transpondert transp_log[num_transp_total];
uint8_t transp_count = 0;
bool types_to_range[256] ={ };			// holds a record of all the transp types this one searches for. todo: consider keeping this on eeprom	
xList Tag_list;

_Transpondert* get_tag(uint32_t UID)
{
	int i;
	// check if the UID exists in the list.
	for (i = 0; i < num_transp_total; i++)
	{
		if (transp_log[i].UID == UID)
			return &transp_log[i];
	}
	return 0;
}

int get_slot(uint32_t UID)
{
	int i, old_tag = num_transp_total;
	uint32_t oldest = time_now();

	// check if the UID exists in the list. 
	for (i = 0; i < num_transp_total; i++)
	{
		if (transp_log[i].UID == UID)
			return i;
	}

	// the UID does not yet exist, so add new entry.
	// add the new transponder to the list 
	for (i = 0; i < num_transp_total - 1; i++)
	{
		if (transp_log[i].UID == 0)			// find the first empty slot
			break;							// found a slot. now use it. 
		else if (transp_log[i].last_seen < oldest) //keep track of the oldest tag in the list 
		{
			oldest = transp_log[i].last_seen;
			old_tag = i;
		}
	}
	// catch if the list is full...
	if (i >= num_transp_total - 2)
	{
		// we want to replace the oldest tag in the list
		i = old_tag;

		clear_transp(&transp_log[i]);		// clean the item of all data.
	}
	else
		transp_count++; 					// add the new transponder to the list. if we're replacing an old entry, list size doesn't change.

	// we now have a valid slot, lets ensure its list item is intact
	vListInitialiseItem(&(transp_log[i].listI));
	transp_log[i].listI.pvOwner = &transp_log[i];
	transp_log[i].listI.xItemValue = 0xFF00;		// just give it a large value to go to the back...
	vListInsert(&Tag_list, &transp_log[i].listI);

	return i;
}

void re_insert_transp(_Transpondert* T)
{
	// remove and re-insert it into list to update distance sort position. 
	uxListRemove(&(T->listI));					
	T->listI.xItemValue = T->Dist;
	T->listI.pvOwner = T;
	vListInsert(&Tag_list, &(T->listI));
}

/**
 * @brief: updates the UID in the list's last seen time. 
 * @param UID
 * @return
 */
int log_uid(uint32_t UID)
{
	int i;
	for (i = 0; i < num_transp_total; i++)
	{
		if (transp_log[i].UID == UID)
		{
			transp_log[i].last_seen = time_now();
			return 1;
		}
	}
	return 0;
}

void random_tag_generator(void)
{
	int i, list_item;
	
	for(i =1; i<num_transp_total; i++)
	{
		list_item = get_slot(i);
		transp_log[list_item].UID = i;
		if(i>64)
			transp_log[list_item].kind = PDS;
		else
			transp_log[list_item].kind = Pulse;
		transp_log[list_item].last_seen = 1000*i;
		transp_log[list_item].SlaveID = i;
		transp_log[list_item].VehicleID = 1*100;
		transp_log[list_item].LF.last_LF = 1000*i;
		transp_log[list_item].LF.RSSI = i%32;
		transp_log[list_item].rssi = i%255;
		transp_log[list_item].volts = 60+i%30;
		transp_log[list_item].type = i%16;
	}
}

void clear_transp_log(void)
{
	int i;
	for (i = 0; i < num_transp_total - 1; i++)
	{
		clear_transp(&transp_log[i]);
	}

	transp_count = 0;

	//////////////////////
	vListInitialise(&Tag_list);
	
	
//	random_tag_generator();
}

/**
 * @brief 	Clean all tags older than a certain time in seconds.
 * age		the age cutoff to clean in seconds. (older tags will be cut).
 */
int clean_transp_log(int age)
{
	int i, count = 0;
	for (i = 0; i < num_transp_total - 1; i++)
	{
		if ((transp_log[i].UID != 0) && (time_since(transp_log[i].last_seen) > age * 1000))
		{
			clear_transp(&transp_log[i]);
			count++;
			transp_count--;
		}
	}
	return count;
}

void clear_transp(_Transpondert* T)
{
	if(T->listI.pvOwner != 0)
		uxListRemove(&T->listI); // remove the old entry in the list.


	T->UID = 0;
	T->last_seen = 0;
	T->type = 0;
	T->group = 0;
	T->status = 0;
	T->ManTagAck = 0;
	T->rssi = 0;
	T->volts = 0;
	T->SlaveID = 0xFF;
	T->VehicleID = 0xFFFF;
	T->range_retries = 0;
	T->range_needed = 0;
	T->Dist = 0;
	T->kind = unknown;
	T->LF.RSSI = -1;
	T->LF.last_LF = 0;
	T->LF.VehicleID = 0;
	T->Speed = 0;
	T->GPS_Data.Longitude = 0;
	T->GPS_Data.Latitude = 0;
	T->GPS_Data.FixType = 0;
	T->GPS_Data.HorizontalAccuracy = 0;
	T->GPS_Data.VerticalAccuracy = 0;
	T->GPS_Data.Speed = 0;
	T->GPS_Data.HeadingVehicle = 0;
	T->GPS_Data.FixAge = 0xFF;

#ifdef USE_TAG_NAME
	memset(T->name, 0, STR_MAX);
#endif

#ifdef traffic_system
	T->current_gate = 0;
	T->last_gate = 0;
	T->last_seen_gate = 0;
#endif
}

/**
 * @brief: returns true if the type is allowed, false otherwise
 */bool transp_type_check(uint8_t type)
{
	return types_to_range[type];
}

/**
 * @brief: sets the transponder type in the list enabled = true, false - disabled.
 */
void transp_type_add(uint8_t type, bool val)
{
	types_to_range[type] = val;
}

/**
 * @brief: returns the next transponder meeting the filter requirements
 * @param transp - pointer to the transponder that is found, if it is. 
 * @param restart - start from the top of the list
 * @param max_dist - only transps closer than this will be returned. if 0 all will be returned
 * @param max_age - only tags seen more recent than max_age seconds ago will be returned
 * @param type - only this type of tag will be returned. 0  returns any type
 * @return true if a matching transponder was found, false otherwise
 */
_Transpondert* transp_filter(int restart, int max_dist, int max_age, uint8_t type, uint8_t filter_vehicles, _kind kind)
{
	//bool dist_ok = true, age_ok = true, type_ok = true, kind_ok = true;
	bool dist_ok = false, age_ok = false, type_ok = false, kind_ok = false;
	_Transpondert* transp;
	
	if (restart == 0)
	{
		Tag_list.pxIndex = Tag_list.xListEnd.pxNext;
		// check if the list has been initialised. 
		if(Tag_list.uxNumberOfItems == 0 && (Tag_list.pxIndex == 0))
			clear_transp_log();
	}

	while (Tag_list.pxIndex != (xListItem*) &Tag_list.xListEnd)
	{
		transp = (_Transpondert*) Tag_list.pxIndex->pvOwner;
		Tag_list.pxIndex = Tag_list.pxIndex->pxNext;
		if (transp->UID)
		{
			if (type)
			{
				if (transp->type == type)
					type_ok = true;
				else
					type_ok = false;
			}
			else
				type_ok = true;

			if (kind != unknown)
			{
				if (transp->kind == kind)
					kind_ok = true;
				else
					kind_ok = false;
			}
			else
				kind_ok = true;

			if (max_dist == 0xFFFF)
				dist_ok = true;
			else
			{
				if (transp->kind == Ranging)
				{
					if ((transp->Dist != 0) && (transp->Dist < max_dist) && (max_dist != 0xFFFF))	// return only PODs with range data if a max_dist is given
						dist_ok = true;
					else if ((transp->Dist == 0) && (max_dist == 0xFFFF))	// add a special case that returns all PODs without distance data if dist = 0xffff
						dist_ok = true;
					else
						dist_ok = false;
				}
				else if (transp->kind == Pulse)
				{
					if ((transp->LF.RSSI >= max_dist) && (transp->LF.last_LF != 0))
						dist_ok = true;
					else
						dist_ok = false;
				}
				else
					dist_ok = false;
			}
			
			if (max_age)
			{
				if (max_dist == 0xFFFF)
				{
					if (time_since(transp->last_seen) < (uint32_t) (max_age * 1000))
						age_ok = true;
					else
						age_ok = false;
				}
				else
				{
					if (time_since(transp->LF.last_LF) < (uint32_t) (max_age * 1000))
						age_ok = true;
					else
						age_ok = false;
				}
			}
			else
				age_ok = true;
		}


		if (dist_ok && age_ok && type_ok && kind_ok)
		{
			return transp;			// send the valid transponder back. incr static pointer to next item for the next round of checks.
		}
	}
	return 0;
}

