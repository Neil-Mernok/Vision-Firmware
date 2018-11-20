/*
 * Tasks.c
 * Created on: September, 2016
 * Author: Kobus Goosen
 */

#include "Tasks.h"
#include "Transponders.h"
#include "RF_APL.h"
#include "Nanotron_RF_APL.h"
#include "phy.h"


/**
 * @brief: sends a packet to the nanotron task asking it to range to the give ID, after a random number of milliseconds. 
 * @param ID
 */
extern "C" void add_ID_to_be_ranged(uint32_t ID)
{
	range_id_type item_to_range;
	int i = 0;

	_Transpondert* T = get_tag(ID);
	if (T != NULL)
	{
		if (time_since(T->last_seen) < RANGE_SLOT_TIME)
		{
			item_to_range.UID = ID;
			item_to_range.time_to_range = time_now();
			/// set the time we want to range to this item. Now plus a random time < RANGE_SLOT_TIME.
			/// make the time shorter if we haven't seen it for a while. 
			//if(time_since(transp_log[list_item].last_ranged) > (vision_settings.interval*3))
			item_to_range.time_to_range += rand() & ((RANGE_SLOT_TIME - 1));
			//else
			//item_to_range.time_to_range += RANGE_SLOT_TIME>>1 + rand()&((RANGE_SLOT_TIME-1)>>1);
			pipe_put(&nanotron.p, &item_to_range);
		}
		else
		{
			i++;
			return;
		}
	}
}


/**
 * @brief this is called when a atg has new distance info. 
 * @param pvParameters
 */
extern "C" void Range_handler(_Transpondert* t, uint16_t dist, bool forward)
{
	Vision_Status.sts.RF_Working = true;

	t->Dist = dist;
	t->range_retries = 0;
	t->last_ranged = time_now();
	// if this is an original range value, send a range report to the other tag. 
	if(forward == false)
		nApl_range_callout(t->UID, dist, 0);
		
	if (vision_settings.getActivities().forward_dists)
	{
		if (t->Dist <= 65535 /*vision_settings.max_dist*/)					// todo: add back max-dist. 
		{
			Send_POD_toMaster(t, CODE, 3);
		}
	}
}


/**
 * @brief: sets the nanotron device and all parameters. initialises the isr handler task. 
 * @param pvParameters
 */
void Nanotron_task(task* t, int* cant_sleep)
{
	static uint8_t last_power;
	static range_id_type RID;
	RF_message to_send;
	static long dummy = 0;
	//int i;

	if (t->state == 0)			// this is run the first time only. 
	{
		// create a pipe to hold IDs we need to range to. 
		pipe_init(&(t->p), 10, sizeof(range_id_type));
		// make usre the RF pipe is open for coms. 
		if(Vision_Status.sts.CC_SPI_working == 0)
			pipe_init(&cc1101.p, 25, sizeof(RF_message));

#	if CONFIG_REG_MAP_NR == 501
		NTRXSetDefaultMode();
#	endif
		if (NTRXInit() == FALSE)
		{
			Vision_Status.sts.NN_SPI_working = false;
			t->state = -1;						// instruct the nanotron task to never run again. 
			return;
		}
		else
			Vision_Status.sts.NN_SPI_working = true;

		// Initialise user application
		last_power = vision_settings.rf_power;		//todo: fix RF power for Ranger
		APLInit(Vision_Status.UID, last_power*9, vision_settings.antOfset);

		t->state = 1;			// indicate that the process can run. 
	}
	else if (t->state == -1)
		return;

	/////////////////////////////////////////////////////////////////////////////////////////////
	//// 	this is where the processing happens
	/////////////////////////////////////////////////////////////////////////////////////////////

	////	First check if we need to range to any tags.  
	if (pipe_peek(&t->p, &RID)!= 0 && PHYIsIDLE())
	{
		if (RID.time_to_range < time_now())
		{
			pipe_get(&t->p, &RID); 					// if its time, remove from the queue. 
			// only request ranges if we have a master interested in the result...
			bool should_range = false;
			if(time_since(Vision_Status.last_master_coms) < 15000) 
				should_range = true;
			
			// don't ranging if we've got a range very recently
			_Transpondert* tagt = get_tag(RID.UID);
			if(tagt != NULL){
				if((tagt->RangePeriod != 0) && (time_since(tagt->last_ranged) < ((uint32_t)tagt->RangePeriod - RANGE_SLOT_TIME - 15)))
					should_range = false;
			}
		
			// check if we need to send a range request
			if (should_range)
			{
				// perform the ranging 
				if(APL_RangeToID(RID.UID, vision_settings.antOfset) != 0)
				{ 
					// the ranging failed. retry a few times.  
					if(tagt->range_retries++ < 3)
						add_ID_to_be_ranged(tagt->UID);
				} 
			}
			else
				pipe_get(&t->p, &RID);				// keep the range requests from piling up if we cant range. 
							
			dummy = 0;
		}
		else
			dummy++;
	}
	////	Use this opportunity to send any pending RF like master coms, etc. 
	else if ((Vision_Status.sts.CC_SPI_working == 0) && (pipe_peek(&cc1101.p, &to_send))) 
	{
		// insert a delay parameter in the packet to tell it to wait a certain amount before responding. creates RF slots. 
		if (to_send.time_to_respond <= time_now())
		{
			pipe_get(&cc1101.p, &to_send);			// remove the message from the list as we're going to process it now. 
			switch (to_send.type)
			{
			case rf_MasterP:
				nApl_send_master_message(to_send.buff, to_send.len);
				break;
			case rf_Data:
				nApl_send_data_message(to_send.buff, to_send.len);
				break;
			case rf_Respons:
				nApl_send_master_response(to_send.buff, to_send.len);
				break;
			case rf_ID_name:
			case rf_ID_puls:
				// forcibly send ID massage.
				SetLed(&LED1, Green, 0);
				nApl_report_ID(to_send.type);
				SetLed(&LED1, LED1.last_color, 0);
				break;
			default:
				break;
			}
		}
		/////(*cant_sleep)++; // cc chip still busy, so we cant sleep.
	}
	////	Otherwise, if possible we can send a range request. 
	else if (time_now() > t->pause_until)			// timer for the task has expired. need to send ID ping.
	{
		if (time_since(Vision_Status.ranging_exclusion) < (RANGE_SLOT_TIME + 5))
		{
			// RF is busy, so assume some else is busy with ranging.
			// so delay processing for another while
			task_delay(t, RANGE_SLOT_TIME + 10 + rand() % 30);
			return;
		}

		// reset the task timer
		task_delay(t, vision_settings.interval._value + rand() % 20);

		// transmit RF if needed. 

		//APL_Calibrate();
//		APL_RX_EN(1);
		if (last_power != vision_settings.rf_power._value)
		{
			last_power = vision_settings.rf_power;
			APL_SetTXPow(last_power*9);
		}

		if (vision_settings.getActivities().broadcast_ID)
		{
			SetLed(&LED1, Green, 0);
			nApl_report_ID(rf_RangReq);
			SetLed(&LED1, Off, 0);
		}	
	}

	// always run the phypoll....
	PHYPoll(0);
	
	// effectively any ranging activity keeps the micro awake. 
	if ((vision_settings.getActivities().broadcast_ID) || (vision_settings.getActivities().get_range_all) || (vision_settings.getActivities().get_range_select))
		(*cant_sleep)++;
}
