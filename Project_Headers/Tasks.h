/*
 * Tasks.h
 *
 *  Created on: Feb 2015
 *      Author: KobusGoosen
 */

#ifndef TASKS_H_
#define TASKS_H_

#include "Global_Variables.h"
#include "my_rf_settings.h"
#include "Vision_Parameters.h"
#include "master_interface.h"
#include "as3933.h"
#include "RF_APL.h"
#include "LF_TX.h"

#ifdef __cplusplus
extern "C" {
#endif

// this is the total amount of time that the devices can ask to range after an ID broadcast (ms). 
#define RANGE_SLOT_TIME		128

typedef struct range_id_list 
{
	uint32_t	UID;
	uint32_t time_to_range;	
} range_id_type;

/////////////////////// 	Tasks		//////////////////////////////////

void Master_task(task* t, int* cant_sleep);
void Nanotron_task(task* t, int* cant_sleep);
void GetStatus(task* t);
void Refresh_Settings_task(task* t);
uint8_t get_TAG_kind(void);
void USB_start(void);
void CC1101_Task(task* t, int* rf_wake_flag, int* cant_sleep);
void LF_Task(task* t, int* cant_sleep);
void LF_TX_Task(task* t);
void LF_RSSI_watcher(task* t, int count, int* cant_sleep);
void Distress_watcher(task* t, int* cant_sleep);

void uBlox_task(task* t, int* cant_sleep);
void uBlox_Init(void);


//Todo: Neil eeprom function
void EEPROM_size_Task(task* t);
void MernokAssetGroupstoList(task* t);

//void Test_task(task* t);
//void Test_task2(task* t);
//void Test_task3(task* t);
//void Test_task4(task* t);
//void Test_task5(task* t);
//void Test_task6(task* t);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_TASKS_H_ */
