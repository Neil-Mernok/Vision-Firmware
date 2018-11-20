/*
 * Tasks.h
 *
 *  Created on: Feb 2015
 *      Author: KobusGoosen
 */

#ifndef BOOTTASKS_H_
#define BOOTTASKS_H_

#include "Global_Variables.h"

extern uint16_t boot;
extern uint16_t boot_error_count;
extern uint8_t boot_data[];
extern uint16_t boot_buf_counter;

/////////////////////// 	Tasks		//////////////////////////////////
//// task structs
extern task settings;
extern task cc1101;
extern task messages_to_master;

void USB_start(void);
void Master_task(task* t);
void boot_process(task* t);
void jump(void);
void CC1101_Task(task* t);

#endif /* RTOS_TASKS_H_ */
