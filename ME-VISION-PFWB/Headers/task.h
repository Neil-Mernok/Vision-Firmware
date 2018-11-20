/*
 * task.h
 *
 *  Created on: Oct 16, 2014
 *      Author: KobusGoosen
 */

#ifndef TASK_H_
#define TASK_H_

#include "Global_Variables.h"


typedef struct task
{
	int (* taskhandler)();
	long pause_until;
	kpipe p;
	int state;
	int run_flag;
	void* task_vars; 					// pointer to some task structure used to store variables
}task;


int task_process(task* t);
void task_delay(task* t, long millis);

#endif /* TASK_H_ */
