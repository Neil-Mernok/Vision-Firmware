/*
 * task.h
 *
 *  Created on: Oct 16, 2014
 *      Author: KobusGoosen
 */

#ifndef TASK_H_
#define TASK_H_

#include <stdint.h>
#include <stdbool.h>
#include "list.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct task
{
	int (* taskhandler)();
	uint32_t pause_until;
	kpipe p;
	int state;
	int run_flag;
	void* task_vars; 					// pointer to some task structure used to store variables
}task;


int task_process(task* t);
void task_delay(task* t, long millis);
#ifdef __cplusplus
}
#endif

#endif /* TASK_H_ */
