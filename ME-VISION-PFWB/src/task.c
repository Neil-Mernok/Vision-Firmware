/*
 * task.c
 *
 *  Created on: Oct 16, 2014
 *      Author: KobusGoosen
 */

#include "Global_Variables.h"

/**
 * @brief: check if the task is ready to be run, then run it.
 * reasons to run a task:
 * 							- its run flag has been set
 * 							- its pipe has data in. 
 * 							- its delay has been reached. 
 * @param t: the task in question. 
 * @return
 */
int task_process(task* t)
{
//	if(t->run_flag)
//	{		
//		t->taskhandler(t);					// call the task itself.
//		return 1;
//	}
//	else if(t->pause_until != 0)
//	{ 
//		if(t->pause_until<=time_now())				// timer has elapsed
//		{
//			t->taskhandler(t);					// call the task itself.
//			return 1;
//		}
//	}
//	else if(pipe_peek(&(t->p)))												// data avaibale in the pipe
//	{
//		t->taskhandler(t);					// call the task itself.
//		return 1;
//	}
	return 0;  
}


/**
 * @brief: adjusts the task params so that it is ignored for at least *millis* ms. 
 * @param t
 * @param millis
 */
void task_delay(task* t, long millis)
{
	t->pause_until = time_now()+millis;
}

