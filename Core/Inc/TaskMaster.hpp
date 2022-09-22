/*
 * TaskMaster.hpp
 *
 *  Created on: Sep 9, 2022
 *      Author: heapr0ll
 */

#ifndef INC_TASKMASTER_HPP_
#define INC_TASKMASTER_HPP_

#include "main.h"
#include "stm32f4xx_it.h"
#include <utility>
#include "Task.hpp"
#include <array>

template<int N>
class TaskMaster
{

public:

	// Initializers.
	TaskMaster();
	~TaskMaster();

	// Task Scheduling utility functions.
	void update();						// Updates task meta data (delay, period, data, state, etc.)
	void runNextTask();					// Function to kick off a task when there time has expired.

	// Adding and removing tasks.
	bool push_back(Task* new_task);	// Adds task by rvalue move.
	bool remove_task(int index);		// Marks task at specific index as dead.

	// Initialization functions.
	void init(TIM_HandleTypeDef* htim);
	void start();
	TIM_HandleTypeDef* getTimerInst() { return _timer_inst; };

private:
	unsigned long _openTasks = N;

	// Array of objects that extend the Task class.
	Task* _taskList[N];

	TIM_HandleTypeDef* _timer_inst;
};

// Initializers.
template <int N>
void TaskMaster<N>::init(TIM_HandleTypeDef* htim)
{
	_timer_inst = htim;
}

template <int N>
void TaskMaster<N>::start()
{
	// Start the interrupt routine of the timer.
	HAL_TIM_Base_Start_IT(_timer_inst);
}

template <int N>
TaskMaster<N>::TaskMaster()
{

}

template <int N>
TaskMaster<N>::~TaskMaster()
{

}

// Task Scheduling utility functions.
template <int N>
void TaskMaster<N>::update()
{
	for(int i = 0; i < N; i++)
	{
		// Update task only if its still alive.
		if (_taskList[i] == nullptr) continue;

		if (_taskList[i]->getState() >= 0 )
		{

			// This task has expired. Mark task as ready to run.
			if(_taskList[i]->getDelay() == 0)
			{
				_taskList[i]->setState(1);						// Marked as ready to run.
				_taskList[i]->setDelay(_taskList[i]->getPeriod());
			} else {
				_taskList[i]->setDelay(_taskList[i]->getDelay() - 1);					// Counting down delay time.
			}
		}
	}
}

template <int N>
void TaskMaster<N>::runNextTask()
{
	for(int i = 0; i < N; i++)
	{
		// Check if task is ready to run and has a valid pointer to an execution function.
		if (_taskList[i] == nullptr) continue;

		if (_taskList[i]->getState() == 1 )
		{
			_taskList[i]->setState(2);							// Mark task as running.
			_taskList[i]->taskFunction();						// Run the overloaded task function.
			_taskList[i]->setState(0);							// Mark as idle.

			// Task has finished running.

			// Mark task as dead if it was a one-shot task.
			if (_taskList[i]->getPeriod() == 0)
			{
				_taskList[i]->setState(-1);					// Mark task as dead.
				_openTasks++;
			}

		}
	}
}

// Adding and removing tasks.
template <int N>
bool TaskMaster<N>::push_back(Task* new_task)
{
	for(int i = 0; i < N; i++)
	{
		// If task pointer has not been assigned, lets use that one before trying to dereference a nullptr.
		if (_taskList[i] == nullptr)
		{
			_taskList[i] = new_task;
			_openTasks--;
			return true;
		}

	}

	// All tasks are occupied, increase number of allowable tasks in task_constructor.
	return false;
}

template <int N>
bool TaskMaster<N>::remove_task(int index)
{
	if (index < N)
	{
		if (_taskList[index] == nullptr) return false;

		// Make pointer null.
		_taskList[index] == nullptr;
		_openTasks++;
		return true;
	}

	// Index out of range!
	return false;
}

#endif /* INC_TASKMASTER_HPP_ */
