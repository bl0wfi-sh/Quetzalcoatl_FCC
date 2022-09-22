/*
 * Task.hpp
 *
 *  Created on: Sep 16, 2022
 *      Author: heapr0ll
 */

#ifndef INC_TASK_HPP_
#define INC_TASK_HPP_

#include <string>
#include "cnsl_tools.hpp"

class Task
{

private:
	volatile int state;								// -1 - Dead, 0 - Idle, 1 - Ready to run, 2 - Running
	char priority;									// 0-255. 0 - lowest, 255 highest.
	volatile unsigned long delay_tk;				// Delay in ticks of passed timer.
	volatile unsigned long period_tk;				// Period in in ticks of passed timer.
	std::string name;								// Name of task.
	volatile unsigned long runtime_tk;				// Tick count for a single loop of a task.

public:

	// Default Constructor
	// All tasks init as dead tasks on the stack.
	Task();

	// Constructor for Brace Initialization.
	Task(int st, char pri, unsigned long del, unsigned long per);

	/*
	 * @brief Constructor for tasks that define period and delay time in seconds.
	 *
	 * delay_s			- Start delay of task in seconds.
	 * peri_s			- Period of task in seconds. 0 - Means 1 shot task.
	 * mstr_ticks_s		- How many seconds does the master clock tick represent.
	 * pri				- Priority of task.
	 * clbk				- Function pointer for callback function.
	 * dt				- Pointer to data to be used by callback function, if needed.
	 */
	Task(float delay_s, float peri_s, float mstr_tick_s, char pri, std::string nm);

	// Move constructor.
	// Allows the creation of a new Task through rvalues without actually copying the rvalue objects.
	// Ex: Task hello( std::move( Task(x, y, z) ) );
	Task(Task&& other) noexcept;

	// Move assignment operator.
	Task& operator=(Task&& other) noexcept;
	Task& operator=(const Task& other) = delete;	// Disabling copy constructor so you can only move objects.

	// Callback function for Task object within scheduler.
	virtual bool taskFunction()
	{
		return true;
	}

	// Callback functions for Task object within console.
	virtual bool consoleFunca()
	{
		println("Function not implemented by Task!");
		return true;
	}

	virtual bool consoleFuncb(std::string& s)
	{
		println("Function not implemented by Task!");
		return true;
	}

	// Getters and setters
	int getState(){ return state; }
	void setState(int x) { state = x; }

	char getPriority(){ return priority; }

	unsigned long getDelay(){ return delay_tk; }
	void setDelay(unsigned long d){ delay_tk = d; }

	unsigned long getPeriod(){ return period_tk; }

	std::string getName() { return name; }
};


#endif /* INC_TASK_HPP_ */
