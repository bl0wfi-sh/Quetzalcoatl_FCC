/*
 * Command.hpp
 *
 *  Created on: Sep 19, 2022
 *      Author: heapr0ll
 */

#ifndef INC_COMMAND_HPP_
#define INC_COMMAND_HPP_

#include "Task.hpp"
#include <string>

class command
{

public:
	command(std::string name, bool (*pntra)(), bool(*pntrb)(std::string& s));
	command( std::string name, bool (Task::*tsk_pntr_a)(), bool (Task::*tsk_pntr_b)(std::string& s) );
	~command();

	// Command functions to be called.
	bool (*runa)();
	bool (*runb)(std::string& s);

	// Getters
	std::string getName() { return name; }

private:
	std::string name;

};



#endif /* INC_COMMAND_HPP_ */
