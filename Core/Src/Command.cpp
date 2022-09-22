/*
 * Command.cpp
 *
 *  Created on: Sep 19, 2022
 *      Author: heapr0ll
 */

#include "Command.hpp"

command::command(std::string name, bool (*pntra)(), bool(*pntrb)(std::string& s))
{
	this->name = name;
	runa = pntra;
	runb = pntrb;
}

command::command( std::string name, bool (Task::*tsk_pntr_a)(), bool (Task::*tsk_pntr_b)(std::string& s) )
{
	this->name = name;
	runa = (bool (*)())tsk_pntr_a;
	runb = (bool (*)(std::string&))tsk_pntr_b;
}

command::~command()
{

}
