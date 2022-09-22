/*
 * Console.hpp
 *
 *  Created on: Sep 19, 2022
 *      Author: heapr0ll
 */

#ifndef INC_CONSOLE_HPP_
#define INC_CONSOLE_HPP_

#include "main.h"
#include <map>
#include <string>
#include <cstring>
#include "sys_cmnds.hpp"
#include "Task.hpp"

class Console
{
public:
	Console();
	~Console();

	// Delete the copy assignment operator as this is a singleton.
	Console& operator=(const Console& other) = delete;

	// Utility functions for the console.
	void start();

	bool isCommand(std::string &cmd);
	bool isTask(std::string &cmd);

	bool exec(std::string &cmd);
	void printBanner() { println(wlcm); }
	void printCursor() { print(cursor); }
	void loadCmnds();
	bool isNewCommandReady();
	void printAllCommands();

	// Getters.
	std::map<std::string, command>* getCommands() { return &cmnds; }

	// Setters
	void addCommand(std::string& name, bool (*pntra)(), bool(*pntrb)(std::string& s));
	void addCommandByTask(Task* tsk);

	// Main loop thats tried.
	void loop();

private:
	std::map<std::string, command> cmnds;
	std::map<std::string, Task*> task_cmnds;
};



#endif /* INC_CONSOLE_HPP_ */
