/*
 * Console.cpp
 *
 *  Created on: Sep 19, 2022
 *      Author: heapr0ll
 */

#include "Console.hpp"

// Our singleton instance of the console!
Console cs;

// Callback that handles UART events.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (consoleInput.raw_byte == '\r' || consoleInput.raw_byte == '\n')
	{
		uint8_t tmp[2] = {'\r', '\n'};
		HAL_UART_Transmit(&huart1, tmp, 2, 100);
	} else if (consoleInput.raw_byte == 0x08)												// Handle the case of a backspace being typed.
	{
		// Move the cursor back, print a space, then move the cursor back again.
		if (consoleInput.numReadBytes > 0)
		{
			uint8_t tmp2 = 0x08;	// backspace
			uint8_t tmp1 = ' ';		// space
			HAL_UART_Transmit(&huart1, &tmp2, 1, 100);
			HAL_UART_Transmit(&huart1, &tmp1, 1, 100);
			HAL_UART_Transmit(&huart1, &tmp2, 1, 100);
		}

	}else{																			// Echo back out whatever the user types!

		HAL_UART_Transmit(&huart1, (uint8_t *)(&consoleInput.raw_byte), 1, 100);
	}

	// Copy data into raw_input until a new line character is received, or the buffer overflows.
	// It is up to the consumer of this buffer to mark the struct as NOT ready again after they have copied the buffer data.
	if ( (consoleInput.ready == false) && (consoleInput.numReadBytes < 300) )		// Leaving space for one last 0 byte so this can be interpreted as a string!
	{
		// Got a new line character, this is the end of a valid command.
		// Mark buffer as ready.
		if (consoleInput.raw_byte == '\r' || consoleInput.raw_byte == '\n')
		{
			consoleInput.ready = true;

		} else if (consoleInput.raw_byte == 0x08)						// Handling backspaces!
		{
			// Decrease the count of written bytes, and write a null byte in that position.
			if (consoleInput.numReadBytes > 0)
			{
				consoleInput.numReadBytes -= 1;
				consoleInput.raw_input[consoleInput.numReadBytes] = 0;
			}

		}else{
			consoleInput.raw_input[consoleInput.numReadBytes] = consoleInput.raw_byte;
			consoleInput.numReadBytes++;
		}
	}

	// Set off another DMA transaction.
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)(&consoleInput.raw_byte), 1);
}

Console::Console()
{
	loadCmnds();
}

Console::~Console()
{

}

bool Console::isCommand(std::string &cmd)
{
	if(cmnds.count(cmd) < 1)
	{
		return false;
	}else{
		return true;
	}
}

bool Console::isTask(std::string &cmd)
{
	if(task_cmnds.count(cmd) < 1)
	{
		return false;
	}else{
		return true;
	}
}

void Console::loop()
{
	if (isNewCommandReady())
	{
		std::string new_cmd(consoleInput.raw_input);
		memset(consoleInput.raw_input, 0, sizeof(consoleInput.raw_input));		// Wipe the raw buffer, get it ready to read new data.
		consoleInput.numReadBytes = 0;
		consoleInput.ready = false;											// Green light for interrupt to start dumping data into raw_input buffer!

		if (new_cmd.length() > 0)
		{

			// The help command must be implemented here as I can't access console map from inside sys_cmnds.hpp
			if (new_cmd == "help")
			{
				// Show all possible system commands
				printAllCommands();
			}else if (!exec(new_cmd))
			{
				print("\'" + new_cmd + "\'");
				println(" is an unrecognized command!");
				println("Use 'help' to show all valid terminal commands.");
			}

		}

		printCursor();
	}
}

void Console::start()
{
	startUART();
	printBanner();
	printCursor();
}

bool Console::exec(std::string &cmd)
{
	std::string command = bfrspc(cmd); 		// Separate string at space

	if (isCommand(command) || isTask(command)) { 				// Check if input is even a command

		std::string args = aftspc(cmd); 	// Separate arguments from command

		if (isCommand(command))				// Its a system command.
		{
			if (args.length() > 0) {
				cmnds.find(command)->second.runb(args);	// Execute command with arguments.
			} else {
				cmnds.find(command)->second.runa();		// Execute command without arguments.
			}
		}else{

			// Its a task!
			if (args.length() > 0) {
				task_cmnds.find(command)->second->consoleFuncb(args);
			} else {
				task_cmnds.find(command)->second->consoleFunca();
			}

		}

		return true;

	} else {
		return false;
	}
}

void Console::loadCmnds()
{
	// Loading all commands, must be done manually for now.
	cmnds.insert( {echo.getName(),  echo} );
	cmnds.insert( {clear.getName(), clear} );
	cmnds.insert( {banner.getName(), banner} );
	cmnds.insert( {help.getName(), help} );
}

bool Console::isNewCommandReady()
{
	return consoleInput.ready;
}

void Console::printAllCommands()
{
	println("System Commands:\tSystem Tasks:");

	unsigned int small_cnt = 0;

	if (cmnds.size() >= task_cmnds.size())
	{
		for (auto it = cmnds.cbegin(); it != cmnds.cend(); it++)
		{
			print(it->first);
			print("\t\t\t");

			if (small_cnt < task_cmnds.size())
			{
				auto tsk_it = task_cmnds.cbegin();
				std::advance(tsk_it, small_cnt);
				print(tsk_it->first);
			}

			println("");
			small_cnt++;
		}
	}else{							// Task list is bigger than command list.

		for (auto it = task_cmnds.cbegin(); it != task_cmnds.cend(); it++)
		{
			if (small_cnt < cmnds.size())
			{
				auto cmnd_it = cmnds.cbegin();
				std::advance(cmnd_it, small_cnt);
				print(cmnd_it->first);
				print("\t\t\t");
			}else{
				print("\t\t\t\t");
			}

			println(it->first);
			small_cnt++;
		}

	}
}

void Console::addCommand(std::string& name, bool (*pntra)(), bool(*pntrb)(std::string& s))
{
	cmnds.insert( {name, command(name, pntra, pntrb)} );
}

void Console::addCommandByTask(Task* tsk)
{
	// Call the addCommand function.
	task_cmnds.insert( {tsk->getName(), tsk} );
}
