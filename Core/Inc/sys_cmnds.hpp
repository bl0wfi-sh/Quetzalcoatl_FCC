/*
 * sys_cmnds.hpp
 *
 *  Created on: Sep 19, 2022
 *      Author: heapr0ll
 */

#ifndef INC_SYS_CMNDS_HPP_
#define INC_SYS_CMNDS_HPP_

#include "cnsl_tools.hpp"
#include "Command.hpp"

/*
 * @brief: Help command.
 *  Just a place holder so the help string can be printed as a possible command.
 */
static bool helpa()
{
	return true;
}
static bool helpb(std::string& s)
{
	return true;
}
static command help = command("help", &helpa, &helpb);

/*
 * @brief: Echo command.
 *  Simply echo's back anything you type in.
 */
static bool echoa()
{
	println("echo: Prints a string to the terminal.");
	return true;
}
static bool echob(std::string& s)
{
	println(s);
	return true;
}
static command echo = command("echo", &echoa, &echob);

/*
 * @brief: Clear command.
 *  Clears the terminal window when it gets too messy.
 */
static bool cleara()
{
	print("\x1B[2J");
	return true;
}
static bool clearb(std::string& s)
{
	print("\x1B[2J");
	return true;
}
static command clear = command("clear", &cleara, &clearb);

/*
 * @brief: Banner command.
 *  Prints the banner to the screen.
 */
static bool bannera()
{
	println(wlcm);
	return true;
}
static bool bannerb(std::string& s)
{
	println(wlcm);
	return true;
}
static command banner = command("banner", &bannera, &bannerb);

#endif /* INC_SYS_CMNDS_HPP_ */
