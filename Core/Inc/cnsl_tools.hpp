/*
 * cnsl_tools.hpp
 *
 *  Created on: Sep 19, 2022
 *      Author: heapr0ll
 */

#ifndef INC_CNSL_TOOLS_HPP_
#define INC_CNSL_TOOLS_HPP_

#include "main.h"
#include <cstring>
#include <string>

const int MAX_LINE_LENGTH = 1000;
const std::string cursor = "> ";
const std::string wlcm =
		"\r\n"
		"                                                    ,   ,                                \r\n"
		"                                                    $,  $,     ,                         \r\n"
		"                                                    \"ss.$ss. .s'                         \r\n"
		"                                            ,     .ss$$$$$$$$$$s,                        \r\n"
		"                                            $. s$$$$$$$$$$$$$$`$$Ss                      \r\n"
		"                                            \"$$$$$$$$$$$$$$$$$$o$$$       ,              \r\n"
		"                                           s$$$$$$$$$$$$$$$$$$$$$$$$s,  ,s               \r\n"
		"                                          s$$$$$$$$$\"$$$$$$\"\"\"\"$$$$$$\"$$$$$,             \r\n"
		"                                          s$$$$$$$$$$s\"\"$$$$ssssss\"$$$$$$$$\"             \r\n"
		"                                         s$$$$$$$$$$'         `\"\"\"ss\"$\"$s\"\"              \r\n"
		"                                         s$$$$$$$$$$,              `\"\"\"\"\"$  .s$$s        \r\n"
		"                                         s$$$$$$$$$$$$s,...               `s$$'  `       \r\n"
		"                                     `ssss$$$$$$$$$$$$$$$$$$$$####s.     .$$\"$.   , s-   \r\n"
		"                                       `\"\"\"\"$$$$$$$$$$$$$$$$$$$$#####$$$$$$\"     $.$'    \r\n"
		"               Tiny Terminal                 \"$$$$$$$$$$$$$$$$$$$$$####s\"\"     .$$$|     \r\n"
		"                   C++                         \"$$$$$$$$$$$$$$$$$$$$$$$$##s    .$$\" $    \r\n"
		"                                               $$\"\"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\"   `    \r\n"
		"                                              $$\"  \"$\"$$$$$$$$$$$$$$$$$$$$S\"\"\"\"'         \r\n"
		"                                         ,   ,\"     '  $$$$$$$$$$$$$$$$####s             \r\n"
		"                                         $.          .s$$$$$$$$$$$$$$$$$####\"            \r\n"
		"                             ,           \"$s.   ..ssS$$$$$$$$$$$$$$$$$$$####\"            \r\n"
		"                             $           .$$$S$$$$$$$$$$$$$$$$$$$$$$$$#####\"             \r\n"
		"                             Ss     ..sS$$$$$$$$$$$$$$$$$$$$$$$$$$$######\"\"              \r\n"
		"                              \"$$sS$$$$$$$$$$$$$$$$$$$$$$$$$$$########\"                  \r\n"
		"                       ,      s$$$$$$$$$$$$$$$$$$$$$$$$#########\"\"'                      \r\n"
		"                       $    s$$$$$$$$$$$$$$$$$$$$$#######\"\"'      s'         ,           \r\n"
		"                       $$..$$$$$$$$$$$$$$$$$$######\"'       ....,$$....    ,$            \r\n"
		"                        \"$$$$$$$$$$$$$$$######\"' ,     .sS$$$$$$$$$$$$$$$$s$$            \r\n"
		"                          $$$$$$$$$$$$#####\"     $, .s$$$$$$$$$$$$$$$$$$$$$$$$s.         \r\n"
		"               )          $$$$$$$$$$$#####'      `$$$$$$$$$###########$$$$$$$$$$$.       \r\n"
		"              ((          $$$$$$$$$$$#####       $$$$$$$$###\"       \"####$$$$$$$$$$      \r\n"
		"              ) )         $$$$$$$$$$$$####.     $$$$$$###\"             \"###$$$$$$$$$   s'\r\n"
		"             (   )        $$$$$$$$$$$$$####.   $$$$$###\"                ####$$$$$$$$s$$' \r\n"
		"             )  ( (       $$\"$$$$$$$$$$$#####.$$$$$###'                .###$$$$$$$$$$\"   \r\n"
		"             (  )  )   _,$\"   $$$$$$$$$$$$######.$$##'                .###$$$$$$$$$$     \r\n"
		"             ) (  ( ).         \"$$$$$$$$$$$$$#######,,,.          ..####$$$$$$$$$$$\"     \r\n"
		"            (   )$ )  )        ,$$$$$$$$$$$$$$$$$$####################$$$$$$$$$$$\"       \r\n"
		"            (   ($$  ( )     _sS\"  `\"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$S$$,       \r\n"
		"             )  )$$$s ) )  .      .   `$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\"'  `$$      \r\n"
		"              (   $$$Ss/  .$,    .$,,s$$$$$$##S$$$$$$$$$$$$$$$$$$$$$$$$S""        '      \r\n"
		"                ))_$$$$$$$$$$$$$$$$$$$$$$$##\"  $$        `$$.        `$$.                \r\n"
		"                    `\"S$$$$$$$$$$$$$$$$$#\"      $          `$          `$                \r\n"
		"                        `\"\"\"\"\"\"\"\"\"\"\"\"\"'         '           '           '\r\n"
		"  @@@@@@   @@@  @@@ @@@@@@@@ @@@@@@@ @@@@@@@@  @@@@@@  @@@       @@@@@@@  @@@@@@   @@@@@@  @@@@@@@ @@@     \r\n"
		" @@!  @@@  @@!  @@@ @@!        @@!        @@! @@!  @@@ @@!      !@@      @@!  @@@ @@!  @@@   @@!   @@!     \r\n"
		" @!@  !@!  @!@  !@! @!!!:!     @!!      @!!   @!@!@!@! @!!      !@!      @!@  !@! @!@!@!@!   @!!   @!!     \r\n"
		" !!:!!:!:  !!:  !!! !!:        !!:    !!:     !!:  !!! !!:      :!!      !!:  !!! !!:  !!!   !!:   !!:     \r\n"
		"  : :. :::  :.:: :  : :: :::    :    :.::.: :  :   : : : ::.: :  :: :: :  : :. :   :   : :    :    : ::.: :\r\n"
		"\r\n";

extern UART_HandleTypeDef huart1;

static struct RawConsoleInput
{
	char raw_input[301] = {0};
	volatile char raw_byte = {0};
	uint16_t numReadBytes = 0;
	bool ready = false;
} consoleInput;

//Utility functions to work with UART hardware!
static void startUART()
{
	HAL_UART_Receive_DMA(&huart1, (uint8_t *)(&consoleInput.raw_byte), 1);
}

// Print string with new line.
static void println(std::string s)
{
	HAL_StatusTypeDef hstatus = HAL_UART_Transmit(&huart1, (uint8_t *)s.c_str(), s.length(), HAL_MAX_DELAY);
	uint8_t tmp[2] = {'\r', '\n'};
	hstatus = HAL_UART_Transmit(&huart1, tmp, 2, HAL_MAX_DELAY);
	if (hstatus != HAL_OK)
	{
		/* Handle error here! */
	}
}

// Print string without new line.
static void print(std::string s)
{
	HAL_StatusTypeDef hstatus = HAL_UART_Transmit(&huart1, (uint8_t *)s.c_str(), s.length(), HAL_MAX_DELAY);
	if (hstatus != HAL_OK)
	{
		/* Handle error here! */
	}
}

static std::string bfrspc(std::string s)
{
    // Returns the string up to the first space
    unsigned int i = 0;
    const char* cs = s.c_str();
    while (cs[i] != ' ' && i < s.length()) {
        i++;
    }
    return s.substr(0,i);
}

static std::string aftspc(std::string s)
{
    // Returns the string after the first space
    unsigned int i = 0;
    const char* cs = s.c_str();
    while (cs[i] != ' ' && i < s.length()) {
        i++;
    }
    if (i >= s.length()) {
        return "";
    }
    else {
        return s.substr(i+1,s.length()-i);
    }
}


#endif /* INC_CNSL_TOOLS_HPP_ */
