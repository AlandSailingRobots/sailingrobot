#include <string>
#include <cstring>
#include <cstdint>
#include <unistd.h>
#include "../SystemServices/Logger.h"
#include "AtlasScientific.h"

int long_timeout = 1500000;         //the timeout needed to query readings and calibrations
int short_timeout = 500000;         //timeout for regular commands

int AtlasI2C::ASWRITE(std::string cmd)
{
	// appends the null character and sends the string over I2C
	cmd += "\00";
	uint8_t size = sizeof(char)*cmd.size();

	uint8_t *test = new uint8_t[31];            //convert string to uint8_t*
	memcpy(test, cmd.c_str(), size);

    int response = writeBlock(test, size);
    delete test;
	return response;
}
std::string AtlasI2C::ASREAD()			//empty string is an error
{
	// reads a specified number of bytes from I2C, then parses and displays the result
	uint8_t *response = new uint8_t[31];
	char response2[31] = {0};

	if(readBlock(response, 31) < 1)         // read from the board
	{
		Logger::error("Error could not read I2C");
		return("");
	}
	for(int i = 1; i < 31; ++i)
    {
		response2[i-1] = response[i] & 0x7f;		//set MSB of all characters to 0
    }
	if (response[0] == 1)            				//if the response isn't an error
	{
		return(std::string(response2));
	}
	else
	{
		Logger::error("Error " + std::to_string((int)response[0]));
		return("");
	}
}

bool StartsWith(std::string input, std::string text)
{
	size_t len = text.size();
	if(input.size() < len)
    {
		return false;
    }
	for(size_t i = 0; i < len; ++i)
	{
		if(std::toupper(input[i]) != std::toupper(text[i]))
		{
			return false;
		}
	}
	return true;
}
std::string AtlasI2C::ASQUERY(std::string text)
{
	// write a command to the board, wait the correct timeout, and read the response
	if(ASWRITE(text) == -1)
	{
		return("");
	}

	// the read and calibration commands require a longer timeout
	if(StartsWith(text, "R") || StartsWith(text, "CAL"))
    {
		usleep(long_timeout);
    }
	else if(StartsWith(text, "SLEEP"))
    {
		return "sleep mode";
    }
	else
    {
		usleep(short_timeout);
    }
	return ASREAD();
}
