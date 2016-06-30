/*
 * UtilityLibrary.cpp
 *
 *  Created on: Mar 31, 2015
 *      Author: sailbot
 */
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>
#include <numeric>
#include "UtilityLibrary.h"

using namespace std;

std::map<std::string,float> UtilityLibrary::parseString(const char* buffer) {
	float windDirection = 0;
	float windSpeed = 0;
	float windTemperature = 0;
	const int IIMWV = 0;
	const int WIXDR = 1;
	bool updated[] = { false, false };
	char * writalbeBuff;
	writalbeBuff = const_cast<char *>(buffer);
	char* split = strtok(writalbeBuff, "$,");

	while (split != NULL) {
		if (strcmp(split, "IIMWV") == 0) {
			split = strtok(NULL, "$,");
			windDirection = atof(split);
			split = strtok(NULL, "$,");
			split = strtok(NULL, "$,");
			windSpeed = atof(split);
			updated[IIMWV] = true;
		} else if (strcmp(split, "WIXDR") == 0) {
			split = strtok(NULL, "$,");
			split = strtok(NULL, "$,");
			windTemperature = atof(split);
			updated[WIXDR] = true;
		}

		if (updated[IIMWV] && updated[WIXDR]) {
			break;
		}
		split = strtok(NULL, "$,");
	}
	if (updated[IIMWV] == false || updated[WIXDR] == false ) {
		throw "UtilLibrary::parseString exception";
	}
	std::map<std::string,float> result;
	result.insert(std::make_pair("windDirection", windDirection));
	result.insert(std::make_pair("windSpeed",windSpeed) );
	result.insert(std::make_pair("windTemperature",windTemperature) );
	return result;
}
