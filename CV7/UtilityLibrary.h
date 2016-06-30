/*
 * UtilityLibrary.h
 *
 *  Created on: Mar 31, 2015
 *      Author: sailbot
 */

#ifndef CV7_UTILITYLIBRARY_H_
#define CV7_UTILITYLIBRARY_H_

#include <map>
#include <string.h>

class UtilityLibrary {
public:
	static std::map<std::string,float> parseString(const char* buffer);
};

#endif /* CV7_UTILITYLIBRARY_H_ */
