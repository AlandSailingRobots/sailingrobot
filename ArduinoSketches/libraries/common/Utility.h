//
// Created by dkarlsso on 4/12/18.
//

#ifndef SAILINGROBOT_UTILITY_H
#define SAILINGROBOT_UTILITY_H

#include <stdint.h>

class Utility {
public:
    static double mapInterval(double val, double fromMin, double fromMax, double toMin, double toMax);

    static uint64_t calcSizeOfBytes(int noOfBytes);
};


#endif //SAILINGROBOT_UTILITY_H
