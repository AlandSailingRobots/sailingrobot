//
// Created by dkarlsso on 4/12/18.
//


#include "CanUtility.h"

double CanUtility::mapInterval(double val, double fromMin, double fromMax, double toMin, double toMax) {
    return (val - fromMin) / (fromMax - fromMin) * (toMax - toMin) + toMin;
}

uint64_t CanUtility::calcSizeOfBytes(int noOfBytes) {

    const int NO_OF_BITS_PER_BYTE = 8;

    int totalNumberOfBits = noOfBytes * NO_OF_BITS_PER_BYTE;

    uint64_t sizeOfBytes = 1;
    for(int i=0;i<totalNumberOfBits;i++) {
        sizeOfBytes = sizeOfBytes*2;
    }

    return 0;
}
