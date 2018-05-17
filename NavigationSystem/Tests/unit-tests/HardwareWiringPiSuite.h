
/****************************************************************************************
 *
 * File:
 * 		HardwareWiringPiSuite.h
 *
 * Purpose:
 *
 *
 * Developer Notes:
 *
 *							12.4.17 JM
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	wiringPiI2CSetup 				wiringPiI2CRead
 *	wiringPiI2CReadBlock			wiringPiI2CReadReg8
 *									wiringPiI2CReadReg16
 *									wiringPiI2CSetupInterface
 *									wiringPiI2CWrite
 *									wiringPiI2CWriteReg8
 *									wiringPiI2CWriteReg16
 *
 *	Functions that does not exist but still have tests:
 *
 *	wiringPiI2CWriteBlock
 *	wiringPiI2CWriteI2CBlock
 *
 ***************************************************************************************/

#pragma once

#include <stdint.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Libs/wiringPi/wiringPi/wiringPiI2C.h"
// For std::this_thread
#include <chrono>
#include <thread>

class HardwareWiringPiSuite : public CxxTest::TestSuite {
   public:
    int testCount = 0;
    int deviceAddress;
    int deviceFD;

    void setUp() {
        deviceAddress = 0x07;
        deviceFD = wiringPiI2CSetup(deviceAddress);
        testCount++;
    }

    void tearDown() {}

    void test_wiringPiReadBlock() {
        std::cout << "\nstart read block...\n";
        uint8_t block[12];
        uint8_t size = 9;
        int val;
        int readBlockRet = wiringPiI2CReadBlock(deviceFD, (char*)block, 0);

        std::cout << "Return Val: " << readBlockRet << "\n";

        TS_ASSERT(readBlockRet != -1);

        for (int i = 0; i < size; i++) {
            val = block[i];
            std::cout << "Val " << i << ": " << val << "\n";
        }
    }

    // void test_wiringPiWriteBlock()
    // {
    // 	std::cout << "\nstart read block...\n";
    // 	uint8_t block[12];
    // 	uint8_t size = 9;
    // 	int command = 20;

    // 	std::cout << "\ncommand =: " << command << "\n";
    // 	for (int i = 0; i < size; i++)
    // 	{
    // 		block[i]=i;
    // 		std::cout << "Val " << i << ": " << i << "\n";
    // 	}

    // 	int writeBlockRet = wiringPiI2CWriteBlock(deviceFD, command, (char*)block, size);
    // 	std::cout << "Return Val: " << writeBlockRet << "\n";

    // }

    // void test_wiringPiWriteI2CBlock()
    // {
    // 	std::cout << "\nstart read block...\n";
    // 	uint8_t block[12];
    // 	uint8_t size = 9;
    // 	int command = 20;

    // 	std::cout << "\ncommand =: " << command << "\n";
    // 	for (int i = 0; i < size; i++)
    // 	{
    // 		block[i]=i;
    // 		std::cout << "Val " << i << ": " << i << "\n";
    // 	}

    // 	int writeBlockRet = wiringPiI2CWriteI2CBlock(deviceFD, command, (char*)block, size);
    // 	std::cout << "Return Val: " << writeBlockRet << "\n";

    // }
};
