/****************************************************************************************
 *
 * File:
 * 		I2CControllerSuite.h
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
 *	init 							write
 *	readBlock						writereg
 *	beginTransmission				read
 *	endTransmission					readReg
 *
 *	Functions that does not exist but still have tests:
 *
 *	writeBlock
 *
 ***************************************************************************************/

#pragma once

#include <stdint.h>
#include <unistd.h>
#include <cstdlib>
#include <iostream>
#include "Hardwares/i2ccontroller/I2CController.h"
#include "Tests/cxxtest/cxxtest/TestSuite.h"
// For std::this_thread
#include <chrono>
#include <thread>

#define BLOCK_READ_SIZE 9
#define DATA_ID_LOC 0
#define INIT_ID 0xFF
#define READ_COMMAND 0

#define ARDUINO_ADDRESS 0x07

class I2CControllerSuite : public CxxTest::TestSuite {
   public:
    int testCount = 0;
    I2CController I2C;

    void setUp() {
        I2C.init(ARDUINO_ADDRESS);
        testCount++;
    }

    void tearDown() {}

    void test_initRead() {
        I2C.beginTransmission();

        uint8_t block[BLOCK_READ_SIZE];
        I2C.readBlock(block, INIT_ID);
        uint8_t initID = block[DATA_ID_LOC];

        I2C.endTransmission();

        TS_ASSERT_EQUALS(initID, INIT_ID);
    }

    void test_read() {
        I2C.beginTransmission();

        std::cout << I2C.I2Cread();

        I2C.endTransmission();
    }

    void test_readBlock() {
        std::cout << "\nstart read block...\n";
        uint8_t data[BLOCK_READ_SIZE];
        int val;

        I2C.beginTransmission();
        int readBlockRet = I2C.readBlock(data, READ_COMMAND);
        I2C.endTransmission();

        std::cout << "Return Val: " << readBlockRet << "\n";

        for (int i = 0; i < readBlockRet; i++) {
            val = data[i];
            std::cout << "Val " << i << ": " << val << "\n";
        }
    }

    // void test_writeBlock()
    // {
    // 	std::cout << "\nstart write block...\n";
    // 	uint8_t block[12];
    // 	uint8_t size = 9;
    // 	int command = 20;

    // 	std::cout << "\ncommand =: " << command << "\n";
    // 	for (int i = 0; i < size; i++)
    // 	{
    // 		block[i]=i;
    // 		std::cout << "Val " << i << ": " << i << "\n";
    // 	}
    // 	uint8_t dataID = 0x77;
    // 	I2C.beginTransmission();
    // 	int retVal = I2C.writeBlock(block, size, dataID);
    // 	I2C.endTransmission();
    // 	TS_ASSERT(retVal != -1);
    // }
};
