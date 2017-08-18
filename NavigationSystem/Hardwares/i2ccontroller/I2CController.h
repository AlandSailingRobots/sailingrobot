/****************************************************************************************
 *
 * File:
 * 		I2CController.h
 *
 * Purpose:
 *		The I2CController provides a thread safe way of accessing I2C devices. Before
 *		calling I2C operations, one must call I2CController::beginTransmission() to
 *		ensure it is safe to access the bus, if this function is not called the I2C
 *		operation functions will return -1.
 *
 * Developer Notes:
 * 		TODO - Jordan: 	Add a timer system so that I2CControllers lose their lock after x
 * 						time so that the whole bus isn't held up when someone forgets to
 * 						not end the transmission.
 * 		TODO - Jordan:	Consider renaming to I2CDevice instead of I2CController as this
 * 						class actually reflects an individual I2CDevice.
 *
 *
 ***************************************************************************************/


#pragma once

#include <vector>
#include <stdint.h>
#include <string>
#include <mutex>

class I2CController {

	public:
		I2CController();
		~I2CController();

		///----------------------------------------------------------------------------------
		/// Initialises the I2C Controller, and attempts to acquire a handle to a specific
		/// I2C device, based on the deviceAddress.
		///----------------------------------------------------------------------------------
		bool init(const int deviceAddress);

		///----------------------------------------------------------------------------------
		/// A simple I2C write, a wrapper for wiringPI wiringPiI2CWrite(int, int).
		///
		/// @param data 			The byte to write to the device.
		///
		/// @returns				Returns true if the operation was a success.
		///----------------------------------------------------------------------------------
		bool I2Cwrite(uint8_t data);

		///----------------------------------------------------------------------------------
		/// A simple I2C write, a wrapper for wiringPI wiringPiI2CWrite(int, int).
		///
		/// @param data 			The byte to write to the device.
		///
		/// @returns				Returns true if the operation was a success.
		///----------------------------------------------------------------------------------
		bool writeReg(uint8_t reg, uint8_t data2);

		///----------------------------------------------------------------------------------
		/// A simple I2C read, a wrapper for wiringPI wiringPiI2CRead(int)
		///
		/// @returns 				Returns a byte from the read, or a -1 if there was a
		///							error.
		///----------------------------------------------------------------------------------
		int I2Cread();

		///----------------------------------------------------------------------------------
		/// A simple I2C read from a particular register, a wrapper for wiringPI
		/// wiringPiI2CReadReg8(int, int)
		///
		/// @returns 				Returns a byte from the read, or a -1 if there was a
		///							error.
		///----------------------------------------------------------------------------------
		int readReg(int regAddress);

		///----------------------------------------------------------------------------------
		/// Reads a block of bytes from the I2C device. The block will not read more than 32
		/// bytes.
		///
		/// @param block			Pointer to the byte block.
		/// @param size				Number of bytes to read, a maximum of 32 bytes will be
		///							read.
		///
		/// @returns 				Returns a byte from the read, or a -1 if there was a
		///							error.
		///----------------------------------------------------------------------------------
		int readBlock(uint8_t* block, uint8_t size);
		int writeBlock(uint8_t* block, uint8_t size);

		///----------------------------------------------------------------------------------
		/// Begins an I2C transmission, this should be called before any I2C operations are
		/// used.
		///----------------------------------------------------------------------------------
		void beginTransmission();

		///----------------------------------------------------------------------------------
		/// Ends an I2C transmission, this called by called after any I2C operations are
		/// completed.
		///----------------------------------------------------------------------------------
		void endTransmission();

	//private:
	protected:
																// Shared by all I2C Controller, this is the lock for
		static std::mutex m_mutex;															// the bus.
		bool m_Locked;					// Indicates if this I2C Controller owns the lock.
		int m_DeviceFD;					// File descriptor for this I2C Device.
		int m_busNr;
};
