/****************************************************************************************
 *
 * File:
 * 		WindowsSerialDataLink.h
 *
 * Purpose:
 *		
 *
 * Developer Notes:
 *
 ***************************************************************************************/

//  #pragma once

//  #include "DataLink.h"
//  #include <windows.h>
//  #include <stdio.h>

//  class WindowsSerialDataLink : public DataLink
//  {
//      public:
//         WindowsSerialDataLink(std::string port, uint16_t baudRate);

//         virtual ~WindowsSerialDataLink();

//         ///----------------------------------------------------------------------------------
//         /// Initialises the data link. This function should ensure that the hardware layer
//         /// is up and functioning.
//         ///----------------------------------------------------------------------------------
//         virtual bool initialise(uint16_t frameSize);

//         ///----------------------------------------------------------------------------------
//         /// Sends an AT command over the data link.
//         ///----------------------------------------------------------------------------------
//         virtual std::string sendATCommand(std::string command, uint16_t responseSize);

//      protected:
//      	virtual void writeData(const uint8_t* data, uint8_t size);
//         virtual void readData(uint8_t* data, uint8_t size);
//         virtual int readByte();
//         virtual void writeByte(uint8_t byte);
//         virtual bool dataAvailable();

//      private:
//         std::string     m_port;
//         uint16_t		m_baudRate;
//         Handle          m_hSerial;
//         DCB             m_dcbSerialParam = {0};
//         COMMTIMEOUTS    m_timeouts = {0};
//  };