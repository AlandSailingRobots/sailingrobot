// #include "WindowsSerialDataLink.h"

// WindowsSerialDataLink::WindowsSerialDataLink(std::string port, uint16_t baudRate)
// : DataLink(true), m_port(port), mbaudRate(baudRate)
// {

// }

// WindowsSerialDataLink::~WindowsSerialDataLink()
// {
//     CloseHandle(m_hSerial);
// }

// bool WindowsSerialDataLink::initialise(uint16_t frameSize)
// {
//     fprintf(stderr, "Opening serial port...");
//     m_hSerial = CreateFile(m_port,
//                 GENERIC_READ|GENERIC_WRITE,     // access ( read and write)
//                 0,                              // (share) 0:cannot share the COM port 
//                 0,                              // security  (None)    
//                 OPEN_EXISTING,                  // creation : open_existing
//                 FILE_FLAG_OVERLAPPED,           // we want overlapped operation
//                 0);                             // no templates file for COM port...

//     if(m_hSerial == INVALID_HANDLE_VALUE)
//     {
//         fprintf(stderr, "Error\n");
//         return false;
//     }
//     else
//     {
//         fprintf(stderr, "OK\n");

//          // Set device parameters (baud, 1 start bit,
//         // 1 stop bit, no parity)
//         m_dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
//         if (GetCommState(hSerial, &dcbSerialParams) == 0)
//         {
//             fprintf(stderr, "Error getting device state\n");
//             CloseHandle(hSerial);
//             return false;
//         }

//         m_dcbSerialParams.BaudRate = m_baudRate;
//         m_dcbSerialParams.ByteSize = 8;
//         m_dcbSerialParams.StopBits = ONESTOPBIT;
//         m_dcbSerialParams.Parity = NOPARITY;
//         if(SetCommState(hSerial, &dcbSerialParams) == 0)
//         {
//             fprintf(stderr, "Error setting device parameters\n");
//             CloseHandle(hSerial);
//             return false;
//         }

//         // Set COM port timeout settings
//         m_timeouts.ReadIntervalTimeout = MAXDWORD; //maybe not initialized?
//         m_timeouts.ReadTotalTimeoutConstant = 0;
//         m_timeouts.ReadTotalTimeoutMultiplier = 0;
//         m_timeouts.WriteTotalTimeoutConstant = 0;
//         m_timeouts.WriteTotalTimeoutMultiplier = 0;
//         if(SetCommTimeouts(hSerial, &timeouts) == 0)
//         {
//             fprintf(stderr, "Error setting timeouts\n");
//             CloseHandle(hSerial);
//             return false;
//         }
//     }

//     return true;
// }

// void WindowsSerialDataLink::writeData(const uint8_t* data, uint8_t size)
// {
//     DWORD byteswritten;

//     WriteFile(m_hSerial, data, size, &byteswritten, NULL);
// }

// void WindowsSerialDataLink::readData(uint8_t* data, uint8_t size)
// {
//     DWORD dwRead;
//     DWORD dwCommModemStatus;
//     Byte bytes_to_read[size];

//     SetCommMask(m_hSerial, EV_RXCHAR | EV_ERR);       //receive character event
//     WaitCommEvent(m_hSerial, &dwCommModemStatus, 0);  //wait for character
//     if (dwCommModemStatus & EV_RXCHAR)  
//     {
//         while(dwRead > 0)
//         {
//             ReadFile(m_hSerial, &bytes_to_read, size, &dwRead, 0);
//         }
//     }
//     else if (dwCommModemStatus & EV_ERR) 
//     {
//         data = 0x101; //error
//     }
// }