/****************************************************************************************
*
* File:
* 		NMEA2000Bus.h
*
* Purpose:
*
*
* Developer Notes:
*   Transferred from old CanBus library. Unused but good to have.
*   Uses arduino canbus so some changes need to be made for this to be used
*
*
***************************************************************************************/

#ifndef NMEA2000BUS_H
#define NMEA2000BUS_H

#include <tuple>
#include "../../../ArduinoSketches/libraries/Canbus/mcp2515.h"
#include "../../../ArduinoSketches/libraries/Canbus/Canbus.h"

typedef std::tuple<uint32_t,uint8_t> IDsID;

class NMEA2000Bus
{
public:
    NMEA2000Bus() {};
    bool Init(int SPISpeed);
    uint8_t CheckForMessages();

    void CreateN2kMsg(N2kMsg &NMsg, uint8_t Dest);
    bool SendMessage(N2kMsg &NMsg);
    int GetN2kMsg();

    bool IsFastPackage(const N2kMsg &Msg);
    bool ParseFastPKG(CanMsg &Msg, N2kMsg &NMsg);

    std::map<IDsID, N2kMsg> FastPKG_;
    std::map<IDsID, int> BytesLeft_;		//< <MessageId, sequence id>, Bytes left>
    std::vector<N2kMsg> MessageQue_;
    std::vector<CanMsg> MessageQue2_;

    CanbusClass Canbus;


    ///Address claim
    int SourceAddress_ = 1;
    uint32_t UniqueNumber_ = 1; 		//21bits
    uint16_t ManufacturerCode_ = 2017; 	//11bits
    uint8_t DeviceInstance_ = 0;
    uint8_t DeviceFunction_ = 150; 		//Bridge
    uint8_t DeviceClass_ = 25;			//Inter/Intranetwork Device
    uint8_t IndustryCode_ = 4;			//marine
    uint8_t SystemInstance_ = 0;
    bool ArbitraryAddressCapable_ = false;

};

#endif