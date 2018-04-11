/****************************************************************************************
*
* File:
* 		NMEA2000Bus.cpp
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

#include "NMEA2000Bus.h"


bool NMEA2000Bus::Init(int SPISpeed)
{
    return Canbus.Init(SPISpeed);
}

bool NMEA2000Bus::IsFastPackage(const N2kMsg &NMsg)
{
    switch(NMsg.PGN)
    {
        case 126464: return true;
        case 126996: return true;
        case 65240: return true;
        case 126208: return true;
        case 129038: return true;
        case 129039: return true;
        case 129041: return true;
        case 129793: return true;
        case 129794: return true;
        case 129798: return true;
        case 129802: return true;
        case 129809: return true;
        case 129810: return true;
    }
    return false;
}

int NMEA2000Bus::GetN2kMsg()		//-1: no message avaliable, 0: fast pakage, 1: message in que
{
    CanMsg Msg;
    if(Canbus.GetMessage(&Msg))
    {
        N2kMsg NMsg;
        IdToN2kMsg(NMsg, Msg.id);
        if(IsFastPackage(NMsg))
        {
            bool FullMsg = ParseFastPKG(Msg, NMsg);
            MessageQue2_.push_back(Msg);
            if(FullMsg)							//if we have received the whole message
            {
                MessageQue_.push_back(NMsg);
                return 1;
            }
            else
            {
                return 0;
            }
        }
        else
        {
            NMsg.DataLen = Msg.header.length;		//Single frame message
            NMsg.Data.resize(Msg.header.length);
            for(int i = 0; i < 8; ++i)
            {
                NMsg.Data[i] = Msg.data[i];
            }
            MessageQue_.push_back(NMsg);
            return 1;
        }
    }
    return -1;			//no message to get
}

bool NMEA2000Bus::ParseFastPKG(CanMsg &Msg, N2kMsg &NMsg)		//return true and overwrite the NMsg if we have received the whole message
{
    IDsID Key = IDsID(Msg.id, Msg.data[0]&0xE0);		//message ID and sequence ID
    uint8_t SequenceNumber = Msg.data[0]&0x1F;

    auto it = BytesLeft_.find(Key);
    if(it != BytesLeft_.end())				//have parts of the message already
    {
        #ifdef VERBOSE
                std::cout << "Fast package, SID: "<< (int)(Msg.data[0]&0xE0) <<", SequenceNumber: " << (int)SequenceNumber << ", Bytesleft: " << (int)it->second << std::endl;
        #endif
        int LastByte;
        if(it->second >= 7)		//check if less than 7 bytes left, 1st byte is the sequence ID and Sequence number followed by 7 bytes of data
        {
            LastByte = 8;
        }
        else
        {
            LastByte = it->second +1;
        }

        for(int i = 1; i < LastByte; ++i)
        {
            FastPKG_[Key].Data[6+(SequenceNumber-1)*7 +i] = Msg.data[i];
        }
        it->second -= 7;				//decrease bytes left

        if(it->second <= 0)				//have the whole message
        {
            NMsg = FastPKG_[Key];
            FastPKG_.erase(Key);
            return true;
        }
        else
        {
            return false;
        }
    }
    else									//not found, create new N2kMsg
    {
        uint8_t BytesInMsg = Msg.data[1];
        NMsg.DataLen = BytesInMsg;
        NMsg.Data.resize(BytesInMsg);
        for(int i = 2; i < 8; ++i)
        {
            NMsg.Data[i-2] = Msg.data[i];
        }
        #ifdef VERBOSE
                std::cout << "New fast package, SID: "<< (int)(Msg.data[0]&0xE0) <<", SequenceNumber: " << (int)SequenceNumber << ", Bytesleft: " << (int)BytesInMsg << std::endl;
        #endif
        if(BytesInMsg <= 6)
        {
            return true;
        }
        else
        {
            BytesLeft_[Key] = BytesInMsg-6;		//how many bytes left
            FastPKG_[Key] = NMsg;
            return false;
        }
    }
}
void NMEA2000Bus::CreateN2kMsg(N2kMsg &NMsg, uint8_t Dest)
{
    NMsg.Source = SourceAddress_;
    NMsg.Destination = Dest;
    if(NMsg.PGN == 59904)			//ISO Request
    {
        NMsg.Priority = 6;
        NMsg.DataLen = 3;
        NMsg.Data.resize(NMsg.DataLen);
    }
    else if(NMsg.PGN == 60928)
    {
        NMsg.Priority = 6;
        NMsg.DataLen = 8;
        NMsg.Data.resize(NMsg.DataLen);
        NMsg.Data[0] = UniqueNumber_ & 0xff;
        NMsg.Data[1] = (UniqueNumber_>>8) & 0xff;
        NMsg.Data[2] = (UniqueNumber_>>16) & 0x1f;

        NMsg.Data[2] |= (ManufacturerCode_<<5)&0xff;
        NMsg.Data[3] = ManufacturerCode_ >> 3;

        NMsg.Data[4] = DeviceInstance_;
        NMsg.Data[5] = DeviceFunction_;
        NMsg.Data[6] = (DeviceClass_ << 1) | 0x01;

        NMsg.Data[7] = (ArbitraryAddressCapable_<<7) | (IndustryCode_<<4) | SystemInstance_;
    }
    else
    {
    #ifdef VERBOSE
            std::cout << "Message not implemented\n";
    #endif
    }
}

bool NMEA2000Bus::SendMessage(N2kMsg &NMsg)
{
    if(!IsFastPackage(NMsg))
    {
        CanMsg Msg;
        N2kMsgToId(NMsg, Msg.id);
        Msg.header.ide = 1;
        //Msg.header.rtr = 0;
        Msg.header.length = NMsg.DataLen;
        for(int i = 0; i < NMsg.DataLen; ++i)
        {
            Msg.data[i] = NMsg.Data[i];
        }
        return Canbus.SendMessage(&Msg);
    }
    else
    {
    #ifdef VERBOSE
            std::cout << "Fast package sending not implemented\n";
    #endif
        return false;
    }
}
uint8_t NMEA2000Bus::CheckForMessages()			//returns address of message if there is one, otherwise returns 0
{
    return Canbus.CheckForMessages();
}
