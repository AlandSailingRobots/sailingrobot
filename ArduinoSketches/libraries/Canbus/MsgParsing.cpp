#include "MsgParsing.h"
#include <SPI.h>


void IdToN2kMsg(N2kMsgArd &NMsg, uint32_t &id)
{
    uint8_t Prio = (id>> 26)& 0x07;
    uint8_t DP = (id >> 24) & 1;
    uint8_t PF = (id >> 16);
    uint8_t PS = (id >> 8);
    uint8_t SA = (id);


    NMsg.Priority = Prio;
    NMsg.Source = SA;

    if(PF < 240)		//PDU1, adressable
    {
        NMsg.Destination = PS;
        NMsg.PGN = ((uint32_t)DP<<16) | ((uint32_t)PF<<8); // last byte is 00 for adressable PGNs
    }
    else				//PDU2, broadcast
    {
        NMsg.Destination = 0xff;
        NMsg.PGN = ((uint32_t)DP<<16) | ((uint32_t)PF<<8) | (PS);
    }
}

void N2kMsgToId(N2kMsgArd &NMsg, uint32_t &id)
{
    uint32_t Prio = NMsg.Priority;
    uint32_t DP = (NMsg.PGN >> 16) & 0x01;
    uint32_t PF = (NMsg.PGN >> 8) &0xff;
    uint16_t PS;
    uint8_t SA = NMsg.Source;

    if(PF < 240)
    {
        PS = NMsg.Destination;
    }
    else
    {
        PS = NMsg.PGN & 0xff;
    }

    id = (Prio << 26) | (DP << 24) | (PF << 16) | (PS << 8) | (SA);
}
void PrintMsg(CanMsg &Msg)
{
    if(Msg.header.ide)
    {
        N2kMsgArd NMsg;
        IdToN2kMsg(NMsg, Msg.id);
        Serial.print(Msg.id,HEX);
        Serial.print(" ");
        Serial.print(NMsg.PGN);
        Serial.print(" Priority: ");Serial.print((int)NMsg.Priority);
        Serial.print(" Source: ");Serial.print((int)NMsg.Source);
        Serial.print(" Destination: ");Serial.print((int)NMsg.Destination);
        Serial.print(" DataLen: ");Serial.print((int)Msg.header.length); Serial.print(" ");

        for(int i = 0; i < Msg.header.length; ++i)
        {
            Serial.print((int)Msg.data[i], HEX); Serial.print(" ");
        }
        Serial.println("");
    }
    else
    {
        Serial.println("Message not extended format");
    }
}
void PrintNMEAMsg(N2kMsgArd &NMsg)
{
    Serial.print(NMsg.PGN);
    Serial.print(" Priority: ");	Serial.print((int)NMsg.Priority);
    Serial.print(" Source: ");		Serial.print((int)NMsg.Source);
    Serial.print(" Destination: ");	Serial.print((int)NMsg.Destination);
    Serial.print(" DataLen: ");		Serial.print((int)NMsg.DataLen); Serial.print(" ");
    if(NMsg.PGN == 59392)
    {
        uint8_t Con, GF;
        uint32_t PGN;
        ParsePGN59392(NMsg, Con, GF, PGN);

        Serial.print("Controll: ");		Serial.println((int)Con);
        Serial.print(" GroupFunction: ");Serial.println((int)GF);
        Serial.print(" PGN: ");			Serial.println(PGN);
    }
    else if(NMsg.PGN == 59904)
    {
        uint32_t PGN;
        ParsePGN59904(NMsg, PGN);
        Serial.print("PGN: ");			Serial.println(PGN);
    }
    else if(NMsg.PGN == 60928)
    {
        uint32_t UN;
        uint16_t MC;
        uint8_t DI, DF, DC, SI, IC;
        bool AAC;
        ParsePGN60928(NMsg, UN, MC, DI, DF, DC, SI, IC, AAC);

        Serial.print("UniqueNumber: ");				Serial.print(UN);
        Serial.print(" ManufacturerCode: ");		Serial.print(MC);
        Serial.print(" DeviceInstance: ");			Serial.print((int)DI);
        Serial.print(" DeviceFunction:  ");			Serial.print((int)DF);
        Serial.print(" DeviceClass:  ");			Serial.print((int)DC);
        Serial.print(" SystemInstance:  ");			Serial.print((int)SI);
        Serial.print(" IndustryCode:  ");			Serial.print((int)IC);
        Serial.print(" ArbitraryAddressCapable:  ");Serial.println(AAC);

    }
    else if(NMsg.PGN == 130306)
    {
        uint8_t SID, Ref;
        float WS, WA;
        ParsePGN130306(NMsg, SID, WS, WA, Ref);

        Serial.print("SID:  ");			Serial.print((int)SID);
        Serial.print(" Windspeed:  ");	Serial.print(WS);
        Serial.print(" Windangle:  ");	Serial.print(WA);
        Serial.print(" Reference:  ");	Serial.println((int)Ref);

    }
    else if(NMsg.PGN == 130311)
    {
        uint8_t SID, TI, HI;
        float Temp, Hum, AP;
        ParsePGN130311(NMsg, SID, TI, HI, Temp, Hum, AP);

        Serial.print("SID:  ");			Serial.print((int)SID);
        Serial.print(" TemperatureInstance:  ");	Serial.print(TI);
        Serial.print(" HumidityInstance:  ");	Serial.print(Temp);
        Serial.print(" Humidity:  ");	Serial.print(Hum);
        Serial.print(" AtmosphericPressure:  ");	Serial.println(AP);
    }
    else if(NMsg.PGN == 130312)
    {
        uint8_t SID, TI, TS;
        float ATemp, STemp;
        ParsePGN130312(NMsg, SID, TI, TS, ATemp, STemp);
        Serial.print("SID:  ");			Serial.print((int)SID);
        Serial.print(" TemperatureInstance:  ");	Serial.print(TI);
        Serial.print(" TemperatureSource:  ");	Serial.print(TS);
        Serial.print(" Actual Temperature:  ");	Serial.print(ATemp);
        Serial.print(" Set Temperature:  ");	Serial.println(STemp);


    }
    else if(NMsg.PGN == 130314)
    {
        uint8_t SID, PRI, PS;
        double P;
        ParsePGN130314(NMsg, SID, PRI, PS, P);

        Serial.print("SID:  ");			Serial.print((int)SID);
        Serial.print(" Pressure Instance:  ");	Serial.print(PRI);
        Serial.print(" Pressure Source:  ");	Serial.print(PS);
        Serial.print(" Pressure:  ");	Serial.println(P);
    }
    else if(NMsg.PGN == 126996)
    {
        uint16_t N2kV, PC;
        uint8_t MID[32], SVC[32], MV[32], MSC[32], CL, LE;

        ParsePGN126996(NMsg, N2kV, PC, MID, SVC, MV, MSC, CL, LE);

        Serial.print("NMEA2000 Version:  ");			Serial.print((int)N2kV);
        Serial.print(" ProductCode:  ");	Serial.print(PC);

        Serial.print(" ModelID:  ");
        for(int i = 0; i < 32; ++i)
            Serial.print(MID[i]);

        Serial.print(" SoftwareVersionCode:  ");
        for(int i = 0; i < 32; ++i)
            Serial.print(SVC[i]);


        Serial.print(" ModelVersion:  ");
        for(int i = 0; i < 32; ++i)
            Serial.print(MV[i]);

        Serial.print(" ModelSerialCode:  ");
        for(int i = 0; i < 32; ++i)
            Serial.print(MSC[i]);

        Serial.print(" CertificationLevel:  ");	Serial.print(CL);
        Serial.print(" LoadEquivalency:  ");	Serial.println(LE);
    }
    else
    {
        Serial.println("###Not implemented###");
    }
}
void ParsePGN59392(N2kMsgArd &NMsg, uint8_t &Controll, uint8_t &GroupFunction, uint32_t &PGN)		//ISO Acknowledgement
{
    Controll = NMsg.Data[0];
    GroupFunction = NMsg.Data[1];
    PGN = NMsg.Data[5] | ((uint16_t)NMsg.Data[6]<<8) | ((uint32_t)NMsg.Data[7]<<16);
}
void ParsePGN59904(N2kMsgArd &NMsg, uint32_t &PGN)													//ISO Request
{
    PGN = NMsg.Data[0] | ((uint16_t)NMsg.Data[1]<<8) | ((uint32_t)NMsg.Data[2]<<16);

}
void ParsePGN60928(N2kMsgArd &NMsg, uint32_t &UniqueNumber,				//ISO Address Claim
                   uint16_t &ManufacturerCode,
                   uint8_t &DeviceInstance,
                   uint8_t &DeviceFunction,
                   uint8_t &DeviceClass,
                   uint8_t &SystemInstance,
                   uint8_t &IndustryCode,
                   bool &ArbitraryAddressCapable)
{
    UniqueNumber = NMsg.Data[0] | ((uint16_t)NMsg.Data[1]<<8) | (((uint32_t)NMsg.Data[2]&0x1f)<<16);
    ManufacturerCode = (NMsg.Data[2]&0xe0) | ((uint16_t)NMsg.Data[3]<<3);
    DeviceInstance = NMsg.Data[4];
    DeviceFunction = NMsg.Data[5];
    DeviceClass = NMsg.Data[6]>>1;
    ArbitraryAddressCapable = NMsg.Data[7] & (1<<7);
    IndustryCode = (NMsg.Data[7]>>4)&0x07;
    SystemInstance = NMsg.Data[7] & 0x0f;
}
void ParsePGN126996(N2kMsgArd &NMsg, uint16_t &NMEA2000Version,			//Product Information
                    uint16_t &ProductCode,
                    uint8_t (&ModelID)[32],
                    uint8_t (&SoftwareVersionCode)[32],
                    uint8_t (&ModelVersion)[32],
                    uint8_t (&ModelSerialCode)[32],
                    uint8_t &CertificationLevel,
                    uint8_t &LoadEquivalency)
{
    NMEA2000Version = NMsg.Data[0] | ((uint16_t)NMsg.Data[1]<<8);
    ProductCode = NMsg.Data[2] | ((uint16_t)NMsg.Data[3]<<8);
    for(int i = 0; i < 32; ++i)
    {
        ModelID[i] = NMsg.Data[4 +i];
        SoftwareVersionCode[i] = NMsg.Data[36 +i];
        ModelVersion[i] = NMsg.Data[68 +i];
        ModelSerialCode[i] = NMsg.Data[100 +i];
    }
    CertificationLevel = NMsg.Data[132];
    LoadEquivalency = NMsg.Data[133];
}

void ParsePGN130306(N2kMsgArd &NMsg, uint8_t &SID, float &WindSpeed,				//WindData
                    float &WindAngle, uint8_t &Reference)
{
    SID = NMsg.Data[0];
    uint16_t tmp = NMsg.Data[1] | ((uint16_t)NMsg.Data[2]<<8);
    WindSpeed = tmp*0.01;
    tmp = NMsg.Data[3] | ((uint16_t)NMsg.Data[4]<<8);
    WindAngle = tmp*0.0001;
    Reference = NMsg.Data[5] & 0x07;
}

void ParsePGN130311(N2kMsgArd &NMsg, uint8_t &SID, uint8_t &TemperatureInstance,	//Environmental Parameters
                    uint8_t &HumidityInstance, float &Temperature,
                    float &Humidity, float &AtmosphericPressure)
{
    SID = NMsg.Data[0];
    TemperatureInstance = NMsg.Data[1] & 0x3f;
    HumidityInstance = NMsg.Data[1] >> 6;
    uint16_t tmp = NMsg.Data[2] | ((uint16_t)NMsg.Data[3]<<8);
    Temperature = tmp*0.01;
    Humidity = 0;
    tmp = NMsg.Data[6] | ((uint16_t)NMsg.Data[7]<<8);
    AtmosphericPressure = tmp;		//hPa
}
void ParsePGN130312(N2kMsgArd &NMsg, uint8_t &SID, uint8_t &TemperatureInstance,	//Temperature
                    uint8_t &TemperatureSource, float &ActualTemperature,
                    float &SetTemperature)
{
    SID = NMsg.Data[0];
    TemperatureInstance = NMsg.Data[1];
    TemperatureSource = NMsg.Data[2];
    uint16_t tmp = NMsg.Data[3] | ((uint16_t)NMsg.Data[4]<<8);
    ActualTemperature = tmp*0.01;
    tmp = NMsg.Data[5] | ((uint16_t)NMsg.Data[6]<<8);
    SetTemperature = tmp*0.01;
}

void ParsePGN130314(N2kMsgArd &NMsg, uint8_t &SID, uint8_t &PressureInstance,		//ActualPressure
                    uint8_t &PressureSource, double &Pressure)
{
    SID = NMsg.Data[0];
    PressureInstance = NMsg.Data[1];
    PressureSource = NMsg.Data[2];

    uint32_t tmp = NMsg.Data[3] | ((uint16_t)NMsg.Data[4]<<8) | ((uint32_t)NMsg.Data[5]<<16) | ((uint32_t)NMsg.Data[6]<<24);
    Pressure = tmp / 1000.0f; 			//hPa
}
