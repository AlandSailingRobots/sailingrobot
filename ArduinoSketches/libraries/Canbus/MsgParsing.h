#ifndef msgparsing__h
#define msgparsing__h

#include <canbus_defs.h>
#include <stdint.h>

// General
void IdToN2kMsg(N2kMsgArd& NMsg, uint32_t& id);
void N2kMsgToId(N2kMsgArd& NMsg, uint32_t& id);

void PrintMsg(CanMsg& Msg);
void PrintNMEAMsg(N2kMsgArd& NMsg);

void ParsePGN59392(N2kMsgArd& Msg,
                   uint8_t& Controll,
                   uint8_t& GroupFunction,
                   uint32_t& PGN);  // ISO Acknowledgement

void ParsePGN59904(N2kMsgArd& Msg, uint32_t& PGN);  // ISO Request

void ParsePGN60928(N2kMsgArd& Msg,
                   uint32_t& UniqueNumber,  // ISO Address Claim
                   uint16_t& ManufacturerCode,
                   uint8_t& DeviceInstance,
                   uint8_t& DeviceFunction,
                   uint8_t& DeviceClass,
                   uint8_t& SystemInstance,
                   uint8_t& IndustryCode,
                   bool& ArbitraryAddressCapable);

void ParsePGN126464(N2kMsgArd& Msg,
                    uint8_t& FunctionCode,
                    uint32_t& PGN);  // PGN List (Transmit and Receive)

void ParsePGN126996(N2kMsgArd& Msg,
                    uint16_t& NMEA2000Version,  // Product Information
                    uint16_t& ProductCode,
                    uint8_t (&ModelID)[32],
                    uint8_t (&SoftwareVersionCode)[32],
                    uint8_t (&ModelVersion)[32],
                    uint8_t (&ModelSerialCode)[32],
                    uint8_t& CertificationLevel,
                    uint8_t& LoadEquivalency);

// Windsensor
void ParsePGN130306(N2kMsgArd& Msg,
                    uint8_t& SID,
                    float& WindSpeed,  // WindData
                    float& WindAngle,
                    uint8_t& Reference);

void ParsePGN130311(N2kMsgArd& Msg,
                    uint8_t& SID,
                    uint8_t& TemperatureInstance,  // Environmental Parameters
                    uint8_t& HumidityInstance,
                    float& Temperature,
                    float& Humidity,
                    float& AtmosphericPressure);

void ParsePGN130312(N2kMsgArd& Msg,
                    uint8_t& SID,
                    uint8_t& TemperatureInstance,  // Temperature
                    uint8_t& TemperatureSource,
                    float& ActualTemperature,
                    float& SetTemperature);

void ParsePGN130314(N2kMsgArd& Msg,
                    uint8_t& SID,
                    uint8_t& PressureInstance,  // ActualPressure
                    uint8_t& PressureSource,
                    double& Pressure);

#endif