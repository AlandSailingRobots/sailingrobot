#ifndef msgparsing__h
#define msgparsing__h

#include <stdint.h>

struct CanMsg
{
	uint32_t id;
	struct
	{
		//uint8_t rtr;		//Always zero in J1939
		uint8_t ide;
		uint8_t length;
	} header;
	uint8_t data[8];
};
struct N2kMsg
{
	uint32_t PGN;
	uint8_t Priority;
	uint8_t Source;
	uint8_t Destination;
	int DataLen;
	uint8_t Data[223];
};

//General
void IdToN2kMsg(N2kMsg &NMsg, uint32_t &id);
void N2kMsgToId(N2kMsg &NMsg, uint32_t &id);

void PrintMsg(CanMsg &Msg);
void PrintNMEAMsg(N2kMsg &NMsg);

void ParsePGN59392(N2kMsg &Msg, uint8_t &Controll, uint8_t &GroupFunction, uint32_t &PGN);		//ISO Acknowledgement

void ParsePGN59904(N2kMsg &Msg, uint32_t &PGN);													//ISO Request

void ParsePGN60928(N2kMsg &Msg, uint32_t &UniqueNumber,											//ISO Address Claim
				   uint16_t &ManufacturerCode,
				   uint8_t &DeviceInstance,
				   uint8_t &DeviceFunction,
				   uint8_t &DeviceClass,
				   uint8_t &SystemInstance,
				   uint8_t &IndustryCode,
				   bool &ArbitraryAddressCapable);

void ParsePGN126464(N2kMsg &Msg, uint8_t &FunctionCode, uint32_t &PGN);	//PGN List (Transmit and Receive)

void ParsePGN126996(N2kMsg &Msg, uint16_t &NMEA2000Version,			//Product Information
					uint16_t &ProductCode,
					uint8_t (&ModelID)[32],
					uint8_t (&SoftwareVersionCode)[32],
					uint8_t (&ModelVersion)[32],
					uint8_t (&ModelSerialCode)[32],
					uint8_t &CertificationLevel,
					uint8_t &LoadEquivalency);

//Windsensor
void ParsePGN130306(N2kMsg &Msg, uint8_t &SID, float &WindSpeed,				//WindData
					float &WindAngle, uint8_t &Reference);

void ParsePGN130311(N2kMsg &Msg, uint8_t &SID, uint8_t &TemperatureInstance,	//Environmental Parameters
					uint8_t &HumidityInstance, float &Temperature,
					float &Humidity, float &AtmosphericPressure);

void ParsePGN130312(N2kMsg &Msg, uint8_t &SID, uint8_t &TemperatureInstance,	//Temperature
					uint8_t &TemperatureSource, float &ActualTemperature,
					float &SetTemperature);

void ParsePGN130314(N2kMsg &Msg, uint8_t &SID, uint8_t &PressureInstance,		//ActualPressure
					uint8_t &PressureSource, double &Pressure);

#endif