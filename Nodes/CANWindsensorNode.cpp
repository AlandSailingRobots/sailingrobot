/****************************************************************************************
 *
 * File:
 * 		CANWindsensorNode.cpp
 *
 * Purpose:
 *		 Process messages from the CAN-Service.
 *
 * Developer Notes:
 *		PGN numbers for this node are:
 *			130306, 130311, (more to be added later?)
 *
 ***************************************************************************************/

#include "CANWindsensorNode.h"
#include "Messages/WindDataMsg.h"

#include <chrono>
#include <thread>

#define DATA_OUT_OF_RANGE		-2000

CANWindsensorNode::CANWindsensorNode(MessageBus& msgBus, CANService& can_service, int time_filter_ms)
 : CANPGNReceiver(can_service, {130306, 130311}), ActiveNode(NodeID::WindSensor, msgBus), m_TimeBetweenMsgs(time_filter_ms)
 {
 	m_WindDir  = DATA_OUT_OF_RANGE;
    m_WindSpeed = DATA_OUT_OF_RANGE;
    m_WindTemperature = DATA_OUT_OF_RANGE;
 }

void CANWindsensorNode::processPGN(N2kMsg &NMsg)
{

	if(NMsg.PGN == 130306){
		uint8_t SID, Ref;
		float WS, WA;
		parsePGN130306(NMsg, SID, WS, WA, Ref);

		m_lock.lock();
		m_WindDir = WA;
		m_WindSpeed = WS;

//		MessagePtr windData = std::make_unique<WindDataMsg>(m_WindDir, m_WindSpeed, m_WindTemperature);
//		m_MsgBus.sendMessage(std::move(windData));
		m_lock.unlock();
	}
	else if(NMsg.PGN == 130311)
	{
		uint8_t SID, TI, HI;
		float Temp, Hum, AP;
		parsePGN130311(NMsg, SID, TI, HI, Temp, Hum, AP);

		m_lock.lock();
		m_WindTemperature = Temp;

//		MessagePtr windData = std::make_unique<WindDataMsg>(m_WindDir, m_WindSpeed, m_WindTemperature);
//		m_MsgBus.sendMessage(std::move(windData));
		m_lock.unlock();

	}
	else if(NMsg.PGN == 130312)
	{
		m_lock.unlock();
		uint8_t SID, TI, TS;
		float ATemp, STemp;
		parsePGN130312(NMsg, SID, TI, TS, ATemp, STemp);
		m_lock.unlock();
	}
	else if (NMsg.PGN == 130314)
	{
		m_lock.unlock();
		uint8_t SID, PI, PS;
		double P;
		parsePGN130314(NMsg, SID, PI, PS, P);
		m_lock.unlock();

	}
}

void CANWindsensorNode::parsePGN130306(N2kMsg &NMsg, uint8_t &SID, float &WindSpeed,				//WindData
	float &WindAngle, uint8_t &Reference)
{
	SID = NMsg.Data[0];
	uint16_t tmp = NMsg.Data[1] | (NMsg.Data[2]<<8);
	WindSpeed = tmp*0.01;
	tmp = NMsg.Data[3] | (NMsg.Data[4]<<8);
	WindAngle = tmp*0.0001;
	Reference = NMsg.Data[5] & 0x07;
}

void CANWindsensorNode::parsePGN130311(N2kMsg &NMsg, uint8_t &SID, uint8_t &TemperatureInstance,	//Environmental Parameters
	uint8_t &HumidityInstance, float &Temperature,
	float &Humidity, float &AtmosphericPressure)
{
	SID = NMsg.Data[0];
	TemperatureInstance = NMsg.Data[1] & 0x3f;
	HumidityInstance = NMsg.Data[1] >> 6;
	uint16_t tmp = NMsg.Data[2] | (NMsg.Data[3]<<8);
	Temperature = tmp*0.01;
	//tmp = NMsg.Data[4] | (NMsg.Data[5]<<8);
	//Humidity = tmp*0.004;
	Humidity = 0;
	tmp = NMsg.Data[6] | (NMsg.Data[7]<<8);
	AtmosphericPressure = tmp;		//hPa
}

void CANWindsensorNode::parsePGN130312(N2kMsg &NMsg, uint8_t &SID, uint8_t &TemperatureInstance,	//Temperature
					uint8_t &TemperatureSource, float &ActualTemperature,
					float &SetTemperature)
{
	SID = NMsg.Data[0];
	TemperatureInstance = NMsg.Data[1];
	TemperatureSource = NMsg.Data[2];
	uint16_t tmp = NMsg.Data[3] | (NMsg.Data[4]<<8);
	ActualTemperature = tmp*0.01;
	tmp = NMsg.Data[5] | (NMsg.Data[6]<<8);
	SetTemperature = tmp*0.01;
}

void CANWindsensorNode::parsePGN130314(N2kMsg &NMsg, uint8_t &SID, uint8_t &PressureInstance,		//ActualPressure
					uint8_t &PressureSource, double &Pressure)
{
	SID = NMsg.Data[0];
	PressureInstance = NMsg.Data[1];
	PressureSource = NMsg.Data[2];

	uint32_t tmp = NMsg.Data[3] | (NMsg.Data[4]<<8) | (NMsg.Data[5]<<16) | (NMsg.Data[6]<<24);
	Pressure = tmp / 1000.0f; 			//hPa
}

void CANWindsensorNode::processMessage(const Message* message) {

   std::lock_guard<std::mutex> lock(m_lock);

  if(message->messageType() == MessageType::DataRequest)
  {
    // On system startup we won't have any valid data, so don't send any
    if( m_WindDir!= DATA_OUT_OF_RANGE ||  m_WindTemperature != DATA_OUT_OF_RANGE || m_WindSpeed != DATA_OUT_OF_RANGE)
    {

      MessagePtr windData = std::make_unique<WindDataMsg>(message->sourceID(), this->nodeID(), m_WindDir, m_WindSpeed, m_WindTemperature);
      m_MsgBus.sendMessage(std::move(windData));
    }
  }
}

void CANWindsensorNode::start() {
	std::thread* thr = new std::thread(CANWindSensorNodeThreadFunc, this, std::ref(m_MsgBus));
	thr->detach();
}

void CANWindsensorNode::CANWindSensorNodeThreadFunc(void* nodePtr, MessageBus& bus) {

	float lastRecordedDir=0;
	float lastRecordedSpeed=0;
	float lastRecordedTemp=0;
	CANWindsensorNode* node = (CANWindsensorNode*) nodePtr;

	while(true) {
		node->m_lock.lock();
		// If there has been a change in wind data since the last cycle,
		// update stored values and send out a new message, otherwise
		// wait for the next cycle.

		if( node->m_WindDir == DATA_OUT_OF_RANGE &&  node->m_WindTemperature == DATA_OUT_OF_RANGE && node->m_WindSpeed == DATA_OUT_OF_RANGE){
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			node->m_lock.unlock();
			continue;
		}

		if(lastRecordedDir != node->m_WindDir || lastRecordedSpeed != node->m_WindSpeed 
			|| lastRecordedTemp != node->m_WindTemperature){
			
			lastRecordedDir = node->m_WindDir;
			lastRecordedSpeed = node->m_WindSpeed;
			lastRecordedTemp = node->m_WindTemperature;

			MessagePtr windData = std::make_unique<WindDataMsg>(node->m_WindDir, node->m_WindSpeed, node->m_WindTemperature);
			bus.sendMessage(std::move(windData));
		}

		node->m_lock.unlock();
		std::this_thread::sleep_for(std::chrono::milliseconds(node->m_TimeBetweenMsgs));

	}
}