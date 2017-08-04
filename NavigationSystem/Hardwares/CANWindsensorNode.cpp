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


CANWindsensorNode::CANWindsensorNode(MessageBus& msgBus, CANService& can_service, int time_filter_ms)
: CANPGNReceiver(can_service, {130306, 130311}), ActiveNode(NodeID::WindSensor, msgBus), m_TimeBetweenMsgs(time_filter_ms)
{
  m_WindDir  = DATA_OUT_OF_RANGE;
  m_WindSpeed = DATA_OUT_OF_RANGE;
  m_WindTemperature = DATA_OUT_OF_RANGE;

  msgBus.registerNode(*this, MessageType::DataRequest);
}

CANWindsensorNode::~CANWindsensorNode(){}

void CANWindsensorNode::processPGN(N2kMsg &NMsg)
{

  if(NMsg.PGN == 130306){
    std::lock_guard<std::mutex> lock(m_lock);
    uint8_t SID, Ref;
    float WS, WA;
    parsePGN130306(NMsg, SID, WS, WA, Ref);
    m_WindDir = WA;
    m_WindSpeed = WS;
  }
  else if(NMsg.PGN == 130311)
  {
    std::lock_guard<std::mutex> lock(m_lock);
    uint8_t SID, TI, HI;
    float Temp, Hum, AP;
    parsePGN130311(NMsg, SID, TI, HI, Temp, Hum, AP);
    m_WindTemperature = Temp;
  }
  else if(NMsg.PGN == 130312)
  {
    std::lock_guard<std::mutex> lock(m_lock);
    uint8_t SID, TI, TS;
    float ATemp, STemp;
    parsePGN130312(NMsg, SID, TI, TS, ATemp, STemp);
  }
  else if (NMsg.PGN == 130314)
  {
    std::lock_guard<std::mutex> lock(m_lock);
    uint8_t SID, PI, PS;
    double P;
    parsePGN130314(NMsg, SID, PI, PS, P);
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
  runThread(CANWindSensorNodeThreadFunc);
}

void CANWindsensorNode::CANWindSensorNodeThreadFunc(ActiveNode* nodePtr) {

  CANWindsensorNode* node = dynamic_cast<CANWindsensorNode*> (nodePtr);
  Timer timer;
  timer.start();

  while(true) {
    // Need to convert milliseconds into seconds for the argument
    timer.sleepUntil(node->m_TimeBetweenMsgs*1.0f / 1000);
    node->m_lock.lock();

    if( node->m_WindDir == node->DATA_OUT_OF_RANGE &&  node->m_WindTemperature == node->DATA_OUT_OF_RANGE && node->m_WindSpeed == node->DATA_OUT_OF_RANGE){
      node->m_lock.unlock();
      continue;
    }

    MessagePtr windData = std::make_unique<WindDataMsg>(node->m_WindDir, node->m_WindSpeed, node->m_WindTemperature);
    node->m_MsgBus.sendMessage(std::move(windData));

    node->m_lock.unlock();

    timer.reset();
  }
}
