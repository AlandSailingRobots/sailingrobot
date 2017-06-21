// Listens to State and Wind Data messages, stores the information
// and calculates true and apparent wind speed / direction,
// and sends out new Wind State Messages containing this data
// onto the message bus.


#pragma once

#include "MessageBus/MessageBus.h"
#include "MessageBus/MessageTypes.h"
#include "Messages/StateMessage.h"
#include "Messages/WindDataMsg.h"
#include "Messages/WindStateMsg.h"

#include <mutex>


class WindStateNode : public Node {
public:
  WindStateNode(MessageBus& msgBus, int maxTwdBufferSize);
  ~WindStateNode();
  bool init();
  void processMessage(const Message* message);

private:

  void parseWindMessage(const WindDataMsg* msg);
  void parseStateMessage(const StateMessage* msg);
  void sendMessage();

  void updateApparentWind();
  void updateTrueWind();

  float 	m_WindDir;
	float 	m_WindSpeed;
	float	  m_WindTemp;

  float 	m_vesselHeading;
  double	m_vesselLat;
  double	m_vesselLon;
  double	m_vesselSpeed;

  // True Wind Speed / Direction are calculated and
  // updated whenever a Wind Data Message is received.

  double m_trueWindDirection;
  double m_trueWindSpeed;

  // Apparent Wind Speed / Direction are calculated and
  // updated whenever a State Message is received, and will
  // send out a new Wind State Message at the same time.

  double m_apparentWindDirection;
  double m_apparentWindSpeed;

  bool m_windDataReceived = false;
  bool m_stateMsgReceived = false;

  std::vector<float> m_Twd;
  int m_MaxTwdBufferSize;

  std::mutex m_Lock;

};