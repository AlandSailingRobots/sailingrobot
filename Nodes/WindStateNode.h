
#include "MessageBus/MessageBus.h"
#include "Messages/MessageTypes.h"
#include "Messages/StateMessage.h"
#include "Messages/WindDataMsg.h"

#pragma once

class WindStateNode : public Node {
public:
  WindStateNode(MessageBus& msgBus);
  bool init();
  void processMessage(const Message* message);

private:

  void parseWindMessage(WindDataMsg* msg);
  void parseStateMessage(StateMessage* msg);
  void sendMessage();

  float 	m_WindDir;
	float 	m_WindSpeed;
	float	  m_WindTemp;

  float 	m_vesselHeading;
  double	m_vesselLat;
  double	m_vesselLon;
  double	m_vesselSpeed;

};