/****************************************************************************************
 *
 * File:
 * 		MessagePrinter.h
 *
 * Purpose:
 *		Prints out any messages it gets in the terminal.
 *
 * Developer Notes:
 *    For testing purposes, to check that the correct messages
 *    are sent through the message bus.
 *
 ***************************************************************************************/

#include "Messages/Message.h"
#include "Messages/MessageTypes.h"
#include "Messages/WindDataMsg.h"
#include "Messages/WindStateMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/NavigationControlMsg.h"
#include "Nodes/NodeIDs.h"
#include "MessageBus/MessageBus.h"

#include <chrono>

 class MessagePrinter : public Node
 {
 public:
   MessagePrinter(MessageBus& msgBus)
   : Node(NodeID::MessagePrinter, msgBus)
   {
     msgBus.registerNode(*this, MessageType::WindData);
     msgBus.registerNode(*this, MessageType::WindState);
     msgBus.registerNode(*this, MessageType::StateMessage);
     msgBus.registerNode(*this, MessageType::NavigationControl);
   }

   bool init() {
     return true;
   }

   void processMessage(const Message* message){
      MessageType type = message->messageType();

      if(type == MessageType::WindData){
        printWindMsg((WindDataMsg*) message);
      }
      else if(type == MessageType::WindState){
        printWindState((WindStateMsg*) message);
      }
      else if(type == MessageType::StateMessage) {
        printState((StateMessage*) message);
      }
      else if(type == MessageType::NavigationControl) {
        printNavigationControl((NavigationControlMsg*) message);
      }

   }

   void printWindMsg(WindDataMsg* msg){
      std::cout << "////////////////////////////////////////" << std::endl;
      std::cout << "/// Wind Data Message: " << std::endl;
      std::cout << "/// Wind Direction : " << msg->windDirection() << " ||| ";
      std::cout << "Wind speed : " << msg->windSpeed() << " ||| ";
      std::cout << "Wind temperature : " << msg->windTemp() << " |||";
      std::cout << std::endl <<"////////////////////////////////////////" << std::endl;
   }


   void printWindState(WindStateMsg* msg) {
    std::cout << "////////////////////////////////////////" << std::endl;
    std::cout << "/// Wind State Message: " << std::endl;
    std::cout << "/// True Wind Speed : " << msg->trueWindSpeed() << " ||| ";
    std::cout << " True Wind Direction : " << msg->trueWindDirection() << " ||| " << std::endl;
    std::cout << "/// Apparent Wind Speed : " << msg->apparentWindSpeed() << " ||| ";
    std::cout << " Apparent Wind Direction : " << msg->apparentWindDirection() << " |||";
    std::cout << std::endl <<"////////////////////////////////////////" << std::endl;
   }

   void printState(StateMessage* msg) {

      std::cout << "/////////////////////////////////" << std::endl;
      std::cout << "/// State Message: " << std::endl;
      std::cout << "/// Heading : " << msg->heading() << " Speed : " << msg->speed() << std::endl;
      std::cout << "/// Latitude : " << msg->latitude() << " Longitude : " << msg->longitude() << std::endl;
      std::cout << "/// Course : " << msg->course() << std::endl;
      std::cout <<"////////////////////////////////////////" << std::endl;

   }

   void printNavigationControl(NavigationControlMsg* msg) {
      std::cout << "/////////////////////////////////" << std::endl;
      std::cout << "/// Navigation Control Message: " << std::endl;
      std::cout << "/// Course to Steer : " << msg->courseToSteer() << " Target Speed : " << msg->targetSpeed() << std::endl;
      std::cout << "/// Windvane Self Steering On : " << msg->windvaneSelfSteeringOn() << std::endl;
      std::cout << "/// Navigation State : " << std::endl;
      switch(msg->navigationState()) {
        case NavigationState::speedTarget:
          std::cout << "speedTarget";
          break;
        case NavigationState::sailToWaypoint:
          std::cout << "sailToWayPoint";
          break;
        case NavigationState::stationKeeping:
          std::cout << "stationKeeping";
          break;
      }

      std::cout << std::endl;
      std::cout <<"////////////////////////////////////////" << std::endl;
   }

 };
