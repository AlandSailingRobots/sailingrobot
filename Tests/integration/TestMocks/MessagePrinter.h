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

 };