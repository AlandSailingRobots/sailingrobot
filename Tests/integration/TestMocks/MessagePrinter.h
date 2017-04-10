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
   }

   bool init() {
     return true;
   }

   void processMessage(const Message* message){
      MessageType type = message->messageType();
      std::cout << "\033[2J\033[1;1H";

      if(type == MessageType::WindData){
        printWindMsg((WindDataMsg*)message);
      }

   }

   void printWindMsg(WindDataMsg* msg){
      std::cout << "////////////////////////////////////////" << std::endl;
      std::cout << "/// Wind Direction : " << msg->windDirection() << " ||| ";
      std::cout << "/// Wind speed : " << msg->windSpeed() << " ||| ";
      std::cout << "/// Wind temperature : " << msg->windTemp() << " ||.";
      std::cout << "////////////////////////////////////////" << std::endl;
   }


 };