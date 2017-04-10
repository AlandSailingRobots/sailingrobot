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

 class MessagePrinter : public Node
 {
 public:
   MessagePrinter(MessageBus& msgBus)
   : Node(NodeID::MessagePrinter, msgBus)
   {  };

   bool init() {
     return true;
   }

   void processMessage(const Message* message){
      MessageType type = message->messageType();

      std::cout << "Message received of type " << msgToString(type) << std::endl;

      if(type == MessageType::WindData){
        printWindMsg((WindDataMsg*)message);
      }

   }

   void printWindMsg(WindDataMsg* msg){
      std::cout << "Wind Direction : " << msg->windDirection() << " ||| ";
      std::cout << "Wind speed : " << msg->windSpeed() << " ||| ";
      std::cout << "Wind temperature : " << msg->windTemp() << " ||.";
      std::cout << std::endl;
   }


 };