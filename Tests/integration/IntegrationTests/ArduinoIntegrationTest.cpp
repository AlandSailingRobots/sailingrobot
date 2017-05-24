#include "HardwareServices/CAN_Services/CANService.h"
#include "Messages/ASPireActuatorFeedbackMsg.h"
#include "Messages/WindDataMsg.h"
#include "SystemServices/Timer.h"
#include "Nodes/ActiveNode.h"
#include "Messages/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "Nodes/NodeIDs.h"

#include <iostream>
#include <thread>
#include <chrono>

class SensorDataReceiver : public Node {
public:
    SensorDataReceiver(MessageBus& msgBus, float timeBetweenPrints) : 
    Node(NodeID::None, msgBus), m_TimeBetweenPrints(timeBetweenPrints)
    {
        msgBus.registerNode(*this, MessageType::WindData);
        msgBus.registerNode(*this, MessageType::ASPireActuatorFeedback);
    }

    bool init() { return true; }

    void processMessage(const Message* message) {
        MessageType type = message->messageType();
        if(type == MessageType::ASPireActuatorFeedback) {

            const ASPireActuatorFeedbackMsg* actmsg = dynamic_cast<const ASPireActuatorFeedbackMsg*>(message);
            m_RudderAngle = actmsg->rudderFeedback();
            m_WingsailAngle = actmsg->wingsailFeedback();

        } else if (type == MessageType::WindData) {

            const WindDataMsg* windmsg = dynamic_cast<const WindDataMsg*>(message);
            m_WindSpeed = windmsg->windSpeed();
            m_WindDirection = windmsg->windDirection();
            m_WindTemp = windmsg->windTemp();

        }
    }

    void printSensorData() {
        if(m_RudderAngle == -2000 || m_WingsailAngle == -2000 || m_WindSpeed == -2000 ||
                m_WindDirection == -2000 || m_WindTemp == -2000)
        {
            std::cout << "No data available.." << std::endl;
        }

        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << "| " << "Rudder Angle : " << m_RudderAngle << "\t" << "Wingsail Angle : " << m_WingsailAngle << std::endl; 
        std::cout << "| " << "Wind Speed   : " << m_WindSpeed << "\t" << "Wind Direction : " << m_WindDirection << std::endl;
        std::cout << "| " << "Wind Temp    : " << m_WindTemp << std::endl;
        std::cout << "---------------------------------------------------" << std::endl << std::endl;

    }

private:
    
    float m_TimeBetweenPrints;
    float m_RudderAngle = -2000;
    float m_WingsailAngle = -2000;
    float m_WindSpeed = -2000;
    float m_WindDirection = -2000;
    float m_WindTemp = -2000;
};




MessageBus msgBus;

void messageLoop() {
    msgBus.run();
}

int main() {
    // CANService canService;
    SensorDataReceiver sensorReceiver(msgBus, 2000);
    std::thread thr(messageLoop);
   // sensorReceiver.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    MessagePtr windData = std::make_unique<WindDataMsg>(1,2,3);
    MessagePtr feedbackData = std::make_unique<ASPireActuatorFeedbackMsg>(20,30,15,3);

    msgBus.sendMessage(std::move(windData));
    msgBus.sendMessage(std::move(feedbackData));

    std::string command;
    std::string lastcommand;

    do {
        std::cout << "Enter command ('s' for sending a message, 'p' to print current sensor data, 'q' to quit." << std::endl;
        std::cout << "#> ";
        std::getline(std::cin, command);
        if(!command.empty()) {
            lastcommand = command;
        }

        if(command == "p") {
            sensorReceiver.printSensorData();

        } else if(command == "s") {
            // sendActuatorCommands();
        } else if(command.empty()) {
          if(lastcommand == "p") {
                sensorReceiver.printSensorData();
            }
        }

    } while(command != "q");


    thr.detach();
}