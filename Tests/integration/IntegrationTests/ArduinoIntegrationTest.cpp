#include "HardwareServices/CAN_Services/CANService.h"
#include "HardwareServices/CAN_Services/N2kMsg.h"
#include "SystemServices/Timer.h"
#include "Messages/ASPireActuatorFeedbackMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "Nodes/ActiveNode.h"
#include "Nodes/NodeIDs.h"
#include "Nodes/CANWindsensorNode.h"

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
            std::cout << std::endl << "_ SOME OR ALL DATA MISSING _" << std::endl;
        }

        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "| " << "Rudder Angle : " << m_RudderAngle << "\t" << "Wingsail Angle : " << m_WingsailAngle << std::endl; 
        std::cout << "| " << "Wind Speed   : " << m_WindSpeed << "\t" << "Wind Direction : " << m_WindDirection << std::endl;
        std::cout << "| " << "Wind Temp    : " << m_WindTemp << std::endl;
        std::cout << "------------------------------------------------" << std::endl << std::endl;

    }

private:
    
    float m_TimeBetweenPrints;
    float m_RudderAngle = -2000;
    float m_WingsailAngle = -2000;
    float m_WindSpeed = -2000;
    float m_WindDirection = -2000;
    float m_WindTemp = -2000;
};


CANService canService;
MessageBus msgBus;

void messageLoop() {
    msgBus.run();
}

void sendActuatorCommands() {
    CanMsg Cmsg;
    Cmsg.id = 700;
    Cmsg.header.ide = 0;
    Cmsg.header.length = 8;

    std::string rudderAngle;
    std::string wingsailAngle;
    uint16_t rudderAngle16;
    uint16_t wingsailAngle16;
    float ratio = 65535 / 60;

    do {
        std::cout << "Enter rudder angle (-30 to 30)" << std::endl;
        std::getline(std::cin, rudderAngle);
        rudderAngle16 = stoi(rudderAngle);
    } while(rudderAngle16 < -30 || rudderAngle16 > 30);

    do {
        std::cout << "Enter wingsil angle (-30 to 30)" << std::endl;
        std::getline(std::cin, wingsailAngle);
        wingsailAngle16 = stoi(wingsailAngle);
    } while(wingsailAngle16 < -30 || wingsailAngle16 > 30);

    rudderAngle16 += 30;
    wingsailAngle16 += 30;
    rudderAngle16 *= ratio;
    wingsailAngle16 *= ratio;

    Cmsg.data[0] = rudderAngle16 & 0xff;
    Cmsg.data[1] = rudderAngle16 >> 8;
    Cmsg.data[2] = wingsailAngle16 & 0xff;
    Cmsg.data[3] = wingsailAngle16 >> 8;

    // More data to be added, windvane self steering

    canService.sendCANMessage(Cmsg);

}

int main() {

    // Comment out this line if not running on the pi
    // otherwise program will crash.

    auto future = canService.start();

    SensorDataReceiver sensorReceiver(msgBus, 2000);
    CANWindsensorNode windSensor(msgBus, canService, 500);
    windSensor.start();

    std::thread thr(messageLoop);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::string command;
    std::string lastcommand;

    do {
        std::cout << "Enter command: " << std::endl;
        std::cout << "'s' for sending actuator commands." << std::endl;
        std::cout << "'p' for printing lastest sensor readings." << std::endl;
        std::cout << "'q' to quit program." << std::endl;
        std::cout << "#> ";
        std::getline(std::cin, command);

        if(!command.empty()) {
            lastcommand = command;
        }

        if(command == "p") {
            sensorReceiver.printSensorData();

        } else if(command == "s") {
             sendActuatorCommands();
        } else if(command.empty()) {

        // This is John's idea, so it's easier to print out
        // sensor data multiple times.
          if(lastcommand == "p") {
                sensorReceiver.printSensorData();
            }
        }
    } while(command != "q");


    thr.detach();
}