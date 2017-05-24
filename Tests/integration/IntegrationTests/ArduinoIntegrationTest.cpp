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

class SensorDataReceiver : public ActiveNode {
public:
    SensorDataReceiver(MessageBus& msgBus, float timeBetweenPrints) : 
    ActiveNode(NodeID::None, msgBus), m_TimeBetweenPrints(timeBetweenPrints)
    {
        msgBus.registerNode(*this, MessageType::WindData);
        msgBus.registerNode(*this, MessageType::ASPireActuatorFeedback);

        m_Running.store(true);
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

    void start() {
        runThread(SensorReceiverThreadFunc);
    }

    void pause() {
        m_Running.store(false);
    }

    void resume() {
        m_Running.store(true);
    }

private:

    static void SensorReceiverThreadFunc(ActiveNode* nodePtr) {
        Timer timer;
        timer.start();
        SensorDataReceiver* node = dynamic_cast<SensorDataReceiver*> (nodePtr);
        while(true) {
            
            if(node->m_Running.load() == false) {
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                continue;
            }

            node->m_Mtx.lock();

            if(node->m_RudderAngle == -2000 || node->m_WingsailAngle == -2000 || node->m_WindSpeed == -2000 ||
               node-> m_WindDirection == -2000 || node->m_WindTemp == -2000) {
                std::cout << "No data available, waiting..." << std::endl;
                node->m_Mtx.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            timer.sleepUntil(node->m_TimeBetweenPrints*1.0f / 1000);
            timer.reset();
            std::cout << "---------------------------------------------------" << std::endl;
            std::cout << "| " << "Rudder Angle : " << node->m_RudderAngle << "\t" << "Wingsail Angle : " << node->m_WingsailAngle << std::endl; 
            std::cout << "| " << "Wind Speed   : " << node->m_WindSpeed << "\t" << "Wind Direction : " << node->m_WindDirection << std::endl;
            std::cout << "| " << "Wind Temp    : " << node->m_WindTemp << std::endl;
            std::cout << "---------------------------------------------------" << std::endl << std::endl;

            node->m_Mtx.unlock();
        }
    }

    std::atomic_bool m_Running;
    std::mutex m_Mtx;
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
    sensorReceiver.start();

    char command;

    do {
        std::cout << "Enter command ('s' for sending a message, 'l' for listening of sensor data, 'q' to quit." << std::endl;
        std::cout << "#> " << std::endl;
        std::cin >> command;
        std::cin.ignore(10000, '\n');

        switch(command) {
            case 's':
                sendActuatorMessage();
                break;
            case 'l':
                sensorReceiver.start();
                break;
            case 'q':
                break;
            default:
                break
        }

    } while(command != 'q');

    MessagePtr windData = std::make_unique<WindDataMsg>(1,2,3);
    MessagePtr feedbackData = std::make_unique<ASPireActuatorFeedbackMsg>(20,30,15,3);

    msgBus.sendMessage(std::move(windData));
    msgBus.sendMessage(std::move(feedbackData));

    thr.join();
}