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

#include <ncurses.h>
#include <unordered_map>
#include <thread>
#include <chrono>


/*
    To add new sensors, the sensors should probably send out their data
    on the message bus. Then do the following:
        * Register for the new message
        * Process it and store its data
        * Add the new member fields to the print function.
*/

class SensorDataReceiver : public ActiveNode {
public:
    SensorDataReceiver(MessageBus& msgBus, float timeBetweenPrints) : 
    ActiveNode(NodeID::None, msgBus), m_TimeBetweenPrints(timeBetweenPrints)
    {
        msgBus.registerNode(*this, MessageType::WindData);
        msgBus.registerNode(*this, MessageType::ASPireActuatorFeedback);
    }

    bool init() { return true; }

    void processMessage(const Message* message) {
        MessageType type = message->messageType();
        if(type == MessageType::ASPireActuatorFeedback) {

            const ASPireActuatorFeedbackMsg* actmsg = dynamic_cast<const ASPireActuatorFeedbackMsg*>(message);
            m_SensorValues["Rudder Angle"] = actmsg->rudderFeedback();
            m_SensorValues["Wingsail Angle"] = actmsg->wingsailFeedback();

        } else if (type == MessageType::WindData) {

            const WindDataMsg* windmsg = dynamic_cast<const WindDataMsg*>(message);
            m_SensorValues["Wind Speed"] = windmsg->windSpeed();
            m_SensorValues["Wind Direction"] = windmsg->windDirection();
            m_SensorValues["Wind Temperature"] = windmsg->windTemp();

        }
    }

    void start() {

        m_SensorValues["Rudder Angle"] = -2000;
        m_SensorValues["Wingsail Angle"] = -2000;
        m_SensorValues["Wind Speed"] = -2000;        
        m_SensorValues["Wind Direction"] = -2000;
        m_SensorValues["Wind Temperature"] = -2000;


        m_Win = newwin(6+2*m_SensorValues.size(),56,2,2);
        box(m_Win,0,0);
        wrefresh(m_Win);

        runThread(threadFunc);
        
    }

    void printSensorData() {
     
        werase(m_Win);
        wmove(m_Win, 2,20);
        wprintw(m_Win, "SENSOR READINGS");

    /*    wprintw(m_Win, "%s \t %f", "Rudder Angle : ", m_RudderAngle);
        wprintw(m_Win, "%s \t %f", "Wingsail Angle : ", m_WingsailAngle);
        wprintw(m_Win, "%s \t %f", "Wind Speed : ", m_WindSpeed);
        wprintw(m_Win, "%s \t %f", "Wind Direction : ", m_WindDirection);
        wprintw(m_Win, "%s \t %f", "Wind Temperature : ", m_WindTemp); */

        int pos = 4;
        for(auto it : m_SensorValues) {
            wmove(m_Win, pos, 10);
            wprintw(m_Win, "%s : ", it.first.c_str());
            wmove(m_Win, pos, 35);
            wprintw(m_Win, "%f", it.second);
            pos+=2;
        }

        wrefresh(m_Win);
    }

    static void threadFunc(ActiveNode* nodePtr) {
        SensorDataReceiver* node = dynamic_cast<SensorDataReceiver*> (nodePtr);
        Timer timer;
        timer.start();
        while(true) {
            timer.sleepUntil(node->m_TimeBetweenPrints*1.0f / 1000);
            node->printSensorData();
        }

    }

private:
    
    float m_TimeBetweenPrints;
    float m_RudderAngle = -2000;
    float m_WingsailAngle = -2000;
    float m_WindSpeed = -2000;
    float m_WindDirection = -2000;
    float m_WindTemp = -2000;

    std::unordered_map<std::string, float> m_SensorValues;

    WINDOW* m_Win;

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
    try {
        do {
            std::cout << "Enter rudder angle (-30 to 30)" << std::endl;
            std::getline(std::cin, rudderAngle);
            rudderAngle16 = stoi(rudderAngle);
            rudderAngle16 += 30;
    } while(rudderAngle16 > 60);

        do {
            std::cout << "Enter wingsil angle (-30 to 30)" << std::endl;
            std::getline(std::cin, wingsailAngle);
            wingsailAngle16 = stoi(rudderAngle);
            wingsailAngle16 += 30;
        } while(wingsailAngle16 > 60);
    } catch(std::invalid_argument ex) {
        std::cout << std::endl << "Actuator commands only works with integers." << std::endl << std::endl;
//        std::cout << ex.what() << std::endl;
    }
    

    
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
  //  auto future = canService.start();

    initscr();

    SensorDataReceiver sensorReceiver(msgBus, 250);
    CANWindsensorNode windSensor(msgBus, canService, 500);
    windSensor.start();
    sensorReceiver.start();

    std::thread thr(messageLoop);
    thr.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    getch();
    endwin();

/*
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

    canService.stop();
    future.get();
    thr.detach(); */
    
}