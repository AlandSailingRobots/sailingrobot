#include "Hardwares/CAN_Services/CANService.h"
#include "Hardwares/CAN_Services/N2kMsg.h"
#include "SystemServices/Timer.h"
#include "Messages/ASPireActuatorFeedbackMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/ArduinoDataMsg.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/MessageBus.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/NodeIDs.h"
#include "Hardwares/CANWindsensorNode.h"
#include "Hardwares/ArduinoNode.h"
#include "Hardwares/CANArduinoNode.h"
#include "Hardwares/CANFeedbackReceiver.h"

#include <ncurses.h>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <cctype>
#include <sstream>

#define BACKSPACE 127
#define ENTER 10
#define TAB 9

#define MAX_INPUT 20

typedef std::unordered_map<std::string, float> SensorData;

/*
    To add new sensors, the sensors should probably send out their data
    on the message bus. Then do the following:
        * Register for the new message
        * Process it and store its data
        * Add the new member fields to the print function.
*/

class SensorDataReceiver : public Node {
public:
    SensorDataReceiver(MessageBus& msgBus, float timeBetweenPrints) : 
    Node(NodeID::None, msgBus), m_TimeBetweenPrints(timeBetweenPrints)
    {
        msgBus.registerNode(*this, MessageType::WindData);
        msgBus.registerNode(*this, MessageType::ASPireActuatorFeedback);
	msgBus.registerNode(*this, MessageType::ArduinoData);

        m_SensorValues["Rudder Angle"] = -2000;
        m_SensorValues["Wingsail Angle"] = -2000;
        m_SensorValues["Wind Speed"] = -2000;        
        m_SensorValues["Wind Direction"] = -2000;
        m_SensorValues["Wind Temperature"] = -2000;
				m_SensorValues["RC Mode"] = -2000;

        m_Win = newwin(6+2*m_SensorValues.size(),60,1,2);
        
        box(m_Win,0,0);
        keypad(m_Win, FALSE);
        wrefresh(m_Win);
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

        } else if (type == MessageType::ArduinoData) {
	    const ArduinoDataMsg* arduinomsg = dynamic_cast<const ArduinoDataMsg*>(message);
	    //m_SensorValues["RC Mode"] = arduinomsg->RC();
	    if (arduinomsg->RC() > 10) {
	        m_SensorValues["RC Mode"] = -3000;
	    }
            else {
                m_SensorValues["RC Mode"] = -4000;
            }
	}
        printSensorData();
    }

    void printSensorData() {
     
       wclear(m_Win);
       box(m_Win, 0, 0);

       wmove(m_Win, 2,20);
       wprintw(m_Win, "SENSOR READINGS");
       wmove(m_Win, 2, 10);
       int pos = 4;    
       for(auto it : m_SensorValues) {
           wmove(m_Win, pos, 10);
           wprintw(m_Win, "%s : ", it.first.c_str());
           wmove(m_Win, pos, 35);
           if(it.second == -2000) {
              wprintw(m_Win, "%s", "Data not available.");
	   } else if (it.second == -3000) {
	      wprintw(m_Win, "%s", "On");
           } else if (it.second == -4000) {
              wprintw(m_Win, "%s", "Off");
           } else {
              wprintw(m_Win, "%f", it.second);
           }
           pos+=2;
       }

       wrefresh(m_Win);
    }

    SensorData getValues() {
        return m_SensorValues;
    }

private:
    
    float m_TimeBetweenPrints;
    float m_RudderAngle = -2000;
    float m_WingsailAngle = -2000;
    float m_WindSpeed = -2000;
    float m_WindDirection = -2000;
    float m_WindTemp = -2000;

    SensorData m_SensorValues;

    WINDOW* m_Win;

};


CANService canService;
MessageBus msgBus;

void messageLoop() {
    msgBus.run();
}

std::string FloatToString (float number) {
    std::ostringstream buff;
    buff<<number;
    return buff.str();
}


std::map<std::string, std::string> menuValues;
std::map<std::string, std::string> lastSentValues;
typedef std::map<std::string, std::string>::iterator menuIter;

void printInputMenu(WINDOW* win, menuIter highlightedItem) {
    wclear(win);
    box(win, 0,0);

    wmove(win, 2, 20);
    wprintw(win, "ACTUATOR COMMANDS");
    int pos = 4;
    for(auto it : menuValues) {
        if(it == *highlightedItem) {
            wattron(win, A_REVERSE);
            mvwprintw(win, pos, 5, "%s", it.first.c_str());
            wattroff(win, A_REVERSE);
            wprintw(win, "\t :");
        } else {
            mvwprintw(win, pos, 5, "%s", it.first.c_str());
            wprintw(win, "\t :");
        }
        mvwprintw(win, pos, 30, "%s", it.second.c_str());
        
        pos+=2;
    }

    mvwprintw(win, pos, 20, "PRESS ENTER TO SEND");
    wrefresh(win);
    
} 

void sendActuatorCommands() {
    CanMsg Cmsg;
    Cmsg.id = 700;
    Cmsg.header.ide = 0;
    Cmsg.header.length = 8;

    uint16_t rudderAngle16;
    uint16_t wingsailAngle16;
    uint16_t windvaneAngle16;

    float ratio = 65535 / 60;

    for(auto& it : menuValues) {
        if(it.second.empty()) {
            it.second = lastSentValues[it.first];
        }
    }

    try {
        rudderAngle16 = stoi(menuValues["Rudder Angle"]);
        wingsailAngle16 = stoi(menuValues["Wingsail Angle"]);
        windvaneAngle16 = stoi(menuValues["Windvane Angle"]);
    } catch(std::invalid_argument ex) {
        std::cout << std::endl << "Actuator commands only works with integers." << std::endl << std::endl;
    }
    
    rudderAngle16 *= ratio;
    wingsailAngle16 *= ratio;
    windvaneAngle16 *= ratio;

    Cmsg.data[0] = rudderAngle16 & 0xff;
    Cmsg.data[1] = rudderAngle16 >> 8;
    Cmsg.data[2] = wingsailAngle16 & 0xff;
    Cmsg.data[3] = wingsailAngle16 >> 8;
    Cmsg.data[4] = windvaneAngle16 & 0xff;
    Cmsg.data[5] = windvaneAngle16 >> 8;

    canService.sendCANMessage(Cmsg);

    lastSentValues = menuValues;
}

int main() {

    // Initialize Ncurses
    initscr();

    // Comment out this line if not running on the pi
    // otherwise program will crash.
    auto future = canService.start();

    SensorDataReceiver sensorReceiver(msgBus, 250);
    CANWindsensorNode windSensor(msgBus, canService, 500);
    //CANFeedbackReceiver feedBack(msgBus, canService, 500); Old code, replaced by arduino node
    CANArduinoNode arduino (msgBus, canService, 500);
    windSensor.start();
		arduino.start ();

    std::thread thr(messageLoop);
    thr.detach();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    sensorReceiver.printSensorData();

    menuValues["Rudder Angle"] = "";
    menuValues["Wingsail Angle"] = "";
    menuValues["Windvane Angle"] = "";
  
    menuIter highlighted = menuValues.begin();

    SensorData values = sensorReceiver.getValues();
    WINDOW* inputWin  = newwin(8+2*menuValues.size(),60, 2*values.size() + 10,2);
    keypad(inputWin, TRUE);
    cbreak();

    printInputMenu(inputWin, highlighted);
    while(1) {

        int c = wgetch(inputWin);

        if(isdigit(c)) {
            c -= 48;
            if(highlighted->second.size() <= MAX_INPUT) {
                highlighted->second += std::to_string(c);
            }
        }

        switch(c) {
            case KEY_DOWN:
                if(highlighted != --menuValues.end()) {
                    highlighted++;
                }
                break;
            case KEY_UP:
                if(highlighted != menuValues.begin()) {
                    --highlighted;
                }
                break;
            case BACKSPACE:
                if(!highlighted->second.empty()) {
                    highlighted->second.pop_back();
                }
                break;
            case ENTER:
                sendActuatorCommands();
                for(auto& it : menuValues) {
                    it.second = "";
                }
                break;
            case TAB:
                if(highlighted == --menuValues.end()) {
                    highlighted = menuValues.begin();
                } else {
                    ++highlighted;
                }
        }
        printInputMenu(inputWin, highlighted);
    }

    
    endwin();
    
}
