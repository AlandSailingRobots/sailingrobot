#include <ncurses.h>
#include <unordered_map>
#include <thread>
#include <chrono>
#include <cctype>
#include <sstream>
#include <map>

#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"
#include "MessageBus/MessageTypes.h"
#include "MessageBus/ActiveNode.h"
#include "MessageBus/NodeIDs.h"
#include "Messages/MarineSensorDataMsg.h"
#include "Messages/DataRequestMsg.h"
#include "Hardwares/MarineSensorNode.h"
#include "Math/Utility.h"

#include "SystemServices/Logger.h"

#include "unistd.h"

#define DATA_OUT_OF_RANGE 		-2000


typedef std::unordered_map<std::string, float> SensorData;

/*
    To add new sensors, the sensors should probably send out their data
    on the message bus. Then do the following:
        * Register for the new message
        * Process it and store its data
        * Add the new member fields to the print function.
*/

class I2CDataReceiver : public Node
{
public:
    I2CDataReceiver(MessageBus& msgBus, float timeBetweenPrints) : 
    Node(NodeID::None, msgBus), m_TimeBetweenPrints(timeBetweenPrints)
    {
        msgBus.registerNode(*this, MessageType::MarineSensorData);
        msgBus.registerNode(*this, MessageType::CompassData);

        m_SensorValues["Temperature"] = DATA_OUT_OF_RANGE;
        m_SensorValues["Conductivity"] = DATA_OUT_OF_RANGE;
        m_SensorValues["Ph"] = DATA_OUT_OF_RANGE;
        m_SensorValues["Salidety"] = DATA_OUT_OF_RANGE;

				
        m_Win = newwin(6+2*m_SensorValues.size(),60,1,2);
        
        box(m_Win,0,0);
        keypad(m_Win, FALSE);
        wrefresh(m_Win);
    }

    bool init()
    {
        return true;
    }

    void processMessage(const Message* message)
    {		
        MessageType type = message->messageType();
		Logger::info (msgToString(type));
        if(type == MessageType::MarineSensorData)
        {
            const MarineSensorDataMsg* actmsg = dynamic_cast<const MarineSensorDataMsg*>(message);
            m_SensorValues["Temperature"] = actmsg->temperature();
            m_SensorValues["Conductivity"] = actmsg->conductivity();
            m_SensorValues["Ph"] = actmsg->ph();
            m_SensorValues["Salidety"] = actmsg->salidety();
        }
		printSensorData();
    }

    void printSensorData()
    {
        wclear(m_Win);
        box(m_Win, 0, 0);

        wmove(m_Win, 2,20);
        wprintw(m_Win, "SENSOR READINGS");
        wmove(m_Win, 2, 10);
        int pos = 4;    
        for(auto it : m_SensorValues)
        {
            wmove(m_Win, pos, 10);
            wprintw(m_Win, "%s : ", it.first.c_str());
            wmove(m_Win, pos, 35);
            if(it.second == DATA_OUT_OF_RANGE)
            {
              wprintw(m_Win, "%s", "Data not available.");
	        }
            else if (it.second == -3000)
            {
	            wprintw(m_Win, "%s", "On");
            }
            else if (it.second == -4000)
            {
              wprintw(m_Win, "%s", "Off");
            }
            else
            {
              wprintw(m_Win, "%f", it.second);
            }
            pos+=2;
       }
       wrefresh(m_Win);
    }

	SensorData getValues()
	{
		return m_SensorValues;
	}

private:
    float m_TimeBetweenPrints;
    SensorData m_SensorValues;
    WINDOW* m_Win;
};


MessageBus msgBus;
void messageLoop()
{
    msgBus.run();
}


std::string FloatToString (float number)
{
    std::ostringstream buff;
    buff<<number;
    return buff.str();
}



int main()
{
	Logger::init ("I2CLog.log");
	
	// Initialize Ncurses
	initscr();
	
	NodeID marineSensorID;
	
	I2CDataReceiver sensorReceiver(msgBus, 250);
	MarineSensorNode MarineSensors(msgBus, 10);
	marineSensorID = MarineSensors.nodeID ();
	
	if(MarineSensors.init())
	{
		Logger::info("Marine sensor node init successfull");
	}
	else
	{
		std::cout << "Marine sensor node init failed\n";
	}

	
	std::thread thr(messageLoop);
	thr.detach();

	std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    //main loop
	while (1)
	{
		MessagePtr dataRequest = std::make_unique<DataRequestMsg>(marineSensorID);
		msgBus.sendMessage(std::move(dataRequest));
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}

    endwin();
}
