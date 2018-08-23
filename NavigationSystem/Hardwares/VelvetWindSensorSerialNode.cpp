/****************************************************************************************
*
* File:
*       VelvetWindSensorSerialNode.cpp
*
* Purpose:
*        Process messages from the serial connection to the Arduino
*
* Developer Notes:
*        The arduino on the velvet is sending two different data as for now:
*        windsensor and compass.
*
***************************************************************************************/

#include "VelvetWindSensorSerialNode.h"
#include "../MessageBus/MessageTypes.h"
#include "../Hardwares/Serial_Connection/serial_interface.h"

const float DATA_OUT_OF_RANGE = -2000; // as uint16_t, cannot use -2000.

VelvetWindSensorSerialNode::VelvetWindSensorSerialNode(MessageBus& messageBus, DBHandler& dbhandler) :
ActiveNode(NodeID::VelvetWindSensorSerial, messageBus), m_LoopTime (0.5), m_db(dbhandler)
{
        m_apparentWindDir = DATA_OUT_OF_RANGE;
    messageBus.registerNode(*this, MessageType::ServerConfigsReceived);

}

VelvetWindSensorSerialNode::~VelvetWindSensorSerialNode(){

}

bool VelvetWindSensorSerialNode::init() {
    updateConfigsFromDB();
    return true;
}

void VelvetWindSensorSerialNode::updateConfigsFromDB()
{
    //m_LoopTime = m_db.retrieveCellAsDouble("config_can_arduino","1","loop_time");
}

void VelvetWindSensorSerialNode::processMessage (const Message* message){
    if(message->messageType() == MessageType::ServerConfigsReceived)
    {
        updateConfigsFromDB();
    }
}


void VelvetWindSensorSerialNode::start() {
    runThread(VelvetWindSensorSerialNodeThreadFunc);
}

void VelvetWindSensorSerialNode::VelvetWindSensorSerialNodeThreadFunc(ActiveNode* nodePtr) {


    VelvetWindSensorSerialNode* node = dynamic_cast<VelvetWindSensorSerialNode*> (nodePtr);
    Timer timer;
    timer.start();

    while(true) {
        std::string str_portname = "/dev/ttyACM2";        
        const char *portname = str_portname.c_str();
        int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0)
        {
            std::cout << "error %d opening %s: %s" << errno << portname << strerror (errno) << std::endl;
            //return -1;
        }
        set_interface_attribs (fd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
        set_blocking (fd, 0);                // set no blocking

        usleep ((50) * 100);             // sleep enough to receive 8:  approx 100 uS per char transmit
        char buf [120];
        read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
        std::vector <std::string> tokens;
            std::stringstream check(buf);
            std::string tmp_string;
            while(getline(check, tmp_string, '\n')) //tokenize line per line
            {
                tokens.push_back(tmp_string);
            }
            std::string substring = "Value_in_degree: ";
            std::vector <std::string> subtokens;
            for (uint i=0; i<tokens.size();i++) {
                if (tokens[i].find(substring) != std::string::npos) {
                    // retokenize with ' '
                    std::stringstream subcheck(tokens[i]);
                    std::string tmp_sub_string;
                    while(getline(subcheck, tmp_sub_string, ' ')) 
                    {
                        subtokens.push_back(tmp_sub_string);
                    }
                }                
            }
                                    
            node->m_lock.lock(); // kept the lock after the reading part
                                 // should be thread safe
            if (subtokens.size()>=2) {
                node->m_apparentWindDir = ::atof(subtokens[1].c_str()); //should be the position of the value
            } else {
                node->m_apparentWindDir = DATA_OUT_OF_RANGE;
            }

            if( not (node->m_apparentWindDir == DATA_OUT_OF_RANGE) )
            {
                //Pushing hardcoded speed and temperature for now
                //the sensor on the velvet only give apparent wind angle
                MessagePtr windData = std::make_unique<WindDataMsg>( node->m_apparentWindDir, 4, 20);
                node->m_MsgBus.sendMessage(std::move(windData));
            }
        node->m_lock.unlock();

        timer.sleepUntil(node->m_LoopTime);
        timer.reset();
    }
}
