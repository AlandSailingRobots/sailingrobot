/*
#include <ncurses.h>
#include <unordered_map>
#include <thread>
#include <sstream>
*/

#include "../Hardwares/GPSDNode.h"
//#include "../Hardwares/VelvetWindSensorSerialNode.h"
#include "../SystemServices/Timer.h"
#include "../SystemServices/Logger.h"

#include "../MessageBus/MessageTypes.h"
#include "../MessageBus/MessageBus.h"
#include "../MessageBus/NodeIDs.h"

#include "../Messages/WindDataMsg.h"

#include "../Math/Utility.h"

#include "../Hardwares/Serial_Connection/serial_interface.h"
#include <vector>



class VelvetSensorReceiver: public ActiveNode {

const float DATA_OUT_OF_RANGE = -2000; // as uint16_t, cannot use -2000.



public:
    
	VelvetSensorReceiver(MessageBus& messageBus) :
		ActiveNode(NodeID::VelvetWindSensorSerial, messageBus), m_msgBus(messageBus), m_LoopTime (0.5)
	{
	        m_apparentWindDir = DATA_OUT_OF_RANGE;
	    messageBus.registerNode(*this, MessageType::ServerConfigsReceived);

	}

	bool init() {
	    updateConfigsFromDB();
	    return true;
	}

	void updateConfigsFromDB()
	{
	    //m_LoopTime = m_db.retrieveCellAsDouble("config_can_arduino","1","loop_time");
	}

	void processMessage (const Message* message){
	    if(message->messageType() == MessageType::ServerConfigsReceived)
	    {
	        updateConfigsFromDB();
	    }
	}

	void nstart(ActiveNode* nodePtr) {
	    VelvetSensorReceiver* node = dynamic_cast<VelvetSensorReceiver*> (nodePtr);
	    Timer timer;
	    timer.start();

	    while(true) {
	        //node->m_lock.lock();

	        std::string str = "/dev/ttyACM2";
	        const char *portname = str.c_str();
	        Logger::info("Reading port: %s", portname);
	        int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	        if (fd < 0)
	        {
	            std::cout << "error %d opening %s: %s" << errno << portname << strerror (errno) << std::endl;
	        }
	        set_interface_attribs (fd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
	        set_blocking (fd, 0);                // set no blocking

	        usleep ((50) * 100);             // sleep enough to receive 8:  approx 100 uS per char transmit
	        char buf [120];
	        //int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
	        read (fd, buf, sizeof buf);
	        //std::cout << "Data incoming: " << buf << std::endl;
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
				    //std::cout << "found value!!!!!!!!!!!!! " << '\n';
				    // retokenize with ' '
			        std::stringstream subcheck(tokens[i]);
			        std::string tmp_sub_string;
			        while(getline(subcheck, tmp_sub_string, ' ')) 
			        {
			        	subtokens.push_back(tmp_sub_string);
			        }
				}
				Logger::info("Number of subtokens: %d", subtokens.size());
				/*for (uint j=0;j<subtokens.size();j++) {
					Logger::info("Before display, step: %d", j);
					Logger::info("Subtoken: %s", subtokens[i]);
				}*/
				/*for (auto i=subtokens.begin();i!=subtokens.end();++i){
					Logger::info("Subtoken: %s", (*i).c_str());
				}*/
	        }
	        
	        if (subtokens.size()>=2) {
	            Logger::info("Found value: %s", subtokens[1].c_str()); // 1 should be position of the value
	        } else {
	        	Logger::info("Found no token corresponding to 'value_in_degree' ");
	        }
      


	        //Logger::info("Data incoming: %s, number of char read=%d", buf, n);
	                    
	        node->m_lock.lock(); // kept the lock after the reading part
	                             // should be thread safe
	        if (subtokens.size()>=2) {
	        	node->m_apparentWindDir = ::atof(subtokens[1].c_str());
	        } else {
	        	node->m_apparentWindDir = DATA_OUT_OF_RANGE;
	        }
	        Logger::info("m_apparentWindDir = %f", node->m_apparentWindDir);

	        if( not (node->m_apparentWindDir == DATA_OUT_OF_RANGE) )
	        {
	            MessagePtr windData = std::make_unique<WindDataMsg>( node->m_apparentWindDir, 1, 25);
	            node->m_MsgBus.sendMessage(std::move(windData));
	        }
	        node->m_lock.unlock();

	        timer.sleepUntil(node->m_LoopTime);
	        timer.reset();
	    }
	}

	void start() {
		return;
	}



private:
	    MessageBus& m_msgBus;
	    double m_LoopTime;
	    float m_apparentWindDir;
	    std::mutex m_lock;

	};




enum class NodeImportance {
	CRITICAL,
	NOT_CRITICAL
};


///----------------------------------------------------------------------------------
/// Initialises a node and shutsdown the program if a critical node fails.
///
/// @param node 			A pointer to the node to initialise
/// @param nodeName 		A string name of the node, for logging purposes.
/// @param importance 		Whether the node is a critcal node or not critical. If a
///							critical node fails to initialise the program will
///							shutdown.
///
///----------------------------------------------------------------------------------
void initialiseNode(Node& node, const char* nodeName, NodeImportance importance)
{
	if(node.init())
	{
		Logger::info("Node: %s - init\t[OK]", nodeName);
	}
	else
	{
		Logger::error("Node: %s - init\t\t[FAILED]", nodeName);

		if(importance == NodeImportance::CRITICAL)
		{
			Logger::error("Critical node failed to initialise, shutting down");
			//Logger::shutdown();
			exit(1);
		}
	}
}


///----------------------------------------------------------------------------------
/// Entry point, can accept one argument containing a relative path to the database.
///
///----------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
	printf("================================================================================\n");
	printf("\t\t\t\tVelvet Wind Sensor Integration Test\n");
	printf("\n");
	printf("================================================================================\n");

	// Database Path
	std::string db_path;
	if (argc < 2)
	{
		db_path = "../asr.db";
	}
	else
	{
		db_path = std::string(argv[1]);
	}

	MessageBus messageBus;

	// Initialise logger
	if (Logger::init())
	{
		Logger::info("Built on %s at %s", __DATE__, __TIME__);
    	Logger::info("Velvet Sensor Integration Test");
		Logger::info("Logger init\t\t[OK]");
	}
	else
	{
		Logger::error("Logger init\t\t[FAILED]");
	}

	// Declare nodes
	//-------------------------------------------------------------------------------


    VelvetSensorReceiver velvetSensorReceiver(messageBus);
	

	// Initialise nodes
	//-------------------------------------------------------------------------------
    initialiseNode(velvetSensorReceiver, "Velvet Sensors", NodeImportance::NOT_CRITICAL);


	// Start active nodes
	//-------------------------------------------------------------------------------
    velvetSensorReceiver.nstart(&velvetSensorReceiver);


	// Begins running the message bus
	//-------------------------------------------------------------------------------
	Logger::info("Message bus started!");
	messageBus.run();


	exit(0);
}

