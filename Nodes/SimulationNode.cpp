/****************************************************************************************
 *
 * File:
 * 		SimulationNode.cpp
 *
 * Purpose:
 *		Discuss with simulation via TCP, create message for the program from the
 *    data from simulation and send the command data to the simulation.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "SimulationNode.h"

// For std::this_thread
#include <chrono>
#include <thread>
#include <memory>

#include <sys/types.h>
#include <netdb.h>
#include <fcntl.h>
#include <strings.h> //bzero strerror
#include <cerrno>
#include <cstring>
#include <unistd.h>
#include <cmath>
#include "SystemServices/Logger.h"
#include "utility/SysClock.h"
#include "SystemServices/Logger.h"

#define BASE_SLEEP_MS 100
#define COUNT_COMPASSDATA_MSG 1
#define COUNT_GPSDATA_MSG 1
#define COUNT_WINDDATA_MSG 1
#define COUNT_ARDUINO_MSG 1
#define COUNT_OBSTACLE_MSG 1
#define DRAW_STATE_WITH_VIBES 1

SimulationNode::SimulationNode(MessageBus& msgBus)
	: ActiveNode(NodeID::Simulator, msgBus),
		m_CompassHeading(0), m_CompassPitch(0), m_CompassRoll(0),
		m_GPSHasFix(false), m_GPSOnline(false), m_GPSLat(0), m_GPSLon(0), m_GPSUnixTime(0), m_GPSSpeed(0),
		m_GPSHeading(0), m_WindDir(0), m_WindSpeed(0), m_WindTemp(0), m_ArduinoPressure(0),
		m_ArduinoRudder(0),m_ArduinoSheet(0),m_ArduinoBattery(0),m_ArduinoRC(0),m_rudder(0),m_sail(0),m_count_sleep(0)
{
  m_MsgBus.registerNode(*this, MessageType::ActuatorPosition);
}

void SimulationNode::start()
{
	runThread(SimulationThreadFunc);
}

bool SimulationNode::init(){


		if (init_socket(6400)<0)
		{
			return false;
		}


		Logger::info("Size of float on this platform %d",(int)sizeof(float)*8 );
		Logger::info("Address: %s Port: %d", inet_ntoa(m_handler_socket_server.info_me.sin_addr),6400);

		//================
		unsigned int clntLen;            /* Length of client address data structure */
		/* Set the size of the in-out parameter */
		clntLen = sizeof(m_handler_socket_client.info_me);
		/* Wait for a client to connect */

		Logger::info("Waiting for simulation client...\n");

		if ((m_handler_socket_client.sockfd = accept(m_handler_socket_server.sockfd, (struct sockaddr *) &(m_handler_socket_client.info_me),
					 &clntLen)) < 0)
	  {
			  Logger::error("%s Error in accept() failed: %s",__PRETTY_FUNCTION__,strerror(errno));
				return false;
	  }

    // Set socket as non-blocking
		fcntl(m_handler_socket_client.sockfd, F_SETFL, O_NONBLOCK);
    fcntl(m_handler_socket_server.sockfd, F_SETFL, O_NONBLOCK);
    Logger::info("Handling Simulation client from : %s\n", inet_ntoa(m_handler_socket_client.info_me.sin_addr));

    // code can be simplified here
    if(!init_obstacles()){
        return false;
    }

	return true;
}

int SimulationNode::init_socket(int port)
{
    // init socket
    m_handler_socket_server.sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_handler_socket_server.sockfd == -1)
    {
				Logger::error("%s Error in initialise socket: %s",__PRETTY_FUNCTION__,strerror(errno));
        return -1;
    }

    // configure
    bzero((char *)&m_handler_socket_server.info_me, sizeof(m_handler_socket_server.info_me));
    m_handler_socket_server.info_me.sin_family = AF_INET;
    m_handler_socket_server.info_me.sin_port = htons(port);
    m_handler_socket_server.info_me.sin_addr.s_addr = htonl(INADDR_ANY);

    if( bind(m_handler_socket_server.sockfd, (struct sockaddr*) &m_handler_socket_server.info_me,
             sizeof(m_handler_socket_server.info_me)) == -1)
    {
        Logger::error("%s Error in initialise socket bind: %s",__PRETTY_FUNCTION__,strerror(errno));
        return -1;
    }

    if (listen(m_handler_socket_server.sockfd,1) < 0)
    {
        Logger::error("%s Error in initialise socket listen: %s",__PRETTY_FUNCTION__,strerror(errno));
        return -1;
    }
    return 0;
}

bool SimulationNode::init_obstacles(){
    m_obstacles_coords.lon.clear();
    m_obstacles_coords.lat.clear();
//    m_obstacles_coords.lon = {19.92207992094336,19.92193812140613,19.921326,19.921705,19.92185527391433,19.92238372564315,19.920391}; // deg
//    m_obstacles_coords.lat = {60.10531054974476,60.10533773205465,60.105284,60.105256,60.10524774529737,60.10521566220767,60.107564}; // deg

    m_obstacles_coords.lon = {19.9229765, 19.9228048, 19.9225795, 19.9223757, 19.9221718, 19.9220002, 19.9218071, 19.9216354, 19.9214852, 19.9213350, 19.9211848, 19.9210238, 19.9208844, 19.9229014, 19.9226868, 19.9224937, 19.9222791, 19.9220860, 19.9218714, 19.9217534};
    m_obstacles_coords.lat = {60.1070290, 60.1070397, 60.1070290, 60.1070290, 60.1070450, 60.1070397, 60.1070610, 60.1070557, 60.1070664, 60.1070664, 60.1070717, 60.1070664, 60.1070664, 60.1070183, 60.1070290, 60.1070290, 60.1070397, 60.1070343, 60.1070503, 60.1070610};

    if(m_obstacles_coords.lon.size()!=m_obstacles_coords.lat.size()){
        printf("[ERROR](createObstacleMessage) obstacle vectors not of the same size\n");
        return false;
    }
    return true;
}

void SimulationNode::processMessage(const Message* msg)
{
	MessageType type = msg->messageType();

	switch(type)
	{
	case MessageType::ActuatorPosition:
//    Logger::info("(SimulationNode) ActuatorPosition");
		processActuatorPositionMessage((ActuatorPositionMsg*)msg);
		break;
	default:
		return;
	}
}

void SimulationNode::processActuatorPositionMessage(ActuatorPositionMsg* msg)
{
	m_rudder = msg->rudderPosition();
	m_sail = msg->sailPosition();
}

void SimulationNode::createCompassMessage()
{
	m_CompassHeading = static_cast<int>(m_data_receive.headingVector[0]/10.f+0.5);
	m_CompassPitch = 0;
	m_CompassRoll = 0;

	if (m_count_sleep % COUNT_COMPASSDATA_MSG==0){

		MessagePtr msg = std::make_unique<CompassDataMsg>(CompassDataMsg( m_CompassHeading , m_CompassPitch, m_CompassRoll));

	  	m_MsgBus.sendMessage(std::move(msg));
        Logger::info("(SimulationNode) Sent compass data : %d deg", m_CompassHeading);
  }
}

void SimulationNode::createGPSMessage()
{

	double knots = 1.94384;
	m_GPSHasFix = true;
	m_GPSOnline = true;
	m_GPSLat = m_data_receive.latitude;
	m_GPSLon = m_data_receive.longitude;
	m_GPSUnixTime = SysClock::unixTime();
	m_GPSSpeed = m_data_receive.speed_knot/knots;
	m_GPSHeading = m_data_receive.course_real;
	m_GPSSatellite = 0;
	GPSMode mode = GPSMode::LatLonOk;

	if (m_count_sleep % COUNT_GPSDATA_MSG==0)
	{
		MessagePtr msg = std::make_unique<GPSDataMsg>(GPSDataMsg(m_GPSHasFix, m_GPSOnline, m_GPSLat, m_GPSLon, m_GPSUnixTime, m_GPSSpeed, m_GPSHeading, m_GPSSatellite, mode));
		m_MsgBus.sendMessage(std::move(msg));
//        Logger::info("(SimulationNode) Sent GPSDataMsg : (%f,%f)",m_GPSLon,m_GPSLat);
	}
}

void SimulationNode::createWindMessage()
{
	m_WindDir = m_data_receive.windDirection;
	m_WindSpeed = m_data_receive.windSpeed;
	m_WindTemp = m_data_receive.windTemperature;

	if (m_count_sleep % COUNT_WINDDATA_MSG==0)
	{
		MessagePtr windData = std::make_unique<WindDataMsg>(WindDataMsg(m_WindDir, m_WindSpeed, m_WindTemp));
		m_MsgBus.sendMessage(std::move(windData));
	}
}

void SimulationNode::createArduinoMessage()
{
	m_ArduinoPressure = m_data_receive.pressure;
	m_ArduinoRudder = m_data_receive.rudder;
	m_ArduinoSheet = m_data_receive.sheet;
	m_ArduinoBattery = m_data_receive.battery;

	if (m_count_sleep % COUNT_ARDUINO_MSG==0)
	{
		MessagePtr msg = std::make_unique<ArduinoDataMsg>(ArduinoDataMsg(m_ArduinoPressure, m_ArduinoRudder, m_ArduinoSheet, m_ArduinoBattery, m_ArduinoRC ));
		m_MsgBus.sendMessage(std::move(msg));
	}
}

void SimulationNode::createObstacleMessage(){

    std::vector<ObstacleData> obstacles;

    //Obstacle settings
    const double obstacleRadius = 5; //meters
    ObstacleData obsData;

    // static cast to disable warnings
    // for each coordinates, create an obstacle
    for (int i = 0; i < static_cast<signed>(m_obstacles_coords.lon.size()); ++i) {
        // transformation in radians
        const double obsGpsLon = Utility::degreeToRadian(m_obstacles_coords.lon[i]);
        const double obsGpsLat = Utility::degreeToRadian(m_obstacles_coords.lat[i]);
        bool isCreated = createObstacleDataCircle(obsGpsLon, //rads
                                                  obsGpsLat, //rads
                                                  obstacleRadius, //m
                                                  obsData);
        if(isCreated){
//            Logger::info("(SimulationNode) Obstacle created and appened");
            obstacles.push_back(obsData);
        }
        else{
//            Logger::info("(SimulationNode) Obstacle not seen");
        }
    }

    if (m_count_sleep % COUNT_OBSTACLE_MSG==0)
    {
//        Logger::info("(SimulationNode) Sent obstacle size : %d", obstacles.size());
//        for (auto &&obstacle : obstacles) {
//            Logger::info("(SimulationNode) ==Detected Obstacle==");
//            Logger::info("(SimulationNode) obstacle.LeftBoundheadingRelativeToBoat = %f",obstacle.LeftBoundheadingRelativeToBoat);
//            Logger::info("(SimulationNode) obstacle.RightBoundheadingRelativeToBoat = %f",obstacle.RightBoundheadingRelativeToBoat);
//            Logger::info("(SimulationNode) obstacle.minDistanceToObstacle = %f",obstacle.minDistanceToObstacle);
//            Logger::info("(SimulationNode) obstacle.maxDistanceToObstacle = %f",obstacle.maxDistanceToObstacle);
//        }
        MessagePtr msg = std::make_unique<ObstacleVectorMsg>(obstacles);
        m_MsgBus.sendMessage(std::move(msg));
    }
}

bool SimulationNode::createObstacleDataCircle(double obsGpsLon, //rads
                                              double obsGpsLat, //rads
                                              double obstacleRadius, //meters
                                              ObstacleData & obstacle){

    // Conversion
    const double gpsLat = Utility::degreeToRadian(m_GPSLat);
    const double gpsLon = Utility::degreeToRadian(m_GPSLon);
    const double compassHeading = M_PI/2 - Utility::degreeToRadian(m_CompassHeading);

    // Sensor settings
    const double maxDetectionRange = MAXIMUM_SENSOR_RANGE;

    // Simulation
    const double obstacleHeadingRelativeToBoat = Utility::wrapToPi(atan2(obsGpsLat-gpsLat,
                                                                         obsGpsLon-gpsLon),
                                                                   - compassHeading);
    //std::cout << "obstacleHeadingRelativeToBoat = " << obstacleHeadingRelativeToBoat<< "\n";
    const double distFromObstacle = Utility::calculateGPSDistance(gpsLon   ,gpsLat,
                                                                  obsGpsLon,obsGpsLat); // meters
//    Logger::info("(SimulationNode) Obstacle inside sensor arc : %d", std::abs(obstacleHeadingRelativeToBoat) <= SENSOR_ARC_ANGLE);
//    Logger::info("(SimulationNode) Obstacle inside detection range : %d", distFromObstacle<=maxDetectionRange);
    if(std::abs(obstacleHeadingRelativeToBoat) <= SENSOR_ARC_ANGLE && distFromObstacle<=maxDetectionRange) {

        const double leftHeadingRelativeToBoat =  Utility::wrapToPi(atan2(obstacleRadius, distFromObstacle),
                                                                    obstacleHeadingRelativeToBoat);
        const double rightHeadingRelativeToBoat = Utility::wrapToPi(-atan2(obstacleRadius, distFromObstacle),
                                                                    obstacleHeadingRelativeToBoat);
        double minDistanceToObstacle = distFromObstacle - obstacleRadius;
        if(minDistanceToObstacle<=0){
            minDistanceToObstacle = 0.1;
        }
        obstacle = {
                distFromObstacle - obstacleRadius, //double minDistanceToObstacle;
                maxDetectionRange,                 //double maxDistanceToObstacle;
                leftHeadingRelativeToBoat,         //double LeftBoundheadingRelativeToBoat;
                rightHeadingRelativeToBoat,        //double RightBoundheadingRelativeToBoat;
                0,
                0,
                m_GPSLon,
                m_GPSLat,
                static_cast<double>(m_CompassHeading)};
        //std::cout << "I push back\n";
        return true;
    }
    return false;

}

void SimulationNode::processSocketData(){
  createCompassMessage();
  createGPSMessage();
  createWindMessage();
  createArduinoMessage();
  createObstacleMessage();
}

void SimulationNode::setupDataSend(){
  m_data_send.rudder_command = m_rudder*4;
  m_data_send.sheet_command = m_sail*4;
}

void SimulationNode::SimulationThreadFunc(void* nodePtr)
{
	SimulationNode* node = (SimulationNode*)nodePtr;

	int bytes_received = 0;
	struct DATA_SOCKET_RECEIVE dump_data_sock_receive;
	while(true){

    //receive socket from simulation
    bytes_received += read(node->m_handler_socket_client.sockfd,&(node->m_data_receive)+bytes_received,sizeof(struct DATA_SOCKET_RECEIVE)-bytes_received);
    if (bytes_received==sizeof(struct DATA_SOCKET_RECEIVE)){
       bytes_received=0;
			 //flush socket
       while(read(node->m_handler_socket_client.sockfd,&dump_data_sock_receive,sizeof(struct DATA_SOCKET_RECEIVE))>0){};
    }
    if (bytes_received==-1)
      bytes_received=0;

    //clock for sending messages
    if (node->m_count_sleep==10)
		{
			node->m_count_sleep=0;
		}
		node->m_count_sleep++;

    node->processSocketData();
		node->setupDataSend();

    //send data to simulation
    write(node->m_handler_socket_client.sockfd,&(node->m_data_send), sizeof(struct DATA_SOCKET_SEND));

	std::this_thread::sleep_for(std::chrono::milliseconds(BASE_SLEEP_MS));
  }
}
