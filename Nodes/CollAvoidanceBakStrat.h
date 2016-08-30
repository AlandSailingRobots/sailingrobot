/****************************************************************************************
 *
 * File:
 * 		CollisionAvoidanceNode.cpp
 *
 * Purpose:
 *		Compute the new path in case of collision avoidance.
 *
 * Developer Notes:
 *
 ***************************************************************************************/

#pragma once

#include "Nodes/Node.h"
#include "Messages/VesselStateMsg.h"
#include "Messages/ObstacleVectorMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/CollisionAvoidanceMsg.h"
#include "utility/Utility.h"
#include <math.h>
#include "libs/Eigen/Dense"

#define MAXIMUM_SENSOR_RANGE 100.0 // in meters
#define SENSOR_ARC_ANGLE M_PI/3 // Every angle is in radian
#define SENSOR_HEADING_RELATIVE_TO_BOAT 0.0 // There might be several sensors (not tested, best to stay = 0)
#define CHANNEL_RADIUS 40.0 // in meters
#define SAFE_DISTANCE 50.0 // in meters

#define UNIT_DEGREE 1
#define UNIT_RADIANS 0
#define STANDALONE_DRAW_NEW_FIGURE 0
#define STANDALONE_DRAW_NEW_FIGURE_12 0
#define STANDALONE_DRAW_NEW_FIGURE_10 10
#define STANDALONE_DRAW_NEW_FIGURE_11 11
#define STANDALONE_DRAW_NEW_FIGURE_UNIT_TEST 20
#define STANDALONE_DRAW_USE_EXISTING 1

#define DRAWING_ORIGIN_LON_RAD 0.34771144 // in rads
#define DRAWING_ORIGIN_LAT_RAD 1.04906922 // in rads

/**
 * Structure which contains every used data from the sensors.
 */
struct SensorData {
    Eigen::Vector2d gpsPos;
    double speed;
    double compHeading;
    double gpsHeading;
    double windDirection;
    double windSpeed; /**< meter par second */
    int pitch;
    int roll;

    //TODO : put the real inputs here (video, sonar or whatever)
    std::vector<ObstacleData> detectedObstacles;
};
struct FollowedLine {
    Eigen::Vector2d startPoint;
    Eigen::Vector2d endPoint;
};
/**
 * Commands output in radians
 */
struct CommandOutput {
    double deltaRudder;
    double deltaSail;
};
/**
 * The datas output by this node
 */
struct WaypointOutput{
    Eigen::Vector2d startPoint;
    Eigen::Vector2d midPoint;
    Eigen::Vector2d endPoint;
};
struct ObstacleRealPosition{
    std::vector<double> lon;
    std::vector<double> lat;
};

class CollisionAvoidanceNode : public Node {
public:
    CollisionAvoidanceNode(MessageBus& msgBus);
    ~CollisionAvoidanceNode() {};

    /**
     * Initialize the values for simulation or drawing (not really used apart from DEBUG)
     * Setup sailing zone
     * @return success
     */
    bool init();

    /**
     * Process the messages from the message bus.
     * Called by message bus.
     * @param message
     * @return
     */
    void processMessage(const Message* message);

protected:

    FollowedLine m_followedLine;
    std::vector<Eigen::Vector2d> m_sailingZone;
    SensorData m_sensorOutput;
    double m_loop_id;
    double m_simu_without_simulator;
    ObstacleRealPosition m_obs_coords;
    /**
     * Number of waypoints recieved\n
     * This is necessary to prevent problems in the disscussion between
     * WaypoinMgrNode and CollisionAvoidanceNode.\n
     * Else the boat will avoid obstacles seen just after initialisation around Africa.
     */
    double m_number_of_wp_recieved;

    /**
     * Process the message that constains the state of the boat
     * @param msg
     */
    void processVesselState(VesselStateMsg* msg);

    /**
     * Proccess the messages from the color_detection node
     * @param msg
     */
    void processObstacleData(ObstacleVectorMsg* msg);

    /**
     * Process the messages from the waypoint manager in order to know the
     * line the boat is currently following.
     * @param msg
     */
    void processWaypointData(WaypointDataMsg* msg);

    /**
     * The most important function of the class, it calls al the others.
     * It's called by processObstacleData.
     *
     * It uses m_followedLine and m_sensorOutput as main input
     * Then it sends the commands by itself
     */
    void run();

    /**
     * check if the obstacle is a problem :
     *      The obstacle heading indicate that it's on the path of the baat.
     *      The path heading is the heading of followedLine, the arc in which the obstacle is a problem is
     *        atan(CHANNEL_RADIUS/50).
     * @return
     */
    bool check_obstacles();

    WaypointOutput compute_new_path();

    std::vector<ObstacleData> simulateObstacle(ObstacleRealPosition obstacle_coords, // in rads
                                               int unit);
    bool createObstacleDataCircle(double obsGpsLat, //rads
                                  double obsGpsLon, //rads
                                  double obstacleRadius, //meters
                                  ObstacleData & obstacle);

    /**
 * Draw the state of the boat on vibes-viewer.
 * VIBes : http://enstabretagnerobotics.github.io/VIBES/
 *
 * The offset to be closer to 0 is because vibes has some precision problem when the coordinates are not close to 0.
 * VIBes could be improved and it's open-source : go on if you want.
 */
    void drawState();
    std::vector<double> getEigenVectorLine(std::vector<Eigen::Vector2d> vec,int line);
    void drawBoat(SensorData & sensorData,std::string color);
    void drawChannel(FollowedLine & followedLine);
    void vibesFigureHandler(std::string name, int option);
};
