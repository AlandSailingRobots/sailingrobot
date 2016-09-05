/****************************************************************************************
 *
 * File:
 * 		CollisionAvoidanceNode.cpp
 *
 * Purpose:
 *		Compute the new path in case of collision avoidance.
 *
 * Developer Notes:
 *      Idea : when you see an obstacle, go to the right, else, try to follow the objective.
 *
 ***************************************************************************************/
#include "CollAvoidanceBakStrat.h"

#include "SystemServices/Logger.h"
#include "utility/vibes.h"

#define DRAW_STATE_WITH_VIBES 1
#define WAIT_FOR_X_WAYPOINTS 0

//CONSTRUCTOR
CollisionAvoidanceNode::CollisionAvoidanceNode(MessageBus& msgBus)
        : Node(NodeID::CollisionAvoidance, msgBus){
    msgBus.registerNode(*this, MessageType::ObstacleVector);
    msgBus.registerNode(*this, MessageType::VesselState);
    msgBus.registerNode(*this, MessageType::WaypointData);
}

//PUBLIC FUNCTIONS

bool CollisionAvoidanceNode::init(){
    m_loop_id = 0;
    m_simu_without_simulator= false;
    m_number_of_wp_recieved = 0;
    m_obs_coords.lon = {19.9229765, 19.9228048, 19.9225795, 19.9223757, 19.9221718, 19.9220002, 19.9218071, 19.9216354, 19.9214852, 19.9213350, 19.9211848, 19.9210238, 19.9208844, 19.9229014, 19.9226868, 19.9224937, 19.9222791, 19.9220860, 19.9218714, 19.9217534};
    m_obs_coords.lat = {60.1070290, 60.1070397, 60.1070290, 60.1070290, 60.1070450, 60.1070397, 60.1070610, 60.1070557, 60.1070664, 60.1070664, 60.1070717, 60.1070664, 60.1070664, 60.1070183, 60.1070290, 60.1070290, 60.1070397, 60.1070343, 60.1070503, 60.1070610};

    if(DRAW_STATE_WITH_VIBES){
        vibes::beginDrawing();
        vibesFigureHandler("Simu",STANDALONE_DRAW_NEW_FIGURE);
    }
    return true;
}

void CollisionAvoidanceNode::processMessage(const Message* msg){
    MessageType type = msg->messageType();
    switch(type)
    {
        case MessageType::VesselState:
            processVesselState((VesselStateMsg*)msg);
            break;
        case MessageType::ObstacleVector:
            processObstacleData((ObstacleVectorMsg*)msg);
            break;
        case MessageType::WaypointData:
            processWaypointData((WaypointDataMsg*)msg);
            break;
        default:
            return;
    }
}

//PRIVATE MAIN FUNCTIONS

void CollisionAvoidanceNode::processVesselState(VesselStateMsg* msg){
    //Extraction of data from sensors

//    printf("Received VesselStateMsg\n" );

    //Position and speed
    //The latitude and longitude are easier to compute in radians.
    m_sensorOutput.gpsPos(0) = // x
            Utility::degreeToRadian(msg->longitude());
    m_sensorOutput.gpsPos(1) = // y
            Utility::degreeToRadian(msg->latitude());
    m_sensorOutput.speed = msg->speed();

    //Heading
    // TODO : merge the two heading into one
    //compHeading : degree from north -> radian from east
    m_sensorOutput.compHeading = M_PI / 2
                                 - Utility::degreeToRadian(msg->compassHeading());
    //gpsHeading : degree from north -> radian from east
    m_sensorOutput.gpsHeading = M_PI / 2
                                - Utility::degreeToRadian(msg->gpsHeading());

    //Wind
    //windDirection : degree from east -> radian from east
    m_sensorOutput.windDirection = -Utility::degreeToRadian(msg->windDir());
    m_sensorOutput.windSpeed = msg->windSpeed();

    //Pitch and roll
    m_sensorOutput.pitch = msg->compassPitch();
    m_sensorOutput.roll = msg->compassRoll();

    if(m_simu_without_simulator){
        std::vector<ObstacleData> obsData = simulateObstacle(m_obs_coords,UNIT_DEGREE);
        for (auto &&data : obsData) {
            m_sensorOutput.detectedObstacles.push_back(data);
        }
        run();
    }

//    Logger::info("(Collision Avoidance) Received GPS from VesselState : (%f,%f) deg",m_sensorOutput.gpsPos(0)/M_PI*180,m_sensorOutput.gpsPos(1)/M_PI*180);

    if(DRAW_STATE_WITH_VIBES){
        vibes::selectFigure("Simu");
        vibes::clearFigure();
        drawState();
    }
}

void CollisionAvoidanceNode::processObstacleData(ObstacleVectorMsg* msg){

    if(!m_simu_without_simulator){
        Logger::info("(Collision Avoidance) Received ObstacleData >>> processObstacleData()");
        m_sensorOutput.detectedObstacles = msg->obstacles();
        //std::copy(msg->obstacles().begin(),msg->obstacles().end(),std::back_inserter(m_sensorOutput.detectedObstacles));
//        Logger::info("(Collision Avoidance) Detected obstacle size : %d",m_sensorOutput.detectedObstacles.size());
        for (auto &&obstacle : m_sensorOutput.detectedObstacles) {
            obstacle.boatLonAtDetection = Utility::degreeToRadian(obstacle.boatLonAtDetection);
            obstacle.boatLatAtDetection = Utility::degreeToRadian(obstacle.boatLatAtDetection);
            obstacle.boatHeadingAtDetection = M_PI/2-Utility::degreeToRadian(obstacle.boatHeadingAtDetection);
        }
        if(m_number_of_wp_recieved>=WAIT_FOR_X_WAYPOINTS){
            run();
        }
    }

    // Here it begins !
    // Launch collision avoidance process

}

void CollisionAvoidanceNode::processWaypointData(WaypointDataMsg* msg){
//    Logger::info("(Collision Avoidance) Received WaypointData >>> processWaypointData() ");

    m_followedLine.endPoint(0) = msg->nextLongitude()/180*M_PI;
    m_followedLine.endPoint(1) = msg->nextLatitude()/180*M_PI;
    m_followedLine.startPoint(0) = msg->prevLongitude()/180*M_PI;
    m_followedLine.startPoint(1) = msg->prevLatitude()/180*M_PI;
    if(m_number_of_wp_recieved<WAIT_FOR_X_WAYPOINTS){
        m_number_of_wp_recieved++;
    }
    Logger::info("(Collision Avoidance) Current gps position : (%f deg,%f deg)", m_sensorOutput.gpsPos(0)/M_PI*180,m_sensorOutput.gpsPos(1)/M_PI*180);
    Logger::info("(Collision Avoidance) Received followedLine start point : (%f deg,%f deg)", m_followedLine.startPoint(0)/M_PI*180,m_followedLine.startPoint(1)/M_PI*180);
    Logger::info("(Collision Avoidance) Received followedLine end point : (%f deg,%f deg)", m_followedLine.endPoint(0)/M_PI*180,m_followedLine.endPoint(1)/M_PI*180);
}

void CollisionAvoidanceNode::run() {
    //Note on simulation
    /*
     * For now i don't know of any place in the code where it specified if the code is
     * in a simulated environement or not. So this variable is temporary.
     */

//    Logger::info("Number of detected obstacles : %d", m_sensorOutput.detectedObstacles.size());
//    double minDebug = 10000000000.0;
//    for (auto &&obstacle : m_sensorOutput.detectedObstacles) {
//        minDebug = std::min(minDebug,obstacle.minDistanceToObstacle);
//    }
//    Logger::info("Distance to closest obstacle : %f", minDebug);

    if (check_obstacles()) {
        WaypointOutput wpOut = compute_new_path(); // in rads

        MessagePtr msg = std::make_unique<CollisionAvoidanceMsg>(Utility::radianToDegree(wpOut.startPoint(0)),
                                                                 Utility::radianToDegree(wpOut.startPoint(1)),
                                                                 Utility::radianToDegree(wpOut.midPoint(0)),
                                                                 Utility::radianToDegree(wpOut.midPoint(1)),
                                                                 Utility::radianToDegree(wpOut.endPoint(0)),
                                                                 Utility::radianToDegree(wpOut.endPoint(1))); // in deg
        m_MsgBus.sendMessage(std::move(msg));
        Logger::info("(Collision Avoidance) Sent CollisionAvoidanceMsg");

    }
}

bool CollisionAvoidanceNode::check_obstacles() {
    bool obstacleIsOnPath = false;
    const double pathHeading = atan2(m_followedLine.endPoint(1)-m_followedLine.startPoint(1),
                                     m_followedLine.endPoint(0)-m_followedLine.startPoint(0));
    for (auto &&obstacle :m_sensorOutput.detectedObstacles) {
        const double obstacleHeading = Utility::wrapToPi(Utility::wrapToPi(obstacle.LeftBoundheadingRelativeToBoat,
                                                                           obstacle.RightBoundheadingRelativeToBoat)/2.0,
                                                         m_sensorOutput.compHeading);
//        const double obsAngularWidth =
        if(std::abs(Utility::wrapToPi(obstacleHeading,-pathHeading)) <= atan2(CHANNEL_RADIUS,MAXIMUM_SENSOR_RANGE) ){
            obstacleIsOnPath = true;
        }
    }

    return obstacleIsOnPath;
}

WaypointOutput CollisionAvoidanceNode::compute_new_path() {
    WaypointOutput wpOut;
    const double pathHeading = atan2(m_followedLine.endPoint(1)-m_followedLine.startPoint(1),
                                     m_followedLine.endPoint(0)-m_followedLine.startPoint(0));
    const double shiftLength  = 1.0* CONVERSION_FACTOR_METER_TO_GPS;
//    const double shiftLength = Utility::calculateGPSDistance(m_followedLine.startPoint(0),
//                                                             m_followedLine.startPoint(1),
//                                                             m_followedLine.endPoint(0),
//                                                             m_followedLine.endPoint(1))/30.0
//                               * atan(10.0/180*M_PI) * CONVERSION_FACTOR_METER_TO_GPS;
    const double normalPathHeading = Utility::wrapToPi(pathHeading,M_PI/2);

    const double offsetLength = CHANNEL_RADIUS/2.0*CONVERSION_FACTOR_METER_TO_GPS;
    Eigen::Vector2d shiftStartVector(offsetLength*cos(normalPathHeading),
                                     offsetLength*sin(normalPathHeading)); //shift of 10 degrees
    Eigen::Vector2d shiftCollVector((shiftLength)*cos(normalPathHeading),
                                    (shiftLength+offsetLength)*sin(normalPathHeading)); //shift of 10 degrees
    Eigen::Vector2d collision_avoidance_point = m_followedLine.endPoint+shiftCollVector;

    wpOut.startPoint = m_sensorOutput.gpsPos + shiftStartVector;
    wpOut.midPoint = collision_avoidance_point;
    wpOut.endPoint = collision_avoidance_point; //to keep the data structure only

    return wpOut;
}

//UTILITY
//Simulate
std::vector<ObstacleData> CollisionAvoidanceNode::simulateObstacle(
        ObstacleRealPosition obstacle_coords, int unit){

    std::vector<ObstacleData> obstacles;

    //Obstacle settings
    const double obstacleRadius = 5; //meters
    ObstacleData obsData;

    // static cast to disable warnings
    // for each coordinates, create an obstacle
    double obsGpsLon;
    double obsGpsLat;
    for (int i = 0; i < static_cast<signed>(obstacle_coords.lon.size()); ++i) {
        if(unit==1) {
            // transformation in radians
            obsGpsLon = Utility::degreeToRadian(obstacle_coords.lon[i]);
            obsGpsLat = Utility::degreeToRadian(obstacle_coords.lat[i]);
        }
        else{
            obsGpsLon = obstacle_coords.lon[i];
            obsGpsLat = obstacle_coords.lat[i];
        }
        bool isCreated = createObstacleDataCircle(obsGpsLon,
                                                  obsGpsLat,
                                                  obstacleRadius,
                                                  obsData);
        if(isCreated){
            obstacles.push_back(obsData);
        }
    }

    return obstacles;
}
bool CollisionAvoidanceNode::createObstacleDataCircle(double obsGpsLon, //rads
                                                      double obsGpsLat, //rads
                                                      double obstacleRadius, //meters
                                                      ObstacleData & obstacle){
//    Logger::info("(Collision Avoidance) Obstacle data creation");
    // Conversion
    const double gpsLat = m_sensorOutput.gpsPos(1); //rad
    const double gpsLon = m_sensorOutput.gpsPos(0); //rad
    const double compassHeading = m_sensorOutput.compHeading;
//    Logger::info("(Collision Avoidance) inputs ox : %f, oy : %f, bx : %f, by : %f", obsGpsLon, obsGpsLat, gpsLon, gpsLat);

    // Simulation
    // TODO : implement more precisely obstacle simulation.
    const double obstacleHeadingRelativeToBoat = Utility::wrapToPi(atan2(obsGpsLat-gpsLat,
                                                                         obsGpsLon-gpsLon),
                                                                   - compassHeading);
//    std::cout << "compassHeading = " << compassHeading << " rads\n";
//    std::cout << "angle boat to obstacle = " << atan2(obsGpsLat-gpsLat, obsGpsLon-gpsLon) << "\n";
//    std::cout << "obstacleHeadingRelativeToBoat = " << obstacleHeadingRelativeToBoat <<","<<SENSOR_ARC_ANGLE<< "\n";
    const double distFromObstacle = Utility::calculateGPSDistance(gpsLon   ,gpsLat,
                                                                  obsGpsLon,obsGpsLat);
//      const double distFromObstacle = calculateGPSDistance(Eigen::Vector2d(obsGpsLon,obsGpsLat),m_sensorOutput.gpsPos);

//    Logger::info("(Collision Avoidance) distance from obstacle : %f", distFromObstacle);
//    Logger::info("(Collision Avoidance) obstacle heading relative to boat  : %f", obstacleHeadingRelativeToBoat);
    if(std::abs(obstacleHeadingRelativeToBoat) <= SENSOR_ARC_ANGLE && distFromObstacle<=MAXIMUM_SENSOR_RANGE) {
//        Logger::info("(Collision Avoidance) Obstacle detected");
        const double leftHeadingRelativeToBoat =  Utility::wrapToPi(atan2(obstacleRadius, distFromObstacle),
                                                                    obstacleHeadingRelativeToBoat);
        const double rightHeadingRelativeToBoat = Utility::wrapToPi(-atan2(obstacleRadius, distFromObstacle),
                                                                    obstacleHeadingRelativeToBoat);
        double minDistanceToObstacle = distFromObstacle - obstacleRadius;
        if(minDistanceToObstacle<=0){
            minDistanceToObstacle = 0.1;
        }
        obstacle = {
                minDistanceToObstacle,             //double minDistanceToObstacle;
                MAXIMUM_SENSOR_RANGE,                 //double maxDistanceToObstacle;
                leftHeadingRelativeToBoat,         //double LeftBoundheadingRelativeToBoat;
                rightHeadingRelativeToBoat,        //double RightBoundheadingRelativeToBoat;
                0,0,
                Utility::radianToDegree(m_sensorOutput.gpsPos(0)),
                Utility::radianToDegree(m_sensorOutput.gpsPos(1)),
                Utility::radianToDegree(M_PI/2-m_sensorOutput.compHeading)};
        //std::cout << "I push back\n";
        return true;
    }
    return false;

}

//Draw
void CollisionAvoidanceNode::drawState(){
    if(std::fmod(m_loop_id,10.0) == 0) {
        vibes::axisLimits(-400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD,
                          400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD,
                          -400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD,
                          400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD);

        const double pathHeading = atan2(m_followedLine.endPoint(1)-m_followedLine.startPoint(1),
                                         m_followedLine.endPoint(0)-m_followedLine.startPoint(0));;
        const double x1 = MAXIMUM_SENSOR_RANGE*CONVERSION_FACTOR_METER_TO_GPS*cos(-atan2(CHANNEL_RADIUS,MAXIMUM_SENSOR_RANGE));
        const double y1 = MAXIMUM_SENSOR_RANGE*CONVERSION_FACTOR_METER_TO_GPS*sin(-atan2(CHANNEL_RADIUS,MAXIMUM_SENSOR_RANGE));
        const double xR = x1*cos(pathHeading) - y1*sin(pathHeading) + m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD;
        const double yR = x1*cos(pathHeading-M_PI/2) - y1*sin(pathHeading-M_PI/2)+ m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD;
        const double x2 = MAXIMUM_SENSOR_RANGE*CONVERSION_FACTOR_METER_TO_GPS*cos(atan2(CHANNEL_RADIUS,MAXIMUM_SENSOR_RANGE));
        const double y2 = MAXIMUM_SENSOR_RANGE*CONVERSION_FACTOR_METER_TO_GPS*sin(atan2(CHANNEL_RADIUS,MAXIMUM_SENSOR_RANGE));
        const double xL = x2*cos(pathHeading) - y2*sin(pathHeading) + m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD;
        const double yL = x2*cos(pathHeading-M_PI/2) - y2*sin(pathHeading-M_PI/2) + m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD;
        vibes::drawLine({m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD,xR},{m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD,yR},"r");
        vibes::drawLine({m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD,xL},{m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD,yL},"b");

        drawChannel(m_followedLine);
        drawBoat(m_sensorOutput, "[m]");
//        Logger::info("(Drawing) followedLine (%f,%f) deg",
//                     m_followedLine.startPoint(0) - DRAWING_ORIGIN_LON_RAD,
//                     m_followedLine.startPoint(1) - DRAWING_ORIGIN_LAT_RAD);
//        Logger::info("(Drawing) followedLine (%f,%f) deg",
//                     m_followedLine.endPoint(0) - DRAWING_ORIGIN_LON_RAD,
//                     m_followedLine.endPoint(1) - DRAWING_ORIGIN_LAT_RAD);

        vibes::drawPoint(m_followedLine.startPoint(0) - DRAWING_ORIGIN_LON_RAD,
                         m_followedLine.startPoint(1) - DRAWING_ORIGIN_LAT_RAD,"[r]");
        vibes::drawPoint(m_followedLine.endPoint(0) - DRAWING_ORIGIN_LON_RAD,
                         m_followedLine.endPoint(1) - DRAWING_ORIGIN_LAT_RAD,"[b]");
        for (int j = 0; j < static_cast<signed>(m_obs_coords.lon.size()); j++) {
            vibes::drawCircle(m_obs_coords.lon[j]/180*M_PI - DRAWING_ORIGIN_LON_RAD,
                              m_obs_coords.lat[j]/180*M_PI - DRAWING_ORIGIN_LAT_RAD,
                              5*CONVERSION_FACTOR_METER_TO_GPS);
        }
    }
}
std::vector<double> CollisionAvoidanceNode::getEigenVectorLine(std::vector<Eigen::Vector2d> vec,int line){
    std::vector<double> output;
    for(auto & elmt : vec){
        output.push_back(elmt(line));
    }
    return output;
}
void CollisionAvoidanceNode::drawBoat(SensorData & sensorData,std::string color){
    vibes::drawVehicle(sensorData.gpsPos(0)-DRAWING_ORIGIN_LON_RAD,
                       sensorData.gpsPos(1)-DRAWING_ORIGIN_LAT_RAD,
                       sensorData.compHeading/M_PI*180,
                       4*1000*CONVERSION_FACTOR_METER_TO_GPS,
                       "[b]"); // in mm
}
void CollisionAvoidanceNode::drawChannel(FollowedLine & followedLine) {
    vibes::drawPoint(followedLine.startPoint(0)-DRAWING_ORIGIN_LON_RAD,
                     followedLine.startPoint(1)-DRAWING_ORIGIN_LAT_RAD,
                     2);
    vibes::drawPoint(followedLine.endPoint(0)-DRAWING_ORIGIN_LON_RAD,
                     followedLine.endPoint(1)-DRAWING_ORIGIN_LAT_RAD,
                     2);
    vibes::drawLine({followedLine.startPoint(0)-DRAWING_ORIGIN_LON_RAD,followedLine.endPoint(0)-DRAWING_ORIGIN_LON_RAD},
                    {followedLine.startPoint(1)-DRAWING_ORIGIN_LAT_RAD,followedLine.endPoint(1)-DRAWING_ORIGIN_LAT_RAD},
                    "g");
    double lineAngle = atan2(followedLine.endPoint(1)-followedLine.startPoint(1),
                             followedLine.endPoint(0)-followedLine.startPoint(0));
    vibes::drawLine( {followedLine.startPoint(0)
                      - CHANNEL_RADIUS * CONVERSION_FACTOR_METER_TO_GPS
                                       * sin(lineAngle)
                      - DRAWING_ORIGIN_LON_RAD,
                      followedLine.endPoint(0)
                      - CHANNEL_RADIUS * CONVERSION_FACTOR_METER_TO_GPS
                                       * sin(lineAngle)
                      - DRAWING_ORIGIN_LON_RAD},
                     {followedLine.startPoint(1)
                      + CHANNEL_RADIUS * CONVERSION_FACTOR_METER_TO_GPS
                                       * cos(lineAngle)
                      - DRAWING_ORIGIN_LAT_RAD,
                      followedLine.endPoint(1)
                      + CHANNEL_RADIUS * CONVERSION_FACTOR_METER_TO_GPS
                                       * cos(lineAngle)
                      - DRAWING_ORIGIN_LAT_RAD} );
    vibes::drawLine( {followedLine.startPoint(0)
                     + CHANNEL_RADIUS * CONVERSION_FACTOR_METER_TO_GPS
                                      * sin(lineAngle)
                     - DRAWING_ORIGIN_LON_RAD,
                     followedLine.endPoint(0)
                     + CHANNEL_RADIUS * CONVERSION_FACTOR_METER_TO_GPS
                                      * sin(lineAngle)
                     - DRAWING_ORIGIN_LON_RAD},
                    {followedLine.startPoint(1)
                     - CHANNEL_RADIUS * CONVERSION_FACTOR_METER_TO_GPS
                                      * cos(lineAngle)
                     - DRAWING_ORIGIN_LAT_RAD,
                     followedLine.endPoint(1)
                     - CHANNEL_RADIUS * CONVERSION_FACTOR_METER_TO_GPS
                                      * cos(lineAngle)
                     - DRAWING_ORIGIN_LAT_RAD} );
}
void CollisionAvoidanceNode::vibesFigureHandler(std::string name, int option){
    if(option == STANDALONE_DRAW_NEW_FIGURE){
        vibes::newFigure(name);
        vibes::selectFigure(name);
        vibes::axisLimits(-400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD,
                          400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD,
                          -400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD,
                          400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD,
                          name);
//        vibes::axisAuto(name);
        vibes::setFigureProperty("x", 1380);
        vibes::setFigureProperty("y", 715);
        vibes::setFigureProperty("width", 400);
        vibes::setFigureProperty("height", 340);
    }
    else if(option == STANDALONE_DRAW_NEW_FIGURE_11) {
        vibes::newFigure(name);
        vibes::selectFigure(name);
        vibes::axisLimits(-400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD,
                          400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD,
                          -400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD,
                          400 * CONVERSION_FACTOR_METER_TO_GPS + m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD,
                          name);
//        vibes::axisAuto(name);
        vibes::setFigureProperty("x", 1380);
        vibes::setFigureProperty("y", 375);
        vibes::setFigureProperty("width", 400);
        vibes::setFigureProperty("height", 340);
    }
    else if(option == STANDALONE_DRAW_NEW_FIGURE_UNIT_TEST) {
        vibes::newFigure(name);
        vibes::selectFigure(name);
        vibes::axisLimits(-200 * CONVERSION_FACTOR_METER_TO_GPS,
                          200 * CONVERSION_FACTOR_METER_TO_GPS,
                          -200 * CONVERSION_FACTOR_METER_TO_GPS,
                          200 * CONVERSION_FACTOR_METER_TO_GPS,
                          name);
//        vibes::axisAuto(name);
        vibes::setFigureProperty("x", 1080);
        vibes::setFigureProperty("y", 370);
        vibes::setFigureProperty("width", 800);
        vibes::setFigureProperty("height", 640);
    }
    else{
        vibes::selectFigure(name);
    }
}

