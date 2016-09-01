/****************************************************************************************
 *
 * File:
 * 		CollisionAvoidanceNode.cpp
 *
 * Purpose:
 *		Compute the new path in case of collision avoidance.
 *
 * Developer Notes:
 *      INPUT
 *      Current_line (WP1 and WP2)
 *      Boat_state
 *          GPS position
 *          roll, pitch, yaw
 *      speed
 *      Sensors_output
 *      sailing_zone (gps points)
 *      wind direction and strength
 *      obstacles
 *
 *      IDEA
 *      To use dynamic list of struct for obstacles and other variables such as this one
 *
 *      I WANT TO TRACK
 *      boat position and heading
 *      The obstacles with everything
 ***************************************************************************************/

#include "CollisionAvoidanceNode.h"
#include "SystemServices/Logger.h"

#define DRAW_STATE_WITH_VIBES 1
#define WAIT_FOR_X_WAYPOINTS 1

//CONSTRUCTOR
CollisionAvoidanceNode::CollisionAvoidanceNode(MessageBus& msgBus)
        : Node(NodeID::CollisionAvoidance, msgBus){
    msgBus.registerNode(*this, MessageType::ObstacleVector);
    msgBus.registerNode(*this, MessageType::VesselState);
    msgBus.registerNode(*this, MessageType::WaypointData);
}

//PUBLIC FUNCTIONS

bool CollisionAvoidanceNode::setSailingZone() {

    //SailingZone initialization clockwise (x,y)
    Eigen::Vector2d GPSpoint0(19.92088437080383/180*M_PI, 60.10851861384137/180*M_PI); Eigen::Vector2d GPSpoint5( 19.92339491844177/180*M_PI, 60.1082084973807/180*M_PI);
    Eigen::Vector2d GPSpoint1(19.91968274116516/180*M_PI, 60.10640120904281/180*M_PI); Eigen::Vector2d GPSpoint4( 19.92333054542541/180*M_PI, 60.1054921803023/180*M_PI);
    Eigen::Vector2d GPSpoint2(19.91946816444397/180*M_PI, 60.10273285707368/180*M_PI); Eigen::Vector2d GPSpoint3( 19.92191433906555/180*M_PI, 60.1028077254831/180*M_PI);
    m_sailingZone = {GPSpoint0, GPSpoint1, GPSpoint2, GPSpoint3, GPSpoint4, GPSpoint5};
//    Eigen::Vector2d GPSpoint0(-50*CONVERSION_FACTOR_METER_TO_GPS, 50*CONVERSION_FACTOR_METER_TO_GPS); Eigen::Vector2d GPSpoint3(50*CONVERSION_FACTOR_METER_TO_GPS,50*CONVERSION_FACTOR_METER_TO_GPS);
//    Eigen::Vector2d GPSpoint1(-50*CONVERSION_FACTOR_METER_TO_GPS,-50*CONVERSION_FACTOR_METER_TO_GPS); Eigen::Vector2d GPSpoint2(50*CONVERSION_FACTOR_METER_TO_GPS,-50*CONVERSION_FACTOR_METER_TO_GPS);
//    m_sailingZone = {GPSpoint0, GPSpoint1, GPSpoint2, GPSpoint3};
    return true;
}

bool CollisionAvoidanceNode::init(){
    setSailingZone();
    m_tack = false;
    m_tackingDirection = 0;
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
//        Logger::info("(Collision Avoidance) Received ObstacleData >>> processObstacleData()");
        m_sensorOutput.detectedObstacles = msg->obstacles();
        //std::copy(msg->obstacles().begin(),msg->obstacles().end(),std::back_inserter(m_sensorOutput.detectedObstacles));
//        Logger::info("(Collision Avoidance) Detected obstacle size : %d",m_sensorOutput.detectedObstacles.size());
        for (auto &&obstacle : m_sensorOutput.detectedObstacles) {
            obstacle.boatLonAtDetection = Utility::degreeToRadian(obstacle.boatLonAtDetection);
            obstacle.boatLatAtDetection = Utility::degreeToRadian(obstacle.boatLatAtDetection);
            obstacle.boatHeadingAtDetection = M_PI/2-Utility::degreeToRadian(obstacle.boatHeadingAtDetection);
        }
        for (auto &&item : m_sensorOutput.detectedObstacles) {
//            Logger::info("(Collision Avoidance) ==Detected Obstacle==");
//            Logger::info("(Collision Avoidance) item.LeftBoundheadingRelativeToBoat = %f",item.LeftBoundheadingRelativeToBoat);
//            Logger::info("(Collision Avoidance) item.RightBoundheadingRelativeToBoat = %f",item.RightBoundheadingRelativeToBoat);
//            Logger::info("(Collision Avoidance) item.minDistanceToObstacle = %f",item.minDistanceToObstacle);
//            Logger::info("(Collision Avoidance) item.maxDistanceToObstacle = %f",item.maxDistanceToObstacle);
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
//    Logger::info("(Collision Avoidance) Current gps position : (%f deg,%f deg)", m_sensorOutput.gpsPos(0)/M_PI*180,m_sensorOutput.gpsPos(1)/M_PI*180);
//    Logger::info("(Collision Avoidance) Received followedLine start point : (%f deg,%f deg)", m_followedLine.startPoint(0)/M_PI*180,m_followedLine.startPoint(1)/M_PI*180);
//    Logger::info("(Collision Avoidance) Received followedLine end point : (%f deg,%f deg)", m_followedLine.endPoint(0)/M_PI*180,m_followedLine.endPoint(1)/M_PI*180);
}

void CollisionAvoidanceNode::run() {
    //Note on simulation
    /*
     * For now i don't know of any place in the code where it specified if the code is
     * in a simulated environement or not. So this variable is temporary.
     */
    Logger::info("(Collision Avoidance) >>> run()");
    check_obstacles(m_sensorOutput,m_seenObstacles); // seen_obstacle as a reference

    double minDebug = 10000000000.0;
//    Logger::info("(Collision Avoidance) !m_seenObstacles.empty() = %d",!m_seenObstacles.empty());
    //To prevent strange behaviour from m_seenObstacles
    if(!m_seenObstacles.empty()){

        for (auto &&seenObstacle : m_seenObstacles) {
            double dist = calculateGPSDistance(seenObstacle.center,m_sensorOutput.gpsPos);
            minDebug = std::min(minDebug,dist);
        }
//        Logger::info("(Collision Avoidance) Min distance to obstacle : %f m", minDebug);
//        Logger::info("(Collision Avoidance) Size of Obstacle Vector: %d", m_seenObstacles.size());

//    Logger::info("(Collision Avoidance) Current latitude : %f deg",m_sensorOutput.gpsPos(1)/M_PI*180);
//    Logger::info("(Collision Avoidance) Current longitude : %f deg",m_sensorOutput.gpsPos(0)/M_PI*180);
//    Logger::info("(Collision Avoidance) First obstacle latitude : %f deg",m_seenObstacles[0].center/M_PI*180);
//    Logger::info("(Collision Avoidance) First obstacle longitude : %f deg",m_seenObstacles[0].center/M_PI*180);
//    Logger::info("(Collision Avoidance) distance to first obstacle : %f m", calculateGPSDistance(m_seenObstacles[0].center,
//                                                                                                 m_sensorOutput.gpsPos));

        //    update_map();
        if (these_obstacles_are_a_problem(m_seenObstacles)) {
//            Logger::info("(Collision Avoidance) >>> Obstacle is a problem");
            PotentialMap potential_field = compute_potential_field(m_seenObstacles,
                                                                   m_sailingZone,
                                                                   m_followedLine);
            Eigen::Vector2d min = find_minimum_potential_field(potential_field);
            WaypointOutput wpOut = compute_new_path(min,m_followedLine);
//            Logger::info("(Collision Avoidance) New path computed >>> send new path");

            MessagePtr msg = std::make_unique<CollisionAvoidanceMsg>(Utility::radianToDegree(wpOut.startPoint(0)),
                                                                     Utility::radianToDegree(wpOut.startPoint(1)),
                                                                     Utility::radianToDegree(wpOut.midPoint(0)),
                                                                     Utility::radianToDegree(wpOut.midPoint(1)),
                                                                     Utility::radianToDegree(wpOut.endPoint(0)),
                                                                     Utility::radianToDegree(wpOut.endPoint(1)));
            m_MsgBus.sendMessage(std::move(msg));
//            Logger::info("(Collision Avoidance) Sent collision avoidance message");
        }
    } else{
        m_seenObstacles.clear();
//        Logger::info("(Collision Avoidance) No obstacles seen");
    }
//    Logger::info("(Collision Avoidance) Current followedLine start point : (%f deg,%f deg)", m_followedLine.startPoint(0)/M_PI*180,m_followedLine.startPoint(1)/M_PI*180);
//    Logger::info("(Collision Avoidance) Current followedLine end point : (%f deg,%f deg)", m_followedLine.endPoint(0)/M_PI*180,m_followedLine.endPoint(1)/M_PI*180);
}

/*
 * Update waypoints or compute an easier way to handle them.
 * Update m_followedLine ?
 */
//    void update_waypoints(){}

void CollisionAvoidanceNode::check_obstacles(
        SensorData & sensorData,
        std::vector<Obstacle> & seenObstacles) {

    // clean the obstacles thar should have been detected and are not
    cleanObstacles(sensorData,seenObstacles); // seen_obstacle as a reference

    // register the new obstacles inside upToDateObstacles (convert from sensorData)
    std::vector<Obstacle> upToDateObstacles = registerObstacles(sensorData, seenObstacles); // seen_obstacle not as a reference

//    standAloneDrawObstacles(upToDateObstacles,"[r]","upToDateObstacles",STANDALONE_DRAW_NEW_FIGURE_11);

    // for each up to date obstacle we clean the areas where nothing has been detected
    mergeObstacles(sensorData,upToDateObstacles,seenObstacles); // seen_obstacle as a reference

    //TODO :   what():  Boost.Geometry Overlay invalid input exception when running simulation. Obstacles are all destroyed then recreated at some point of the simulation.
    //TODO : Obstacles are not updated properly on running the simulation
}

//void CollisionAvoidanceNode::check_obstaclesBearingOnly(
//        SensorData sensorData,
//        BearingOnlyVars & vars) {
//
//    // clean the obstacles thar should have been detected and are not
//    cleanObstaclesBO(sensorData,seenObstacles); // seen_obstacle as a reference
//
//    // register the new obstacles inside upToDateObstacles (convert from sensorData)
//    std::vector<Obstacle> upToDateObstacles = registerObstaclesBO(sensorData, seenObstacles); // seen_obstacle not as a reference
//
//    // for each up to date obstacle we clean the areas where nothing has been detected
//    mergeObstaclesBO(sensorData,upToDateObstacles,seenObstacles); // seen_obstacle as a reference
//}

/*
 *
 */
//map update_map(){}

bool CollisionAvoidanceNode::these_obstacles_are_a_problem(
        std::vector<Obstacle> & seenObstacles) { // OUTPUT if these obstacles are a problem
    Logger::info("(Collision Avoidance) >>> These_obstacles_are_a_problem ");

    bool theseObstaclesAreAProblem = false;
    // We have polygons. We need to see if there is an intersection between
    // the channel and the polygons. That is to say : one of the points of the
    // polygon is inside the channel or it's center

    for(auto & obstacle : seenObstacles){
        for(auto & point : obstacle.polygon){
            // compute the distance between the line and the point.
            const std::vector<double> distanceInfo = distanceFromSegment(m_followedLine.startPoint,
                                                                         m_followedLine.endPoint,
                                                                         point);
            const double distance = distanceInfo[0];
//            std::cout << "_distance to line segment = " << distance << "\n";

            // compute the distance between the boat and the closest point of the obstacle
            // TODO : use boost poly for get closest point ?
            const double distObstacleFromBoat = calculateGPSDistance(m_sensorOutput.gpsPos,
                                                                     getClosestPoint(obstacle.polygon,
                                                                                     m_sensorOutput.gpsPos));
//            std::cout << "_distance boat from obstacle = " << distObstacleFromBoat << "\n";

            bool thisObstacleIsAProblem = false;
//            std::cout << "__(distance < CHANNEL_RADIUS) = " << (distance < CHANNEL_RADIUS) << "\n";
//            std::cout << "__(distObstacleFromBoat < SAFE_DISTANCE) = " << (distObstacleFromBoat < SAFE_DISTANCE) << "\n";
            if(   (distance < CHANNEL_RADIUS)
                  && (distObstacleFromBoat < SAFE_DISTANCE)){

                thisObstacleIsAProblem = true;
            }
            theseObstaclesAreAProblem = theseObstaclesAreAProblem || thisObstacleIsAProblem;
        }

        // Do it another time for the center
        // TODO : put code inside these_obstacles_are_a_problem into a function
        const std::vector<double> distanceInfo = distanceFromSegment(m_followedLine.startPoint,
                                                                     m_followedLine.endPoint,
                                                                     obstacle.center);
        const double distance = distanceInfo[0];
//        std::cout << "_distance center to line segment = " << distance << "\n";

        // compute the distance between the boat and the closest point of the obstacle
        // TODO : use boost poly for get closest point ?
        const double distObstacleFromBoat = calculateGPSDistance(m_sensorOutput.gpsPos,
                                                                 getClosestPoint(obstacle.polygon,
                                                                                 m_sensorOutput.gpsPos));
//        std::cout << "_distance boat from center of obstacle = " << distObstacleFromBoat << "\n";

        bool thisObstacleIsAProblem = false;
//        std::cout << "__(distanceCenter < CHANNEL_RADIUS) = " << (distance < CHANNEL_RADIUS) << "\n";
//        std::cout << "__(distObstacleCenterFromBoat < SAFE_DISTANCE) = " << (distObstacleFromBoat < SAFE_DISTANCE) << "\n";
        if(   (distance < CHANNEL_RADIUS)
              && (distObstacleFromBoat < SAFE_DISTANCE)){

            thisObstacleIsAProblem = true;
        }
        theseObstaclesAreAProblem = theseObstaclesAreAProblem || thisObstacleIsAProblem;

    }
    return theseObstaclesAreAProblem;
}

PotentialMap CollisionAvoidanceNode::compute_potential_field(
        std::vector<Obstacle> & seenObstacles,
        std::vector<Eigen::Vector2d> & sailingZone,
        FollowedLine & followedLine) {
    // Init
    // Set up of the matrix of 100*100 around the boat.
    //TODO : get these values from the database/elsewhere ?
    const int matrixHeight   =  100; const int matrixWidth    =  100;
    const int yUpperMapBound =  100; const int xUpperMapBound =  100; // in meters
    const int yLowerMapBound = -100; const int xLowerMapBound = -100; // in meters

    // Since the distance computed here is only on one axis,
    // a simple conversion factor is sufficient
    PotentialMap PotField = {
            m_sensorOutput.gpsPos(0)
            + (xLowerMapBound * CONVERSION_FACTOR_METER_TO_GPS),
            m_sensorOutput.gpsPos(0)
            + (xUpperMapBound * CONVERSION_FACTOR_METER_TO_GPS),
            m_sensorOutput.gpsPos(1)
            + (yLowerMapBound * CONVERSION_FACTOR_METER_TO_GPS),
            m_sensorOutput.gpsPos(1)
            + (yUpperMapBound * CONVERSION_FACTOR_METER_TO_GPS),
            Eigen::ArrayXXd::Zero(matrixHeight, matrixWidth)
    };

    // On a distance of 150m there is an error of approximatively 2 meters, so a linear conversion from
    // GPS coordinates to meters is possible at this scale
    const Eigen::ArrayXXd Px = Eigen::RowVectorXd::LinSpaced(matrixWidth,
                                                             xLowerMapBound,
                                                             xUpperMapBound).replicate(matrixHeight,1);
    const Eigen::ArrayXXd Py =    Eigen::VectorXd::LinSpaced(matrixHeight,
                                                             yLowerMapBound,
                                                             yUpperMapBound).replicate(1, matrixWidth);


    // Obstacle potential function
    const Eigen::ArrayXXd obsPot = computeObstaclePot(Px,Py,m_sensorOutput,seenObstacles,followedLine);

    // Objective potential function
    const Eigen::ArrayXXd objPot = computeObjectivePot(Px,Py,followedLine);

    // Windpreference potential function
    const Eigen::ArrayXXd windPot = computeWindPot(Px,Py,m_sensorOutput);

    // Boat preference
    const Eigen::ArrayXXd boatPot = computeBoatPot(Px,Py,m_sensorOutput);

    // Sailing Zone
    const Eigen::ArrayXXd sailZonePot = computeSailingZonePot(Px,Py,sailingZone);

    // Computation of the field
    PotField.field = obsPot - objPot + boatPot + windPot + sailZonePot;
    return PotField;
}

Eigen::Vector2d CollisionAvoidanceNode::find_minimum_potential_field(
        PotentialMap & potField) {
    double I_row = 0, I_col = 0;
    potField.field.minCoeff(&I_row, &I_col);
    double I_x = I_col*(potField.xMax-potField.xMin)
                 /potField.field.cols()-(std::abs(potField.yMin));
    double I_y = I_row*(potField.yMax-potField.yMin)
                 /potField.field.rows()-(std::abs(potField.xMin));

    const Eigen::Vector2d collision_avoidance_point(I_x,I_y);

    return collision_avoidance_point;
}

WaypointOutput CollisionAvoidanceNode::compute_new_path(
        Eigen::Vector2d collision_avoidance_point,
        FollowedLine followedLine) {
    WaypointOutput wpOut;
    const double avoidDist = 30;

    // Initialize the 3 wps
    // Behind the obstacle, not sure that using the heading of the boat is the best idea
    // TODO some test on issue above (using heading of the boat in startCollPoint)
    // TODO replace compheading by a general heading
    const Eigen::Vector2d trigoVect(cos(m_sensorOutput.compHeading + M_PI),
                                    sin(m_sensorOutput.compHeading + M_PI));
    const Eigen::Vector2d startCollPoint = getPointWithDistanceAndBearing( avoidDist,
                                                                           m_sensorOutput.compHeading + M_PI,
                                                                           m_sensorOutput.gpsPos);
    // Avoid point : collision_avoidance_point
    //Front of the obstacle
    const Eigen::Vector2d endCollPoint   = getPointWithDistanceAndBearing(-avoidDist,
                                                                          m_sensorOutput.compHeading + M_PI,
                                                                          m_sensorOutput.gpsPos);

    wpOut.startPoint = startCollPoint;
    wpOut.midPoint = collision_avoidance_point;
    wpOut.endPoint = endCollPoint;

    // Set m_followedLine accordingly
//    followedLine.startPoint = startCollPoint;
//    followedLine.endPoint = collision_avoidance_point;

    return wpOut;
}

CommandOutput CollisionAvoidanceNode::compute_commands(
        FollowedLine line) {

    //controller_simpleLine Simple line following controller
    //   Controller based on the paper "A simple controller for line
    //   following of sailboats" by Luc Jaulin and Fabrice Le Bars
    // TODO Constans/Parameters
//    m = x(1:2);
//    theta = x(3);
//    v_boat = x(4);

    const double delta_rMax = M_PI / 4;     // rad   maximum rudder angle
    const double incidenceAngle = M_PI / 4; // rad   incidence angle
    const double ngzAngle = M_PI / 4;       // rad   close hauled angle
//    const double ngzAngleBack = 0;          // rad   back wind angle
    // ngzAngleOUT = ngzAngle+pi/8;         // rad   out of the no-go zone angle
    // NGZHeading = mod(psi+pi,2*pi);       // rad   no-go zone angle

    // Step 3 : compute lineHeading
    double lineHeading = atan2(line.endPoint(1) - line.startPoint(1),
                               line.endPoint(0) - line.startPoint(0));
    if (std::abs(wrapToPi(atan2(m_sensorOutput.gpsPos(1) - line.endPoint(1),
                                m_sensorOutput.gpsPos(0) - line.endPoint(0)),- lineHeading))
        < M_PI / 2){
        // Inverse the followed line if the boat has gone further the waypoint
        const Eigen::Vector2d tmpStorageVec = line.endPoint;
        line.endPoint = line.startPoint;
        line.startPoint = tmpStorageVec;

        lineHeading = atan2(line.endPoint(1) - line.startPoint(1),
                            line.endPoint(0) - line.startPoint(0));
    }

    // Step 1 : compute distance from line
    const double signedDistance = signedDistanceFromLine(line.startPoint,
                                                         line.endPoint,
                                                         m_sensorOutput.gpsPos);

    // Step 2 : set up tacking direction -1: ,1:
    if (std::abs(signedDistance) > CHANNEL_RADIUS){
        m_tackingDirection = Utility::sgn(signedDistance); // matlab : q
        // OR q = sign(theta-(mod(psi,pi)-pi)); (OLD algo)
    }

    // Step 4 : compute desired heading
    //take care of the incidence angle
    double desiredHeading_star = lineHeading - 2 * incidenceAngle // theta_star
                                               / M_PI*atan(signedDistance
                                                           / CHANNEL_RADIUS);
    double desiredHeading_bar;

    // Step 5-9 : tacking part
    if (    ((cos(m_sensorOutput.windDirection - desiredHeading_star)
              + cos(ngzAngle+M_PI/8) < 0)
             && (m_tack)
            )

            || // leaving the NGZ
            (cos(m_sensorOutput.windDirection - desiredHeading_star)
             + cos(ngzAngle) < 0)

            || // entering the NGZ
            (cos(m_sensorOutput.windDirection  - lineHeading)
             + cos(ngzAngle) < 0)
            ){

        if(!m_tack) {
            m_tackingDirection = Utility::sgn(m_sensorOutput.compHeading
                                              - (fmod(m_sensorOutput.windDirection, M_PI) - M_PI));
            m_tack = true;
        }
        desiredHeading_bar = M_PI + m_sensorOutput.windDirection
                             - m_tackingDirection * ngzAngle;
        // Backwind nogozone (WIP)
//        if(
//                (cos(m_sensorOutput.windDirection + M_PI - desiredHeading_star)
//                 + cos(ngzAngleBack) < 0)
//                ||
//                ( (std::abs(signedDistance)< CHANNEL_RADIUS)
//                  && (cos(m_sensorOutput.windDirection + M_PI-lineHeading)
//                      + cos(ngzAngleBack) < 0) )
//        ){
//
//            if (!m_tack) {
//                m_tackingDirection = Utility::sgn(signedDistanceFromLine(line.startPoint,
//                                                                         line.endPoint,
//                                                                         m_sensorOutput.gpsPos));
//                m_tack = true;
//            }
//            desiredHeading_bar = m_sensorOutput.windDirection - m_tackingDirection * ngzAngleBack;
//        }
    }
    else{
        m_tack = 0;
        desiredHeading_bar = desiredHeading_star;
    }

    // Step 10-11 : rudder command
    CommandOutput commandOutput;
    if(cos(m_sensorOutput.compHeading - desiredHeading_bar) >= 0) {
        commandOutput.deltaRudder = delta_rMax
                                    * sin(m_sensorOutput.compHeading - desiredHeading_bar);
    }
    else {
        commandOutput.deltaRudder = delta_rMax
                                    * Utility::sgn(sin(m_sensorOutput.compHeading
                                                       - desiredHeading_bar));
    }

    // Step 12 : sail command
    commandOutput.deltaSail = M_PI/4 * (cos(m_sensorOutput.windDirection
                                            - desiredHeading_bar)
                                        + 1);
    return commandOutput;
}

// UTILITY FUNCTIONS
// TODO : move to Utility folder ?

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

    // Sensor settings
    const double maxDetectionRange = MAXIMUM_SENSOR_RANGE;

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
    if(std::abs(obstacleHeadingRelativeToBoat) <= SENSOR_ARC_ANGLE && distFromObstacle<=maxDetectionRange) {
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
                maxDetectionRange,                 //double maxDistanceToObstacle;
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
        drawEigenPoly(m_sailingZone, "b[]");
        drawChannel(m_followedLine);
        drawObstacles(m_seenObstacles, "[r]");
        drawBoat(m_sensorOutput, "[m]");
        vibes::drawPoint(m_followedLine.startPoint(0),m_followedLine.startPoint(1),"[r]");
        vibes::drawPoint(m_followedLine.endPoint(0),m_followedLine.endPoint(1),"[b]");
        for (int j = 0; j < static_cast<signed>(m_obs_coords.lon.size()); j++) {
            vibes::drawCircle(m_obs_coords.lon[j]/180*M_PI - DRAWING_ORIGIN_LON_RAD,
                              m_obs_coords.lat[j]/180*M_PI - DRAWING_ORIGIN_LAT_RAD,
                              5*CONVERSION_FACTOR_METER_TO_GPS);
        }
        vibes::drawCircle(m_sensorOutput.gpsPos(0) - DRAWING_ORIGIN_LON_RAD,
                          m_sensorOutput.gpsPos(1) - DRAWING_ORIGIN_LAT_RAD,
                          MAXIMUM_SENSOR_RANGE * CONVERSION_FACTOR_METER_TO_GPS);
    }
}
std::vector<double> CollisionAvoidanceNode::getEigenVectorLine(std::vector<Eigen::Vector2d> vec,int line){
    std::vector<double> output;
    for(auto & elmt : vec){
        output.push_back(elmt(line));
    }
    return output;
}
std::vector<double>  CollisionAvoidanceNode::getBoostVectorLine(boostPolygon vec,int line){
    std::vector<double> output;
    for(boostPoint & elmt : vec.outer()){
        if(line==0){
            output.push_back(elmt.get<0>());
        }
        else{
            output.push_back(elmt.get<1>());
        }
    }
    return output;
}
void CollisionAvoidanceNode::drawObstacles(std::vector<Obstacle> & seen_obstacles,std::string color){
    for(auto & obs : seen_obstacles) {
        drawObstacle(obs,color);
    }
}
void CollisionAvoidanceNode::drawObstacle(Obstacle obs,std::string color){
    drawEigenPoly(obs.polygon,color);
    vibes::drawPoint(obs.center(0)-DRAWING_ORIGIN_LON_RAD,
                     obs.center(1)-DRAWING_ORIGIN_LAT_RAD,
                     2);
}
void CollisionAvoidanceNode::drawEigenPoly(std::vector<Eigen::Vector2d> poly,std::string color){
    std::vector<double> xVec = getEigenVectorLine(poly, 0);
    for (auto &&item : xVec) {
        item = item - DRAWING_ORIGIN_LON_RAD;
    }
    std::vector<double> yVec = getEigenVectorLine(poly, 1);
    for (auto &&item : yVec) {
        item = item - DRAWING_ORIGIN_LAT_RAD;
    }
    vibes::drawPolygon(xVec,yVec,color);
}
void CollisionAvoidanceNode::drawBoostPoly(boostPolygon poly, std::string color){
    std::vector<double> xVec = getBoostVectorLine(poly, 0);
    for (auto &&item : xVec) {
        item = item - DRAWING_ORIGIN_LON_RAD;
    }
    std::vector<double> yVec = getBoostVectorLine(poly, 1);
    for (auto &&item : yVec) {
        item = item - DRAWING_ORIGIN_LAT_RAD;
    }
    vibes::drawPolygon(xVec,yVec,color);
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
//    vibes::drawLine( {followedLine.startPoint(0)-CHANNEL_RADIUS - DRAWING_ORIGIN_LON_RAD
//                                                 * CONVERSION_FACTOR_METER_TO_GPS
//                                                 * sin(lineAngle),
//                      followedLine.endPoint(0)  -CHANNEL_RADIUS - DRAWING_ORIGIN_LON_RAD
//                                                 * CONVERSION_FACTOR_METER_TO_GPS
//                                                 * sin(lineAngle)},
//                     {followedLine.startPoint(1)+CHANNEL_RADIUS - DRAWING_ORIGIN_LAT_RAD
//                                                 * CONVERSION_FACTOR_METER_TO_GPS
//                                                 * cos(lineAngle),
//                      followedLine.endPoint(1)  +CHANNEL_RADIUS - DRAWING_ORIGIN_LAT_RAD
//                                                 * CONVERSION_FACTOR_METER_TO_GPS
//                                                 * cos(lineAngle)} );
//    vibes::drawLine( {followedLine.startPoint(0)+CHANNEL_RADIUS - DRAWING_ORIGIN_LON_RAD
//                                                 * CONVERSION_FACTOR_METER_TO_GPS
//                                                 * sin(lineAngle),
//                      followedLine.endPoint(0)  +CHANNEL_RADIUS - DRAWING_ORIGIN_LON_RAD
//                                                 * CONVERSION_FACTOR_METER_TO_GPS
//                                                 * sin(lineAngle)},
//                     {followedLine.startPoint(1)-CHANNEL_RADIUS - DRAWING_ORIGIN_LAT_RAD
//                                                 * CONVERSION_FACTOR_METER_TO_GPS
//                                                 * cos(lineAngle),
//                      followedLine.endPoint(1)  -CHANNEL_RADIUS - DRAWING_ORIGIN_LAT_RAD
//                                                 * CONVERSION_FACTOR_METER_TO_GPS
//                                                 * cos(lineAngle)} );
    vibes::drawLine( {followedLine.startPoint(0)-CHANNEL_RADIUS
                                                 * CONVERSION_FACTOR_METER_TO_GPS
                                                 * sin(lineAngle) - DRAWING_ORIGIN_LON_RAD,
                      followedLine.endPoint(0)  -CHANNEL_RADIUS
                                                 * CONVERSION_FACTOR_METER_TO_GPS
                                                 * sin(lineAngle) - DRAWING_ORIGIN_LON_RAD},
                     {followedLine.startPoint(1)+CHANNEL_RADIUS
                                                 * CONVERSION_FACTOR_METER_TO_GPS
                                                 * cos(lineAngle) - DRAWING_ORIGIN_LAT_RAD,
                      followedLine.endPoint(1)  +CHANNEL_RADIUS
                                                 * CONVERSION_FACTOR_METER_TO_GPS
                                                 * cos(lineAngle) - DRAWING_ORIGIN_LAT_RAD} );
    vibes::drawLine( {followedLine.startPoint(0)+CHANNEL_RADIUS
                                                 * CONVERSION_FACTOR_METER_TO_GPS
                                                 * sin(lineAngle) - DRAWING_ORIGIN_LON_RAD,
                      followedLine.endPoint(0)  +CHANNEL_RADIUS
                                                 * CONVERSION_FACTOR_METER_TO_GPS
                                                 * sin(lineAngle) - DRAWING_ORIGIN_LON_RAD},
                     {followedLine.startPoint(1)-CHANNEL_RADIUS
                                                 * CONVERSION_FACTOR_METER_TO_GPS
                                                 * cos(lineAngle) - DRAWING_ORIGIN_LAT_RAD,
                      followedLine.endPoint(1)  -CHANNEL_RADIUS
                                                 * CONVERSION_FACTOR_METER_TO_GPS
                                                 * cos(lineAngle) - DRAWING_ORIGIN_LAT_RAD} );
}
void CollisionAvoidanceNode::drawPotField(PotentialMap & potfield,int option){
    const double min = potfield.field.minCoeff();
    const double max = potfield.field.maxCoeff();
    const double range = max-min; // red, orange ,yellow, green, cyan, blue
    for (int i = 0; i < potfield.field.cols(); ++i) {
        for (int j = 0; j < potfield.field.rows(); ++j) {
            if(potfield.field.coeff(j,i) < min + 1/7.0*range){
                drawPotFieldPoint(i,j,potfield,"black[black]",option);
            }
            else if(potfield.field.coeff(j,i) < min + 2/7.0*range){
                drawPotFieldPoint(i,j,potfield,"blue[blue]",option);
            }
            else if(potfield.field.coeff(j,i) < min + 3/7.0*range){
                drawPotFieldPoint(i,j,potfield,"cyan[cyan]",option);
            }
            else if(potfield.field.coeff(j,i) < min + 4/7.0*range){
                drawPotFieldPoint(i,j,potfield,"green[green]",option);
            }
            else if(potfield.field.coeff(j,i) < min + 5/7.0*range){
                drawPotFieldPoint(i,j,potfield,"yellow[yellow]",option);
            }
            else if(potfield.field.coeff(j,i) < min + 6/7.0*range){
                drawPotFieldPoint(i,j,potfield,"orange[orange]",option);
            }
            else if(potfield.field.coeff(j,i) < min + 7/7.0*range){
                drawPotFieldPoint(i,j,potfield,"red[red]",option);
            }
        }
    }

}
void CollisionAvoidanceNode::drawPotFieldPoint(int i, int j, PotentialMap & potfield, std::string color, int option){
    // Option : {0:point,1:square}
    if(option==1) {
        const double stepX = (potfield.xMax - potfield.xMin)/potfield.field.cols();
        const double stepY = (potfield.yMax - potfield.yMin)/potfield.field.rows();
        vibes::drawBox(static_cast<double>(i) / potfield.field.cols()
                       * (potfield.xMax - potfield.xMin)
                       + potfield.xMin - stepX/2 - DRAWING_ORIGIN_LON_RAD,
                       static_cast<double>(i) / potfield.field.cols()
                       * (potfield.xMax - potfield.xMin)
                       + potfield.xMin + stepX/2 - DRAWING_ORIGIN_LON_RAD,
                       static_cast<double>(j) / potfield.field.rows()
                       * (potfield.yMax - potfield.yMin)
                       + potfield.yMin - stepY/2 - DRAWING_ORIGIN_LAT_RAD,
                       static_cast<double>(j) / potfield.field.rows()
                       * (potfield.yMax - potfield.yMin)
                       + potfield.yMin + stepY/2 - DRAWING_ORIGIN_LAT_RAD,
                       color);
    }
    else{
        vibes::drawPoint(static_cast<double>(i) / potfield.field.cols() * (potfield.xMax - potfield.xMin)
                         + potfield.xMin - DRAWING_ORIGIN_LON_RAD,
                         static_cast<double>(j) / potfield.field.rows() * (potfield.yMax - potfield.yMin)
                         + potfield.yMin - DRAWING_ORIGIN_LAT_RAD,
                         2, color);
    }
}
void CollisionAvoidanceNode::standAloneDrawObstacles(std::vector<Obstacle> seen_obstacles,std::string color, std::string name, int option){
    vibesFigureHandler(name,option);
    for(auto & obs : seen_obstacles) {
        drawObstacle(obs,color);
    }
}
void CollisionAvoidanceNode::standAloneDrawBoostMultiPoly(boostMultiPolygon multiPoly, std::string color, std::string name, int option){
    vibesFigureHandler(name,option);
    for(auto & poly : multiPoly) {
        drawBoostPoly(poly,color);
    }
}
void CollisionAvoidanceNode::standAloneDrawBoostPoly(boostPolygon poly, std::string color, std::string name, int option){
    vibesFigureHandler(name,option);
    drawBoostPoly(poly,color);

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

double CollisionAvoidanceNode::wrapToPi(
        double radAngle1,
        double radAngle2) {
    return fmod(radAngle1+radAngle2+3*M_PI,2*M_PI)-M_PI;
}

// TODO use function in utility/courseMath.h ?
double CollisionAvoidanceNode::calculateGPSDistance(
        Eigen::Vector2d point1,
        Eigen::Vector2d point2) {
    //...(1) : latitude //...(0) : longitude
    const double deltaLat = wrapToPi(point2(1),- point1(1));
    const double deltaLon = wrapToPi(point2(0),- point1(0));
    const double a  = sin(deltaLat/2)*sin(deltaLat/2)
                      + cos(point1(1))*cos(point2(1))*sin(deltaLon/2)*sin(deltaLon/2);
    const double c = 2*atan2(sqrt(a),sqrt(1-a));
    return EARTH_RADIUS*c;
}

Eigen::Vector2d CollisionAvoidanceNode::findMidPoint(
        Eigen::Vector2d pt1,
        Eigen::Vector2d pt2){
    const Eigen::Vector3d cartPt1 = latLonToCartesian(pt1);
    const Eigen::Vector3d cartPt2 = latLonToCartesian(pt2);
    const Eigen::Vector3d cartPt3 = (cartPt1+cartPt2)/2;
    return cartesianToLatLon(cartPt3);
}

Eigen::Vector2d CollisionAvoidanceNode::findCenter(
        const std::vector<Eigen::Vector2d> polygon,
        int option) {
    if(option==1){
        const Eigen::Vector2d midPoint = findMidPoint(polygon[0],polygon[3]);
        const double distance = calculateGPSDistance(polygon[0],polygon[3])/2;
        const double bearing = wrapToPi(atan2(polygon[3](1)-polygon[0](1),
                                              polygon[3](0)-polygon[0](0)),
                                        - M_PI/2);
        const Eigen::Vector2d closeCenter = getPointWithDistanceAndBearing(distance,bearing,midPoint);
        return closeCenter;
    }
    else {
        // Not totally accurate but ok for the precision we need.
        double sumOfX = 0;
        double sumOfY = 0;
        for (auto &vec : polygon) {
            sumOfX += vec(0);
            sumOfY += vec(1);
        }
        const Eigen::Vector2d meanPolygon(sumOfX / polygon.size(), sumOfY / polygon.size());
        return meanPolygon;
    }
}

// TODO use function in utility/courseMath.h ?
double CollisionAvoidanceNode::getInitialBearing(
        Eigen::Vector2d pt1,
        Eigen::Vector2d pt2) {
    const double bearingFromNorth = atan2(sin(pt2(0)-pt1(0))*cos(pt2(1)),
                                          cos(pt1(1))*sin(pt2(1))-sin(pt1(1))*cos(pt2(1))
                                                                  *cos(pt2(0)-pt1(0)));
    return M_PI/2-bearingFromNorth;
}

double CollisionAvoidanceNode::getArea(
        std::vector<Eigen::Vector2d> polygon){
    double sumAngle = 0;
    for(int i = 0;i< static_cast<signed>(polygon.size());i++){
        const int iPlusOne = static_cast<int>((i + 1) % polygon.size());
        const int iMinusOne = static_cast<int>((i - 1) % polygon.size());
        const double angleCurrentVertex = wrapToPi(getInitialBearing(polygon[i],
                                                                     polygon[iMinusOne]),
                                                   - getInitialBearing(polygon[i],
                                                                       polygon[iPlusOne]));
        //std::cout << i << " : " << angleCurrentVertex << "\n";
        sumAngle += angleCurrentVertex;
    }
    //std::cout << sumAngle << "\n";
    return (sumAngle-(polygon.size()-2)*M_PI)*EARTH_RADIUS*EARTH_RADIUS;
}

double CollisionAvoidanceNode::signedDistanceFromLine(
        Eigen::Vector2d linePt1,
        Eigen::Vector2d linePt2,
        Eigen::Vector2d point){

    //Lat Lon to Cartesian coordinates
    const Eigen::Vector3d a = latLonToCartesian(linePt1);
    const Eigen::Vector3d b = latLonToCartesian(linePt2);
    const Eigen::Vector3d m = latLonToCartesian(point);

    const Eigen::Vector3d n = (a.cross(b)) / (a.cross(b).norm());
    const double distFromTriangle = m.transpose() * n;
    const double arcLenght = EARTH_RADIUS * asin(distFromTriangle / EARTH_RADIUS);
    return arcLenght;
}

Eigen::Vector2d CollisionAvoidanceNode::getClosestVertex(
        std::vector<Eigen::Vector2d> polygon,
        Eigen::Vector2d point){
    int idClosestPoint = -1;
    double minDistance = 40000000;
    double distance;
    for(int i = 0;i<static_cast<signed>(polygon.size());i++) {
        distance = calculateGPSDistance(polygon[i], point);
        minDistance = std::min(minDistance, distance);
        if (minDistance == distance) {
            idClosestPoint = i;
        }
    }
    return polygon[idClosestPoint];
}

Eigen::Vector2d CollisionAvoidanceNode::getClosestPoint(
        std::vector<Eigen::Vector2d> polygon,
        Eigen::Vector2d point){

    bool computeMidPoint; // Used to lighten the computations.
    int closestPointId0;
    int closestPointId1;
    double min = 40000000; // 40M should be sufficient (~>perimeter of Earth)
    Eigen::Vector2d closestPoint;
    const Eigen::Vector3d worldOrigin(0,0,0);

    for(int i = 0;i<static_cast<signed>(polygon.size());i++){
        const int iPlusOne = static_cast<const int>((i+1) % polygon.size());
        const std::vector<double> distInfo = distanceFromSegment(polygon[i],
                                                                 polygon[iPlusOne],
                                                                 point);
        min = std::min(min,distInfo[0]);

        if(min==distInfo[0] && distInfo[1]==2){
            computeMidPoint = true;
            closestPointId0 = i;
            closestPointId1 = iPlusOne;
        }
        else if(min==distInfo[0] && distInfo[1]==0){
            closestPoint = polygon[i];
            computeMidPoint = false;
        }
        else if(min==distInfo[0] && distInfo[1]==1){
            closestPoint = polygon[i];
            computeMidPoint = false;
        }
    }

    if(computeMidPoint){
        //get the projection in 3D on the plan created by the center of Earth and the segment
        //get its gps coordinates by cartesian to latLon
        //3D vector necessary for calculations.
        const Eigen::Vector3d segPt0 = latLonToCartesian(polygon[closestPointId0]);
        const Eigen::Vector3d segPt1 = latLonToCartesian(polygon[closestPointId1]);
        const Eigen::Vector3d pointCartesian = latLonToCartesian(point);

        //normal unit vector to the plane
        const Eigen::Vector3d planeNormal = ((segPt0-worldOrigin).cross(segPt1-worldOrigin))
                                            /((segPt0-worldOrigin).cross(segPt1-worldOrigin).norm());
        const Eigen::Vector3d projection = ((-pointCartesian.dot(planeNormal)) * planeNormal)
                                           + pointCartesian;

        //std::cout << "projection : \n" << projection << "\n";

        closestPoint = cartesianToLatLon(projection);
        // Since altitude isn't used in the calculations, this is the right
        // latitude and longitude
    }
    return closestPoint;
}

Eigen::Vector2d CollisionAvoidanceNode::getPointWithDistanceAndBearing(
        double distance, // in meters
        double bearing,  // in radians from east
        Eigen::Vector2d startPoint) {
    // lat in rad, long in rad. Everything from east anticlockwise
    const double angularDist = distance/EARTH_RADIUS;
    // Since the bearing is here in radians from east, rightAngleForFormula = pi/2 - bearing
    // so cos are transformed in sin, and inversely the same for sin, for the bearing.
    const double newLat = asin(sin(startPoint(1))*cos(angularDist)
                               +cos(startPoint(1))*sin(angularDist)*sin(bearing));
    const double newLong = (startPoint(0) + atan2(cos(bearing)*sin(angularDist)*cos(startPoint(1)),
                                                 cos(angularDist) - sin(startPoint(1))
                                                                    *sin(newLat)));
    const Eigen::Vector2d endPoint(newLong,newLat);

    return endPoint;
}

std::vector<double> CollisionAvoidanceNode::distanceFromSegment(
        Eigen::Vector2d segmentPt1,
        Eigen::Vector2d segmentPt2,
        Eigen::Vector2d point) {

    // Return minimum distance between line segment vw and point p
    const double lengthSegment = calculateGPSDistance(segmentPt2, segmentPt1);
    if (lengthSegment == 0.0){
        const double gpsDistance = calculateGPSDistance(point, segmentPt1);
        const std::vector<double> result = {gpsDistance,0};
        return result; // v == w case
    }
    //Lat Lon to Cartesian coordinates
    const Eigen::Vector3d a = latLonToCartesian(segmentPt1);
    const Eigen::Vector3d b = latLonToCartesian(segmentPt2);
    const Eigen::Vector3d m = latLonToCartesian(point);

    const Eigen::Vector3d worldOrigin(0, 0, 0);
    //Creation of the triangle
    const Eigen::Vector3d triangle[3] = {worldOrigin, a, b};

    if (projectionInsideSlice(triangle, m)) {
        const double gpsDistance = std::abs(signedDistanceFromLine(segmentPt1,
                                                                   segmentPt2,
                                                                   point));
        const std::vector<double> result = {gpsDistance,2};
        return result;
    }
    else { //projection is not inside the slice
        const std::vector<Eigen::Vector2d> line = {segmentPt1,segmentPt2};
        const Eigen::Vector2d closestVertex = getClosestVertex(line,point);
        const double gpsDistance = calculateGPSDistance(point, closestVertex);
        if(closestVertex==segmentPt1){
            const std::vector<double> result = {gpsDistance,0};
            return result;
        }
        else{
            const std::vector<double> result = {gpsDistance,1};
            return result;
        }

    }
}

Eigen::Vector2d CollisionAvoidanceNode::cartesianToLatLon(
        Eigen::Vector3d vector){
    const Eigen::Vector2d gpsCoord(atan2(vector(1),vector(0)),
                                   asin(vector(2)/vector.norm()));
    return gpsCoord;
}

Eigen::Vector3d CollisionAvoidanceNode::latLonToCartesian(
        Eigen::Vector2d vector){
    const Eigen::Vector3d cartesianVector(EARTH_RADIUS*cos(vector(1))*cos(vector(0)),
                                          EARTH_RADIUS*cos(vector(1))*sin(vector(0)),
                                          EARTH_RADIUS*sin(vector(1)));
    return cartesianVector;
}

bool CollisionAvoidanceNode::projectionInsideSlice(
        const Eigen::Vector3d triangle[3],
        Eigen::Vector3d point){
    // If this is not a slice
    if(std::abs((triangle[1]-triangle[0]).norm()-(triangle[2]-triangle[0]).norm()) > 10){
        printf("[ERROR](projectionInsideSlice) Not a triangle\n");
        printf("[ERROR_DETAIL](projectionInsideSlice) Vertice 1 : ");
        std::cout << (triangle[1]-triangle[0]).norm() << "\n";
        printf("[ERROR_DETAIL](projectionInsideSlice) Vertice 2 : ");
        std::cout << (triangle[2]-triangle[0]).norm() << "\n";
        printf("[ERROR_DETAIL](projectionInsideSlice) Difference : ");
        std::cout << std::abs((triangle[1]-triangle[0]).norm()-(triangle[2]-triangle[0]).norm()) << "\n";
        return false;
        //TODO find a way to stop the program here.
    }

    const Eigen::Vector3d u = triangle[1]-triangle[0];
    const Eigen::Vector3d v = triangle[2]-triangle[0];
    const Eigen::Vector3d n = u.cross(v);
    const Eigen::Vector3d w = point-triangle[0];
    const double coord2 = (u.cross(w).dot(n))/(n.norm()*n.norm());
    //std::cout << "coord2 : " << coord2 << "\n";
    const double coord1 = (w.cross(v).dot(n))/(n.norm()*n.norm());
    //std::cout << "coord1 : " << coord1 << "\n";
    //std::cout << "coord0 : " << 1-coord2-coord1 << "\n";
    //const double coord0 = 1-coord2-coord1;
    //std::cout << "cond1 : " << (coord1>=0.0) << "\n";

    return (coord2>=0.0)&&(coord1>=0.0);
}

boostPolygon CollisionAvoidanceNode::eigenPolyToBoostPoly(
        Obstacle obstacle){
    boostPolygon poly;
    for (auto &point : obstacle.polygon) {
        boost::geometry::append(poly.outer(), boostPoint(point(0), point(1)));
    }
    // add at the end the first point to close the polygon.
    boost::geometry::append(poly.outer(), boostPoint(obstacle.polygon[0](0),
                                                     obstacle.polygon[0](1)));
    // boost lib define its polygon clockwise and ours are counterclockwise
    boost::geometry::reverse(poly);
    return poly;
}

std::vector<Eigen::Vector2d> CollisionAvoidanceNode::boostPolyToEigenPoly(
        boostPolygon boostPoly){
    std::vector<Eigen::Vector2d> poly;
    //reverse polygon
    boost::geometry::reverse(boostPoly);
    //remove point at the end
    boostPoly.outer().erase(boostPoly.outer().end());
    for (auto &point : boostPoly.outer()) {
        poly.push_back(Eigen::Vector2d(point.get<0>(),point.get<1>()));
    }
    return poly;
}

void CollisionAvoidanceNode::updateObstacleWithBoostPoly(
        Obstacle & obstacle,
        boostPolygon poly){
    // From clockwise to counterClockwise
    boost::geometry::reverse(poly);

    // The old obstacle is updated without the last point
    obstacle.polygon.clear();
    for (int i = 0 ; i< static_cast<signed>(poly.outer().size()-1); i++){
        const Eigen::Vector2d pt(poly.outer()[i].get<0>(), poly.outer()[i].get<1>());
        obstacle.polygon.push_back(pt);
    }

    // Computes the new center
    obstacle.center = findCenter(obstacle.polygon);
}

void CollisionAvoidanceNode::cleanObstacles(
        SensorData sensorData,
        std::vector<Obstacle> &seenObstacles){
    int i = 0;
//    std::cout << "Check_obstacle reached\n";
//    std::cout << "Before clean seenObstacles.size() = " << seenObstacles.size() << "\n";
    if (seenObstacles.size() != 0) {
        // for each memorized obstacles
        while (i < static_cast<signed>(seenObstacles.size())) {
            //the current memorized obstacle does not belong to detectedObstacles
            /*
             * The computations made here verify that an obstacle isn't too far from any other detected
             * obstacle and is not detected : that would mean that it doesn't exist any more and then need
             * to be cleaned from the memory
             */

            //The obstacle is not up to date any more
            seenObstacles[i].upToDate = false;

            //Init of the conditions
            bool obstaclesAreNotTooCloseWithCurrentMemorizedObstacle = 1;
            // for each detected obstacles it will check if it's close to the memorized one
            // if not it should be cleaned from the memory.
            for (auto &sensDatObstacle : sensorData.detectedObstacles) {

                //Computation of several useful variables
                const double headingCenterOfDetectedObstacle =
                        wrapToPi(wrapToPi(sensDatObstacle.LeftBoundheadingRelativeToBoat,
                                          sensDatObstacle.RightBoundheadingRelativeToBoat)/2.0,
                                 sensDatObstacle.boatHeadingAtDetection);
                const double widthOfCurrentDetectedObstacleAtClosest =
                        sensDatObstacle.minDistanceToObstacle
                        * atan(wrapToPi(sensDatObstacle.LeftBoundheadingRelativeToBoat,
                                        - sensDatObstacle.RightBoundheadingRelativeToBoat) / 2.0);

                //Computation of the best center for the detected obstacle
                const double centerDistance = widthOfCurrentDetectedObstacleAtClosest / 2.0
                                              + sensDatObstacle.minDistanceToObstacle;
                const Eigen::Vector2d centerDetectedObstacle =
                        getPointWithDistanceAndBearing(centerDistance,
                                                       headingCenterOfDetectedObstacle,
                                                       Eigen::Vector2d(sensDatObstacle.boatLonAtDetection,
                                                                       sensDatObstacle.boatLatAtDetection));

                // 1rst Condition : current obstacle is not too close from the boat
                const bool currentObstaclesIsNotTooClose = calculateGPSDistance(seenObstacles[i].center,
                                                                                centerDetectedObstacle)
                                                           - widthOfCurrentDetectedObstacleAtClosest
                                                           > DISTANCE_NOT_THE_SAME_OBSTACLE;
//                std::cout << "reduced dist = " <<  calculateGPSDistance(seenObstacles[i].center,
//                                                                        centerDetectedObstacle)
//                                                   - widthOfCurrentDetectedObstacleAtClosest << "\n";
                obstaclesAreNotTooCloseWithCurrentMemorizedObstacle =
                        obstaclesAreNotTooCloseWithCurrentMemorizedObstacle
                        && currentObstaclesIsNotTooClose;
            }
            // 2nd condition : the obstacle should have been detected
            const bool obstacleInsideRange =  // Same error given by the IDE
                    calculateGPSDistance(seenObstacles[i].center, sensorData.gpsPos) < MAXIMUM_SENSOR_RANGE;
            const bool obstacleInsideArc =
                    std::abs(wrapToPi(
                            atan2(seenObstacles[i].center(1) - sensorData.gpsPos(1),
                                  seenObstacles[i].center(0) - sensorData.gpsPos(0)),
                            - wrapToPi(sensorData.compHeading,
                                       SENSOR_HEADING_RELATIVE_TO_BOAT))
                    )
                    < SENSOR_ARC_ANGLE;
            const bool obstacleShouldHaveBeenDetected =
                    (obstacleInsideRange)
                    && (obstacleInsideArc);

            // Where the conditions are used
//            std::cout << "Condition 1 obstacleShouldHaveBeenDetected = " << obstacleShouldHaveBeenDetected << "\n";
//            std::cout << "Condition 2 obstaclesAreNotTooCloseWithCurrentMemorizedObstacle = " << obstaclesAreNotTooCloseWithCurrentMemorizedObstacle << "\n";
            if (obstacleShouldHaveBeenDetected && obstaclesAreNotTooCloseWithCurrentMemorizedObstacle) {
                // Remove the undetected obstacle from the memory
                seenObstacles.erase(seenObstacles.begin() + i);
            }
            i++;
        }
    }
}

std::vector<Obstacle> CollisionAvoidanceNode::registerObstacles(
        SensorData sensorData,
        std::vector<Obstacle> seenObstacles){
    std::vector<Obstacle> upToDateObstacles;
    for (auto &sensDatObstacle : sensorData.detectedObstacles) {
        // Currently obstacles are added without merge.
        // polygon is initialized anti-clockwise

        // Init for polygon creation
        const Eigen::Vector2d boatGpsPosAtDetection(sensDatObstacle.boatLonAtDetection,
                                                    sensDatObstacle.boatLatAtDetection);

//        const double obstacleHeading = wrapToPi(wrapToPi(sensDatObstacle.LeftBoundheadingRelativeToBoat,
//                                                         sensDatObstacle.RightBoundheadingRelativeToBoat) / 2.0,
//                                                sensDatObstacle.boatHeadingAtDetection);
//        std::cout << "sensDatObstacle.boatHeadingAtDetection = " << sensDatObstacle.boatHeadingAtDetection << " rad\n";
//        std::cout << "obstacle heading = " << obstacleHeading << " rad\n";

        Obstacle newObstacle;
        const double halfAngleObsWidth = wrapToPi(sensDatObstacle.LeftBoundheadingRelativeToBoat,
                                                  - sensDatObstacle.RightBoundheadingRelativeToBoat) / 2.0;
        const double rightAngle = wrapToPi(sensDatObstacle.RightBoundheadingRelativeToBoat,
                                           wrapToPi(sensDatObstacle.boatHeadingAtDetection,
                                                    SENSOR_HEADING_RELATIVE_TO_BOAT));
        const double leftAngle = wrapToPi(sensDatObstacle.LeftBoundheadingRelativeToBoat,
                                          wrapToPi(sensDatObstacle.boatHeadingAtDetection,
                                                   SENSOR_HEADING_RELATIVE_TO_BOAT));
        const double minDistance = sensDatObstacle.minDistanceToObstacle
                                   / cos(halfAngleObsWidth);
        const double maxDistance = sensDatObstacle.maxDistanceToObstacle
                                   / cos(halfAngleObsWidth);

//        std::cout << "rightAngle = " << rightAngle << "\n";
//        std::cout << "leftAngle = " << leftAngle << "\n";
//        std::cout << "sensDatObstacle.maxDistanceToObstacle * cosAngleFromObsWidth = " << maxDistance << "\n";

        // Polygon creation
        const Eigen::Vector2d pt1 = getPointWithDistanceAndBearing(minDistance, rightAngle,
                                                                   boatGpsPosAtDetection);
        newObstacle.polygon.push_back(pt1);

        const Eigen::Vector2d pt2 = getPointWithDistanceAndBearing(maxDistance, rightAngle,
                                                                   boatGpsPosAtDetection);
        newObstacle.polygon.push_back(pt2);

        const Eigen::Vector2d pt3 = getPointWithDistanceAndBearing(maxDistance, leftAngle,
                                                                   boatGpsPosAtDetection);
        newObstacle.polygon.push_back(pt3);

        const Eigen::Vector2d pt4 = getPointWithDistanceAndBearing(minDistance, leftAngle,
                                                                   boatGpsPosAtDetection);
        newObstacle.polygon.push_back(pt4);

        //Computation of the best center (longer but more accurate than the other)
        const double centerDistance = calculateGPSDistance(pt1, pt4) / 2;
        const double centerAngle = wrapToPi(atan2(pt4(1) - pt1(1), pt4(0) - pt1(0)),
                                            - M_PI / 2);

//        std::cout << "centerAngle = " << centerAngle << "\n";
        const Eigen::Vector2d midPoint = findMidPoint(pt1, pt4);
//        std::cout << "midPoint = " << midPoint << "\n";
        const Eigen::Vector2d bestCenterDetectedObstacle = getPointWithDistanceAndBearing(centerDistance,
                                                                                          centerAngle,
                                                                                          midPoint);
//        std::cout << "bestCenterDetectedObstacle = " << bestCenterDetectedObstacle << "\n";
        newObstacle.center = bestCenterDetectedObstacle;

        //Color
        newObstacle.color = "Null";

        //upToDate
        newObstacle.upToDate=true;

        // Add it as well to the up to date obstacles to save time
        upToDateObstacles.push_back(newObstacle);
    }
//    int testid = 0;
//    for (auto &&seenObstacle : seenObstacles) {
//        testid++;
//        std::cout << "coord obstacle " <<testid<<",0,"<<seenObstacle.polygon[0](1)/M_PI*180<<","<<seenObstacle.polygon[0](0)/M_PI*180 <<"\n";
//    }
    return upToDateObstacles;
}

void CollisionAvoidanceNode::mergeObstacles(
        SensorData sensorData,
        std::vector<Obstacle> upToDateObstacles,
        std::vector<Obstacle> &seenObstacles){

    //std::vector<Obstacle> to std::vector<boostPolygon>
    boostMultiPolygon boostUpToDatePoly; // output of the union
    boostMultiPolygon boostUpToDatePolygons; // used in obstacle creation
    boostMultiPolygon boostUpToDatePolyWithVision; // input of the union
    Obstacle tmp;
    for(auto & upToDateObs : upToDateObstacles){
        boostUpToDatePolygons.push_back(eigenPolyToBoostPoly(upToDateObs));
    }
    boostUpToDatePolyWithVision = boostUpToDatePolygons;

    // No sensor vision polygon
    // Needed to prevent the unseen obstacle to disappear with the intersection
    const double x = sensorData.gpsPos(0);
    const double y = sensorData.gpsPos(1);
    const double heading = wrapToPi(sensorData.compHeading,
                                    SENSOR_HEADING_RELATIVE_TO_BOAT);
    const double rightPointHeading = wrapToPi(heading,- SENSOR_ARC_ANGLE);
    const double leftPointHeading  = wrapToPi(heading,  SENSOR_ARC_ANGLE);
    boostPolygon noSensorVision;
    boost::geometry::append(noSensorVision.outer(),boostPoint(x,y));
    boost::geometry::append(noSensorVision.outer(),boostPoint(x + CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * cos(leftPointHeading),
                                                              y + CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * sin(leftPointHeading)));
    boost::geometry::append(noSensorVision.outer(),boostPoint(x - CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * sin(heading),
                                                              y + CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * cos(heading)));
    boost::geometry::append(noSensorVision.outer(),boostPoint(x - CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * cos(heading),
                                                              y - CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * sin(heading)));
    boost::geometry::append(noSensorVision.outer(),boostPoint(x + CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * sin(heading),
                                                              y - CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * cos(heading)));
    boost::geometry::append(noSensorVision.outer(),boostPoint(x + CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * cos(rightPointHeading),
                                                              y + CONVERSION_FACTOR_METER_TO_GPS
                                                                  * 2000 * sin(rightPointHeading)));
    boost::geometry::append(noSensorVision.outer(),boostPoint(x,y));
    boost::geometry::reverse(noSensorVision);
    boostUpToDatePolyWithVision.push_back(noSensorVision);

    //Union
    //Gives the zone where we know for sure there is something
    boostUpToDatePoly = polyUnion(boostUpToDatePolyWithVision);

//    standAloneDrawBoostMultiPoly(boostUpToDatePoly,"r[r]","boostUpToDatePoly",STANDALONE_DRAW_NEW_FIGURE);
//    standAloneDrawObstacles(seenObstacles,"","boostUpToDatePoly",STANDALONE_DRAW_USE_EXISTING);

    //Intersection
    int j = 0;
    //For each memorized obstacle, makes the intersection with the zone
    while (j<static_cast<signed>(seenObstacles.size())) {
        boostPolygon memPoly = eigenPolyToBoostPoly(seenObstacles[j]);
        boostMultiPolygon output;

//        standAloneDrawBoostPoly(memPoly,"b","boostUpToDatePoly",STANDALONE_DRAW_USE_EXISTING);

        for (auto &&upToDatePoly : boostUpToDatePoly) {
//            standAloneDrawBoostPoly(upToDatePoly,"r[r]","bug test",STANDALONE_DRAW_NEW_FIGURE);
//            standAloneDrawBoostPoly(memPoly,"b[]","bug test",STANDALONE_DRAW_USE_EXISTING);
            boost::geometry::correct(memPoly);
            boost::geometry::correct(upToDatePoly);
            boost::geometry::intersection(upToDatePoly, memPoly, output);
//            standAloneDrawBoostMultiPoly(output,"[g]","bug test",STANDALONE_DRAW_USE_EXISTING);
        }

//        standAloneDrawBoostMultiPoly(output,"[g]","boostUpToDatePoly",STANDALONE_DRAW_USE_EXISTING);

        // seenObstacles[j] is modified according to the output of the intersection
        if(!output.empty()) {

            if (output.size() == 1) { // Intersection sucessful => save it
                updateObstacleWithBoostPoly(seenObstacles[j], output[0]);
            }
            else{ // More than 1 intersection => save it then create the others
                printf("[WARNING](mergeObstacles) Output size > 1 \n");
                printf("____[DETAIL](mergeObstacles) Obstacle split into several ones \n");
                // then just take the biggest polygon (not the best solution but it's working)
                double maxArea = 0;
                double previousMaxArea = 0;
                boostPolygon maxPolygon;
                for (auto &&poly : output) {
                    maxArea = std::max(boost::geometry::area(poly),maxArea);
                    if(maxArea!=previousMaxArea){
                        maxPolygon = poly;
                    }
                    previousMaxArea = maxArea;
                }
                updateObstacleWithBoostPoly(seenObstacles[j],maxPolygon);

//                tmp.color = seenObstacles[j].color;
//                tmp.upToDate = false;
//                for (int i = 0; i < static_cast<signed>(output.size()); i++) {
//                    if (i == 0) {
//                        updateObstacleWithBoostPoly(seenObstacles[j], output[0]);
//                    }
//                    else {
//                        updateObstacleWithBoostPoly(tmp, output[i]);
//                        seenObstacles.push_back(tmp);
//                    }
//                }
                printf("____[DETAIL](mergeObstacles) Might cause uncontrolled obstacle multiplication\n");
                printf("____[DETAIL](mergeObstacles) m_seen_obstacles.size() = %f\n", static_cast<double>(m_seenObstacles.size()));
            }
            j++;
        }
        else{ // No intersection => destroy obstacle
            seenObstacles.erase(seenObstacles.begin()+j);
        }
    }

//    standAloneDrawObstacles(seenObstacles,"","seenObstacles after intersection",STANDALONE_DRAW_NEW_FIGURE);

    // Add the obstacles that intersects with nothing
    for(int i = 0; i<static_cast<signed>(boostUpToDatePolygons.size());i++){
        bool createNewObstacle = true;
        for(auto & memObs : seenObstacles){
            boostPolygon memPoly = eigenPolyToBoostPoly(memObs);
            if(boost::geometry::intersects(boostUpToDatePolygons[i],memPoly)){
                createNewObstacle = false;
            }
        }
        if(createNewObstacle){
            seenObstacles.push_back(upToDateObstacles[i]);
        }
    }

//    standAloneDrawObstacles(seenObstacles,"","seenObstacles after adding",STANDALONE_DRAW_NEW_FIGURE);
}

//void CollisionAvoidanceNode::cleanObstaclesBO( //Bearing only
//        SensorData sensorData,
//        BearingOnlyVars & vars){
//    vars.obstacleBearings.clear();
//}
//
//std::vector<Obstacle> CollisionAvoidanceNode::registerObstaclesBO( //Bearing only
//        SensorData sensorData,
//        BearingOnlyVars & vars){
//    for (auto &&obstacle : sensorData.detectedObstacles) {
//        const double obstacleBearing =
//        vars.obstacleBearings.push_back()
//    }
//}

boostMultiPolygon CollisionAvoidanceNode::polyUnion(
        boostMultiPolygon mpoly) {
    boostMultiPolygon output;
    bool continueUnion = true;
    int i1 = 0;

    while(continueUnion && mpoly.size()!=1) {
        continueUnion = true;
        int i2 = i1+1;
        while (i2 < static_cast<signed>(mpoly.size())) {

//            vibes::clearFigure("hello");
//            drawBoostPoly(mpoly,"hello");

            boostPolygon tmpPoly = mpoly[i1];
            boostPolygon tmpPoly2 = mpoly[i2];
            boost::geometry::union_(tmpPoly, tmpPoly2, output);
            if (output.size() == 1) {
                mpoly.erase(mpoly.begin() + i1);
                mpoly.erase(mpoly.begin() + i2 -1);
                mpoly.insert(mpoly.begin(), output[0]);
            }
            else{
                i2++;
                continueUnion = false;
            }
            output.clear();
        }
        i1++;
    }
    return mpoly;
}

Eigen::ArrayXXd CollisionAvoidanceNode::computeObstaclePot(
        Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,
        SensorData sensorData,
        std::vector<Obstacle> seenObstacles,
        FollowedLine followedLine){
    // TODO : conversion to gps coordinate                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      s
    // TODO : modify obstacle according to its width. ?
    const double scaleHole = 50.0;
    const double scalePike = 550.0;
    const double scale = 0.5;
    const double strengthHoles = 2.0;
    const double strengthPike = 4.0;
    const double strength = 5.0;
    const double offsetObstacle = 5;//15;
    const double offsetHoles = 20;//30;

    // Px size = Py size
    Eigen::ArrayXXd obsPot = Eigen::ArrayXXd::Zero(Px.rows(),Px.cols());
    std::vector<Eigen::Vector2d> closestObstaclesPoints;

    for(auto & obstacle : seenObstacles){
        Eigen::Vector2d closestObstaclePoint = getClosestPoint(obstacle.polygon,
                                                               sensorData.gpsPos);
        closestObstaclesPoints.push_back(closestObstaclePoint);
    }

    // Add the pikes
    for(auto & obstacle : closestObstaclesPoints){ //for each seen_obstacle add the pikes

        const double obstacleHeading = wrapToPi(wrapToPi(fmod(m_sensorOutput.compHeading, 2*M_PI)
                                                         * 0.3,
                                                         fmod(atan2(followedLine.endPoint(1) - obstacle(1),
                                                                    followedLine.endPoint(0) - obstacle(0)),
                                                              2*M_PI)
                                                         * 0.7),
                                                M_PI/2);

        const Eigen::ArrayXXd xObsBeforeRot = Px-obstacle(0)/CONVERSION_FACTOR_METER_TO_GPS;
        const Eigen::ArrayXXd yObsBeforeRot = Py-obstacle(1)/CONVERSION_FACTOR_METER_TO_GPS;

        const Eigen::ArrayXXd xAfterRot =   xObsBeforeRot * cos(obstacleHeading)
                                          + yObsBeforeRot * sin(obstacleHeading);
        const Eigen::ArrayXXd yAfterRot = - xObsBeforeRot * sin(obstacleHeading)
                                          + yObsBeforeRot * cos(obstacleHeading) - offsetObstacle;

        const Eigen::ArrayXXd tPike = ( xAfterRot.square() + (yAfterRot).square() )
                                      /scalePike/scale;

        obsPot = obsPot.max(strength*strengthPike*(-(tPike.square())).exp());
    }

    // Add the holes
    for(auto & obstacle : closestObstaclesPoints){ //for each seen_obstacle add the holes
        const double obstacleHeading = wrapToPi(wrapToPi(fmod(m_sensorOutput.compHeading, 2*M_PI)
                                                         * 0.3,
                                                         fmod(atan2(followedLine.endPoint(1) - obstacle(1),
                                                                    followedLine.endPoint(0) - obstacle(0)),
                                                               2*M_PI)
                                                         * 0.7),
                                                M_PI/2);
        const Eigen::ArrayXXd xObsBeforeRot = Px - obstacle(0)/CONVERSION_FACTOR_METER_TO_GPS;
        const Eigen::ArrayXXd yObsBeforeRot = Py - obstacle(1)/CONVERSION_FACTOR_METER_TO_GPS;

        const Eigen::ArrayXXd xRightHoleAfterRot =   xObsBeforeRot * cos(obstacleHeading)
                                                   + yObsBeforeRot * sin(obstacleHeading)
                                                   + offsetHoles;
        const Eigen::ArrayXXd yRightHoleAfterRot = - xObsBeforeRot * sin(obstacleHeading)
                                                   + yObsBeforeRot * cos(obstacleHeading)
                                                   - offsetObstacle;
        const Eigen::ArrayXXd xLeftHoleAfterRot  =   xObsBeforeRot * cos(obstacleHeading)
                                                   + yObsBeforeRot * sin(obstacleHeading)
                                                   - offsetHoles;
        const Eigen::ArrayXXd yLeftHoleAfterRot  = - xObsBeforeRot * sin(obstacleHeading)
                                                   + yObsBeforeRot * cos(obstacleHeading)
                                                   - offsetObstacle;

        const Eigen::ArrayXXd tRightHole = (  xRightHoleAfterRot.square()
                                            + yRightHoleAfterRot.square())
                                           / scaleHole / scale;
        const Eigen::ArrayXXd tLeftHole  = (   xLeftHoleAfterRot.square()
                                            +  yLeftHoleAfterRot.square())
                                           / scaleHole / scale;

        obsPot = obsPot
                 - strength*strengthHoles*(-(tRightHole).square()).exp()
                 - strength*strengthHoles*(-(tLeftHole ).square()).exp();
    }

    return obsPot;
}

Eigen::ArrayXXd CollisionAvoidanceNode::computeObjectivePot(
        Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,
        FollowedLine followedLine){
    // Values found after extensive testing
    const double objectiveSpread = 1/25000.0;
    const double objectiveStrength = 1.6;
    const Eigen::ArrayXXd objPot = objectiveStrength*((- (Px - followedLine.endPoint(0)
                                                               / CONVERSION_FACTOR_METER_TO_GPS).square()
                                                       - (Py - followedLine.endPoint(1)
                                                               / CONVERSION_FACTOR_METER_TO_GPS).square())
                                                      *objectiveSpread).exp();
    return objPot;
}

Eigen::ArrayXXd CollisionAvoidanceNode::computeWindPot(
        Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,
        SensorData sensorData){
    const double noGoZoneAngle = 3*M_PI/4.0;
    const double windStrength = 10;

    const Eigen::ArrayXXd xWindBeforeRot = Px - sensorData.gpsPos(0)
                                                / CONVERSION_FACTOR_METER_TO_GPS;
    const Eigen::ArrayXXd yWindBeforeRot = Py - sensorData.gpsPos(1)
                                                / CONVERSION_FACTOR_METER_TO_GPS;
    const double noGoZoneHeading = sensorData.windDirection + noGoZoneAngle;
    const Eigen::ArrayXXd xWindAfterRot =   xWindBeforeRot*cos(noGoZoneHeading)
                                          + yWindBeforeRot*sin(noGoZoneAngle);
    const Eigen::ArrayXXd yWindAfterRot = - xWindBeforeRot*sin(noGoZoneHeading)
                                          + yWindBeforeRot*cos(noGoZoneHeading);

    // Change from atan to 1/(1+exp(-2*t))
    const double windChangeSlope = 10; // Higher it is the slopiest it is
    const Eigen::ArrayXXd windPot = windStrength * (1/(1+(-windChangeSlope*xWindAfterRot).exp()))
                                                 * (1/(1+(-windChangeSlope*yWindAfterRot).exp()));
    return windPot;
}

Eigen::ArrayXXd CollisionAvoidanceNode::computeBoatPot(
        Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,
        SensorData sensorData){
    const double strengthBoat = 3;
    const double strengthHoleBoat = 1.5;
    const double spreadHoleBoat = 4000;
    const double strengthPikeBoat = 5;
    const double spreadPikeBoat = 200;

    const Eigen::ArrayXXd boatHolePot = strengthHoleBoat
                                        * ((-(Px-sensorData.gpsPos(0) / CONVERSION_FACTOR_METER_TO_GPS).square()
                                            -(Py-sensorData.gpsPos(1) / CONVERSION_FACTOR_METER_TO_GPS).square())
                                           / spreadHoleBoat).exp();
    const Eigen::ArrayXXd boatPikePot = strengthPikeBoat
                                        * ((-(Px-sensorData.gpsPos(0) / CONVERSION_FACTOR_METER_TO_GPS).square()
                                            -(Py-sensorData.gpsPos(1) / CONVERSION_FACTOR_METER_TO_GPS).square())
                                           / spreadPikeBoat).exp();
    const Eigen::ArrayXXd boatPot = strengthBoat*( -boatHolePot + boatPikePot );

    return boatPot;
}

Eigen::ArrayXXd CollisionAvoidanceNode::computeSailingZonePot(
        Eigen::ArrayXXd Px,Eigen::ArrayXXd Py,
        std::vector<Eigen::Vector2d> sailingZone){

    Eigen::ArrayXXd sailingZoneMatrix = Eigen::ArrayXXd::Ones(Px.rows(),Px.cols());//New blank matrix
    for(int i = 0;i<static_cast<signed>(sailingZone.size());i++){
        // The lines of the polygon are added incrementally

        const int iPlusOne = static_cast<int>(fmod(i+1,sailingZone.size())); // index of the next point
        const Eigen::Vector2d linePt0 = sailingZone[i];
        const Eigen::Vector2d linePt1 = sailingZone[iPlusOne];
        const Eigen::Vector2d origin(0,0);

        // Distance between the origin and the line (AB)
        const double offsetFromOrigin = signedDistanceFromLine(linePt0,linePt1,origin); // in meters
//        Eigen::Matrix2d mat;
//        mat << (linePt1-linePt0)/(linePt1-linePt0).norm(),(origin-linePt0);
//        const double offsetFromOrigin = mat.determinant();

        const double lineHeading = atan2(linePt1(1)-linePt0(1),
                                         linePt1(0)-linePt0(0)); // Heading to the line (AB)

        // const Eigen::ArrayXXd xLine = (Px)*cos(lineHeading)+(Py)*sin(lineHeading); // saved in a comment
        const Eigen::ArrayXXd yLine =   (Px)*cos(wrapToPi(lineHeading,M_PI/2))
                                      + (Py)*sin(wrapToPi(lineHeading,M_PI/2));
        // On one side of the line there is 1, 0 on the other
        // offsetFromOrigin is used to move the line from the origin

        const double sailingZoneChangeSlope = 10;
        const Eigen::ArrayXXd line =  (1 / ( 1 + ( - sailingZoneChangeSlope
                                                     * (yLine + offsetFromOrigin)
                                                 ).exp() ));

        sailingZoneMatrix = sailingZoneMatrix*line;
    }

    const double strengthSailingZone = 10;
    const Eigen::ArrayXXd sailZonePot = strengthSailingZone*(1-sailingZoneMatrix);

    return sailZonePot;
}
