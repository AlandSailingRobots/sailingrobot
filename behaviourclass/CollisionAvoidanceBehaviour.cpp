//
// Created by Simon CHANU on 11/07/16.
//

#include "CollisionAvoidanceBehaviour.h"

//INPUT
/* Current_line (WP1 and WP2)
 * Boat_state
 *     GPS position
 *     roll, pitch, yaw
 *     speed
 * Sensors_output
 * sailing_zone (gps points)
 * wind direction and strength
 */
//IDEA
/* To use dynamic list of struct for obstacles and other variables such as this one
 */
//I WANT TO TRACK
/* boat position and heading
 * the obstacles with everything
 */
//SAVED CODE IN CASE OF ARCHITECTURAL CHANGE
/*

//sensorData update_sensor(){}

//    commandOutput run(INPUT){
//        sensors output = update_sensors(); //=> gives sensors output or compute an easier way to handle them
//        update_waypoints(); //=> update waypoints or compute an easier way to handle them
//        check_obstacles();
//    //    update_map();
//        if(these_obstacles_are_a_problem()){
//            Eigen::MatrixXd potential_field = compute_potential_field();
//            minPotField min = find_minimum_potential_field(potential_field);
//            compute_new_path();
//        }
//        return compute_commands();
//    }


*/

//CONSTRUCTOR

CollisionAvoidanceBehaviour::CollisionAvoidanceBehaviour(DBHandler *db) :
        RoutingBehaviour(db) {}

// UTILITY FUNCTIONS

// Might be put into Utility class soon. TODO : Utility class ?
double CollisionAvoidanceBehaviour::angleDiff(
        double radAngle1,
        double radAngle2) {
    return fmod(radAngle1-radAngle2+M_PI,2*M_PI)+M_PI;
}

// Maybe this function should as well be in the Utility class. TODO : Utility class ?
double CollisionAvoidanceBehaviour::calculateGPSDistance(
        Eigen::Vector2d point1,
        Eigen::Vector2d point2) {
    //...(1) : latitude //...(2) : longitude
    const double deltaLat = angleDiff(point2(1),point1(1));
    const double deltaLon = angleDiff(point2(2),point1(2));
    const double Rearth = 6371000;
    const double a  = sin(deltaLat/2)*sin(deltaLat/2)
                      + cos(point1(1))*cos(point2(1))*sin(deltaLon/2)*sin(deltaLon/2);
    const double c = 2*atan2(sqrt(a),sqrt(1-a));
    return Rearth*c;
}
/*
 * TODO : Utility class ?
 */
Eigen::Vector2d CollisionAvoidanceBehaviour::findCenter(
        const std::vector<Eigen::Vector2d> polygon) {
    double sumOfX = 0;
    double sumOfY = 0;
    for(auto & vec : polygon){
        sumOfX += vec(0);
        sumOfY += vec(1);
    }
    const Eigen::Vector2d meanPolygon(sumOfX/polygon.size(),sumOfY/polygon.size());
    return meanPolygon;
}


// TODO : move to Utility ?
double CollisionAvoidanceBehaviour::getArea(
        std::vector<Eigen::Vector2d> polygon){
    double sum = 0;
    for(int i = 0;i<polygon.size()-1;i++){
        sum += polygon[i](0)*polygon[i+1](1)-polygon[i+1](0)*polygon[i](1);
    }
    return sum/2;
}

double CollisionAvoidanceBehaviour::signedDistanceFromLine(
        Eigen::Vector2d linePt1,
        Eigen::Vector2d linePt2,
        Eigen::Vector2d point){

    //Lat Lon to Cartesian coordinates
    const Eigen::Vector3d a = latLonToCartesian(linePt1);
    const Eigen::Vector3d b = latLonToCartesian(linePt2);
    const Eigen::Vector3d m = latLonToCartesian(point);

    const Eigen::Vector3d n = (a.cross(b)) / (a.norm() * b.norm());
    const double distFromTriangle = m.transpose() * n;
    const double arcLenght = EARTH_RADIUS * asin(distFromTriangle / EARTH_RADIUS);
    return arcLenght;
}

Eigen::Vector2d CollisionAvoidanceBehaviour::getClosestPoint(
        std::vector<Eigen::Vector2d> polygon, Eigen::Vector2d point){
    int idClosestPoint = -1;
    double minDistance = 40000;
    double distance;
    for(int i = 0;i<polygon.size();i++) {
        distance = calculateGPSDistance(polygon[i], point);
        minDistance = std::min(minDistance, distance);
        if (minDistance == distance) {
            idClosestPoint = i;
        }
    }
    return polygon[idClosestPoint];
}

double CollisionAvoidanceBehaviour::distanceFromSegment(
        Eigen::Vector2d segmentPt1,
        Eigen::Vector2d segmentPt2,
        Eigen::Vector2d point) {

    // Return minimum distance between line segment vw and point p
    const double lengthSegment = calculateGPSDistance(segmentPt2, segmentPt1);
    if (lengthSegment == 0.0) return calculateGPSDistance(point, segmentPt1);   // v == w case

    //Lat Lon to Cartesian coordinates
    const Eigen::Vector3d a = latLonToCartesian(segmentPt1);
    const Eigen::Vector3d b = latLonToCartesian(segmentPt2);
    const Eigen::Vector3d m = latLonToCartesian(point);

    const Eigen::Vector3d worldOrigin(0, 0, 0);
    //Creation of the triangle
    const Eigen::Vector3d triangle[3] = {worldOrigin, a, b};

    if (projectionInside3DTriangle(triangle, m)) {
        return std::abs(signedDistanceFromLine(segmentPt1,segmentPt2,point));
    }
    else { //projection is not inside the triangle
        const std::vector<Eigen::Vector2d> line = {segmentPoint1,segmentPoint2};
        const Eigen::Vector2d closestPoint = getClosestPoint(line,point);
        return calculateGPSDistance(point, closestPoint);
    }
}

Eigen::Vector2d CollisionAvoidanceBehaviour::cartesianToLatLon(Eigen::Vector3d vector){
    const Eigen::Vector2d gpsCoord(atan2(vector(1),vector(0)),
                                   asin(vector(2)/EARTH_RADIUS));
    return gpsCoord;
}

Eigen::Vector3d CollisionAvoidanceBehaviour::latLonToCartesian(Eigen::Vector2d vector){
    const Eigen::Vector3d cartesianVector(EARTH_RADIUS*cos(vector(1))*cos(vector(0)),
                                          EARTH_RADIUS*cos(vector(1))*sin(vector(0)),
                                          EARTH_RADIUS*sin(vector(1)));
    return cartesianVector;
}

bool CollisionAvoidanceBehaviour::projectionInside3DTriangle(Eigen::Vector3d triangle[3],Eigen::Vector3d point){
    const Eigen::Vector3d u = triangle[1]-triangle[0];*
    const Eigen::Vector3d v = triangle[2]-triangle[0];
    const Eigen::Vector3d n = u.cross(v);
    const Eigen::Vector3d w = point-triangle[0];
    const double coord0 = (u.cross(w).dot(n))/(n.norm()*n.norm());
    const double coord1 = (w.cross(v).dot(n))/(n.norm()*n.norm());
    const double coord2 = 1-coord0-coord1;

    if(   0<=coord0 && coord0<1
       && 0<=coord1 && coord1<1
       && 0<=coord2 && coord2<1){
        return true;
    }
    else{
        return false;
    }
}


void CollisionAvoidanceBehaviour::printStdVectorMat(
        std::string const &name,
        std::vector<Eigen::MatrixXd> const &v) {

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    for (int i = 0; i < (int) v.size(); ++i) {
        std::cout << v[i] << std::endl;
        std::cout << " " << std::endl;
    }
}
void CollisionAvoidanceBehaviour::printStdVectorFloat(
        std::string const &name,
        std::vector<float> const &v) {

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    for (int i = 0; i < (int) v.size(); ++i) {
        std::cout << v[i] << std::endl;
        std::cout << " " << std::endl;
    }
}
void CollisionAvoidanceBehaviour::printMat(
        std::string const &name,
        Eigen::MatrixXd const &mat) {

    std::cout << " " << std::endl;
    std::cout << name << " : " << std::endl;
    std::cout << " " << std::endl;
    std::cout << mat << std::endl;
    std::cout << " " << std::endl;
}

//PRIVATE MAIN FUNCTIONS

SensorData CollisionAvoidanceBehaviour::update_sensors(
        SystemStateModel &systemStateModel,
        const Simulation sim) {
    SensorData sensorData;
    if (sim.waypoints) {
        // TODO : simulation part (called or made here)
    }
    else {
        //Extraction of data from sensors

        //Position and speed
        //The latitude and longitude are easier to compute in radians
        sensorData.gpsPos(0) = // x
                Utility::degreeToRadian(systemStateModel.gpsModel.positionModel.latitude);
        sensorData.gpsPos(1) = // y
                Utility::degreeToRadian(systemStateModel.gpsModel.positionModel.longitude);
        sensorData.gpsSpeed = systemStateModel.gpsModel.speed;

        //Heading
        //compHeading : degree from north -> radian from east
        sensorData.compHeading = Utility::degreeToRadian(systemStateModel.compassModel.heading)
                                 - M_PI / 2;
        //gpsHeading : degree from north -> radian from east
        sensorData.gpsHeading = Utility::degreeToRadian(systemStateModel.gpsModel.heading)
                                - M_PI / 2;

        //Wind
        //windDirection : degree from north -> radian from east
        sensorData.windDirection = Utility::degreeToRadian(systemStateModel.windsensorModel.direction)
                                   - M_PI / 2;
        sensorData.windSpeed = systemStateModel.windsensorModel.speed;

        //Tilt
        sensorData.pitch = systemStateModel.compassModel.pitch;
        sensorData.roll = systemStateModel.compassModel.roll;
    }

    //ASSUMPTION
    /*
     * The sensors will give a confidence interval of the heading and the distance
     * relatively to the boat
     */
    if (sim.obstacles) {
        //Mock obstacles here
        // [UPDATE] Elouan said this script will have to take care of the obstacles without his simulator.
        // TODO : Create a simulation class/node which will compute every sensor output. I might only have to modify Elouan's

        // This don't work right now, this is an example.
        // TODO : make working mock obstacles
        ObstacleData obstacle0 = {2,  //double minDistanceToObstacle;
                                  20, //double maxDistanceToObstacle;
                                  -10,//double LeftBoundheadingRelativeToBoat;
                                  10};//double RightBoundheadingRelativeToBoat;
        sensorData.detectedObstacles.push_back(obstacle0);
        ObstacleData obstacle1 = {2,  //double minDistanceToObstacle;
                                  20, //double maxDistanceToObstacle;
                                  -10,//double LeftBoundheadingRelativeToBoat;
                                  10};//double RightBoundheadingRelativeToBoat;
        sensorData.detectedObstacles.push_back(obstacle1);
        ObstacleData obstacle2 = {2,  //double minDistanceToObstacle;
                                  20, //double maxDistanceToObstacle;
                                  -10,//double LeftBoundheadingRelativeToBoat;
                                  10};//double RightBoundheadingRelativeToBoat;
        sensorData.detectedObstacles.push_back(obstacle2);
        ObstacleData obstacle3 = {2,  //double minDistanceToObstacle;
                                  20, //double maxDistanceToObstacle;
                                  -10,//double LeftBoundheadingRelativeToBoat;
                                  10};//double RightBoundheadingRelativeToBoat;
        sensorData.detectedObstacles.push_back(obstacle3);

    }
    else {
        // TODO : when the sensors wil be ready, put the code to get everything here

    }

    return sensorData;
}

/*
 * Update waypoints or compute an easier way to handle them
 */
//    void update_waypoints(){}

std::vector<Obstacle> CollisionAvoidanceBehaviour::check_obstacles(SensorData sensorData) {
    //Clean obstacles
    int i = 0;
    while(i<=seenObstacles.size()){
        //the current memorized obstacle does not belong to detectedObstacles
        /*
         * The computations made here verify that an obstacle isn't too far from any other detected
         * obstacle and is not detected : that would mean that it doesn't exist any more and then need
         * to be cleaned from the memory
         */
        const Eigen::Vector2d centerOfMemorizedObstacle = findCenter(seenObstacles[i].polygon);

        bool obstaclesAreNotTooCloseWithCurrentMemorizedObstacle;
        for(auto & sensDatObstacle : sensorData.detectedObstacles) {
            const double headingCenterOfDetectedObstacle =
                    angleDiff(sensDatObstacle.LeftBoundHeadingRelativeToBoat,
                              sensDatObstacle.RightBoundHeadingRelativeToBoat)
                    + sensorData.compHeading;
            const double widthOfCurrentDetectedObstacleAtClosest =
                    sensDatObstacle.minDistanceToObstacle
                    * tan(angleDiff(sensDatObstacle.LeftBoundHeadingRelativeToBoat,
                                    sensDatObstacle.RightBoundHeadingRelativeToBoat)
                          / 2);

            const Eigen::Vector2d centerOfCurrentlyDetectedObstacle(
                    sensorData.gpsPos(0)
                    + sensDatObstacle.minDistanceToObstacle
                      * cos(headingCenterOfDetectedObstacle),// x
                    sensorData.gpsPos(1)
                    + sensDatObstacle.minDistanceToObstacle
                      * sin(headingCenterOfDetectedObstacle) // y
            );

            // 1rst Condition
            const bool currentObstaclesIsNotTooClose = calculateGPSDistance(centerOfMemorizedObstacle,
                                                                            centerOfCurrentlyDetectedObstacle)
                                                    + widthOfCurrentDetectedObstacleAtClosest * 2
                                                    > DISTANCE_NOT_THE_SAME_OBSTACLE;
            obstaclesAreNotTooCloseWithCurrentMemorizedObstacle =
                    obstaclesAreNotTooCloseWithCurrentMemorizedObstacle && currentObstaclesIsNotTooClose;
        }
        // 2nd condition
        const bool obstacleInsideRange =  // Same error given by the IDE
                calculateGPSDistance(centerOfMemorizedObstacle,sensorData.gpsPos) < MAXIMUM_SENSOR_RANGE;
        const bool obstacleInsideArc =
                std::abs(angleDiff(
                        atan2(centerOfMemorizedObstacle(1)-sensorData.gpsPos(1),
                              centerOfMemorizedObstacle(0)-sensorData.gpsPos(0)),
                        sensorData.compHeading + SENSOR_HEADING_RELATIVE_TO_BOAT))
                < SENSOR_ARC_ANGLE;
        const bool obstacleShouldHaveBeenDetected =
                (obstacleInsideRange)
                &&(obstacleInsideArc);

        if(obstacleShouldHaveBeenDetected && obstaclesAreNotTooCloseWithCurrentMemorizedObstacle){
            // Remove the undetected obstacle from the memory
            seenObstacles.erase(seenObstacles.begin()+i);
        }
        i++;
    }

    //Register new obstacles
    for(auto & sensDatObstacle : sensorData.detectedObstacles){
        // Currently obstacles are added without merge.
        // polygon is initialized anti-clockwise
        Obstacle newObstacle;

        //Obstacle from scratch for now
        const Eigen::Matrix2d rotation(cos(sensorData.compHeading + SENSOR_HEADING_RELATIVE_TO_BOAT),
                                       -sin(sensorData.compHeading+ SENSOR_HEADING_RELATIVE_TO_BOAT),
                                       cos(sensorData.compHeading + SENSOR_HEADING_RELATIVE_TO_BOAT),
                                       sin(sensorData.compHeading + SENSOR_HEADING_RELATIVE_TO_BOAT));

        // Point 1
        const double x1RelativeToBoat = sensDatObstacle.minDistanceToObstacle
                                        * tan(sensDatObstacle.RightBoundHeadingRelativeToBoat);
        const double y1RelativeToBoat = sensDatObstacle.minDistanceToObstacle;
        Eigen::Vector2d pt1(x1RelativeToBoat,y1RelativeToBoat);
        pt1 = pt1*rotation;
        newObstacle.polygon.push_back(pt1);

        // Point 2
        const double x2RelativeToBoat = sensDatObstacle.maxDistanceToObstacle
                                        * tan(sensDatObstacle.RightBoundHeadingRelativeToBoat);
        const double y2RelativeToBoat = sensDatObstacle.maxDistanceToObstacle;
        Eigen::Vector2d pt2(x1RelativeToBoat,y1RelativeToBoat);
        pt2 = pt2*rotation;
        newObstacle.polygon.push_back(pt2);

        // Point 3
        const double x3RelativeToBoat = sensDatObstacle.maxDistanceToObstacle
                                        * tan(sensDatObstacle.LeftBoundHeadingRelativeToBoat);
        const double y3RelativeToBoat = sensDatObstacle.maxDistanceToObstacle;
        Eigen::Vector2d pt3(x1RelativeToBoat,y1RelativeToBoat);
        pt3 = pt3*rotation;
        newObstacle.polygon.push_back(pt3);

        // Point 4
        const double x4RelativeToBoat = sensDatObstacle.minDistanceToObstacle
                                        * tan(sensDatObstacle.LeftBoundHeadingRelativeToBoat);
        const double y4RelativeToBoat = sensDatObstacle.minDistanceToObstacle;
        Eigen::Vector2d pt4(x4RelativeToBoat,y4RelativeToBoat);
        pt4 = pt4*rotation;
        newObstacle.polygon.push_back(pt4);

        newObstacle.color = "Null";

        // Add new obstacle to the list
        seenObstacles.push_back(newObstacle);
    }

    for(auto & sensDatObstacle : sensorData.detectedObstacles){
        // TODO update of the obtacles with polygon intersection
    }
}

/*
 *
 */
//map update_map(){}

bool CollisionAvoidanceBehaviour::these_obstacles_are_a_problem(
        std::vector<Obstacle> seenObstacles) { // OUTPUT if these obstacles are a problem
    bool theseObstaclesAreAProblem = false;
    // We have polygons. We need to see if there is an intersection between
    // the channel and the polygons. That is to say : one of the points of the
    // polygon is inside the channel

    for(auto & obstacle : seenObstacles){
        for(auto & point : obstacle.polygon){
            // compute the distance between the line and the point.
            double signedDistance = signedDistanceFromLine(followedLine.startPoint,
                                                                  followedLine.endPoint,
                                                                  point);
            bool thisObstacleIsAProblem = false;
            if(std::abs(signedDistance)< CHANNEL_WIDTH
               && calculateGPSDistance(sensorOutput.gpsPos,getClosestPoint(obstacle.polygon)) < SAFE_DISTANCE){
                thisObstacleIsAProblem = true;
            }
            theseObstaclesAreAProblem = theseObstaclesAreAProblem || thisObstacleIsAProblem;
        }
    }
    return theseObstaclesAreAProblem;
}

Eigen::MatrixXd CollisionAvoidanceBehaviour::compute_potential_field(
        std::vector<Obstacle> seen_obstacles,
        std::vector<Eigen::Vector2d> sailing_zone,
        FollowedLine) {
    // Set up of the matrix of 100*100 around the boat.
    Eigen::MatrixXd potField;

    // Init

    // TODO Obstacle potential function
    /*
     * Since the obstacle is a polygon, only the closest point from
     * the polygon will be counted as an obstacle
     */
    const double scaleHole = 50;
    const double scalePike = 550;
    const double scale = 0.5;
    const double strengthHoles = 2;
    const double strengthPike = 4;
    const double strength = 5;
    const double offsetObstacle = 15;
    // Add the pikes

    /*
    for i=1:size(qhat,2)
    xObs = P1-qhat(1,i);
    yObs = P2-qhat(2,i);
    tPike = ((xObs*1).^2+(yObs-offsetObstacle*scale).^2)/scalePike/scale;
    ObsP = max(ObsP, strength*strengthPike*exp(-(tPike).^2));
    end
    % Then add the holes
    for i=1:size(qhat,2)
    T = mod(boatHeading,2*pi)*0.3 + mod( atan2(phat(2)-qhat(2,i),phat(1)-qhat(1,i)) ,2*pi)*0.7 + pi/2;
    xObs = P1-qhat(1,i);
    yObs = P2-qhat(2,i);
    xo =  xObs*cos(T) + yObs*sin(T);
    yo = -xObs*sin(T) + yObs*cos(T);
    tHoleR = ((xo-35*scale).^2+(yo-offsetObstacle*scale).^2)/scaleHole/scale;
    tHoleL = ((xo+35*scale).^2+(yo-offsetObstacle*scale).^2)/scaleHole/scale;

    ObsP = ObsP - strength*strengthHoles*exp(-(tHoleR).^2) - strength*strengthHoles*exp(-(tHoleL).^2);
    end
    */


    // TODO Objective potential function

    // TODO Windpreference potential function

    // TODO Boat preference

    // TODO Computation of the field

    // TODO Adding Sailing Zone
}

MinPotField CollisionAvoidanceBehaviour::find_minimum_potential_field(
        Eigen::MatrixXd Potential_field) {

}

FollowedLine CollisionAvoidanceBehaviour::compute_new_path(
        MinPotField min) {

}

CommandOutput CollisionAvoidanceBehaviour::compute_commands(
        FollowedLine line) {

}

// TODO : When the message architecture will be done, modify all this.
CommandOutput CollisionAvoidanceBehaviour::run(
        SystemStateModel &systemStateModel) {
    //Note on simulation
    /*
     * For now i don't know of any place in the code where it specified if the code is
     * in a simulated environement or not. So this variable is temporary.
     */
    Simulation sim = {false, // waypoints
                      true};// obstacles

    //Gives sensors output or compute an easier way to handle them
    sensorOutput = update_sensors(systemStateModel,
                                  sim);

    //update_waypoints(); //Update waypoints or compute an easier way to handle them

    seenObstacles = check_obstacles(sensorOutput);
    //    update_map();
    if (these_obstacles_are_a_problem(seenObstacles)) {
        Eigen::MatrixXd potential_field = compute_potential_field(seenObstacles,
                                                                  sailingZone,
                                                                  followedLine);
        MinPotField min = find_minimum_potential_field(potential_field);
        followedLine = compute_new_path(min);
    }
    return compute_commands(followedLine);
}

//PUBLIC FUNCTIONS

// TODO : replace hardcoded sailing zone by an import from the database
bool CollisionAvoidanceBehaviour::init() {

    //SailingZone initialization clockwise (x,y)
    Eigen::Vector2d GPSpoint0(-1, 1);
    Eigen::Vector2d GPSpoint1(1, 1);
    Eigen::Vector2d GPSpoint2(1, -1);
    Eigen::Vector2d GPSpoint3(-1, -1);
    sailingZone = {GPSpoint0,
                   GPSpoint1,
                   GPSpoint2,
                   GPSpoint3};

}

void CollisionAvoidanceBehaviour::computeCommands(
        SystemStateModel &systemStateModel,
        std::unique_ptr<Position> const &position,
        double trueWindDirection,
        bool mockPosition,
        bool getHeadingFromCompass) {
    CommandOutput out = run(systemStateModel);
    m_rudderCommand = out.deltaRudder;
    m_sailCommand = out.deltaSail;
}





