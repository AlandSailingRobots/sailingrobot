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
// TODO : move to Utility class ?

double CollisionAvoidanceBehaviour::angleDiff(
        double radAngle1,
        double radAngle2) {
    return fmod(radAngle1-radAngle2-M_PI,2*M_PI)+M_PI;
}

double CollisionAvoidanceBehaviour::calculateGPSDistance(
        Eigen::Vector2d point1,
        Eigen::Vector2d point2) {
    //...(1) : latitude //...(0) : longitude
    const double deltaLat = angleDiff(point2(1),point1(1));
    const double deltaLon = angleDiff(point2(0),point1(0));
    const double a  = sin(deltaLat/2)*sin(deltaLat/2)
                      + cos(point1(1))*cos(point2(1))*sin(deltaLon/2)*sin(deltaLon/2);
    const double c = 2*atan2(sqrt(a),sqrt(1-a));
    return EARTH_RADIUS*c;
}

Eigen::Vector2d CollisionAvoidanceBehaviour::findMidPoint(Eigen::Vector2d pt1,
                                                          Eigen::Vector2d pt2){
    const Eigen::Vector3d cartPt1 = latLonToCartesian(pt1);
    const Eigen::Vector3d cartPt2 = latLonToCartesian(pt2);
    const Eigen::Vector3d cartPt3 = (cartPt1+cartPt2)/2;
    return cartesianToLatLon(cartPt3);
}

Eigen::Vector2d CollisionAvoidanceBehaviour::findCenter(
        const std::vector<Eigen::Vector2d> polygon,
        int option) {
    if(option==1){
        const Eigen::Vector2d midPoint = findMidPoint(polygon[0],polygon[3]);
        const double distance = calculateGPSDistance(polygon[0],polygon[3])/2;
        const double bearing = angleDiff(atan2(polygon[3](1)-polygon[0](1),
                                               polygon[3](0)-polygon[0](0)),
                                         M_PI/2);
        const Eigen::Vector2d closeCenter = getPointWithDistanceAndBearing(distance,bearing,midPoint);
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

double CollisionAvoidanceBehaviour::getInitialBearing(
        Eigen::Vector2d pt1,
        Eigen::Vector2d pt2) {
    const double bearingFromNorth = atan2(sin(pt2(0)-pt1(0))*cos(pt2(1)),
                                          cos(pt1(1))*sin(pt2(1))-sin(pt1(1))*cos(pt2(1))
                                                                  *cos(pt2(0)-pt1(0)));
    return M_PI/2-bearingFromNorth;
}

double CollisionAvoidanceBehaviour::getArea(
        std::vector<Eigen::Vector2d> polygon){
    double sumAngle = 0;
    for(int i = 0;i<polygon.size();i++){
        const int iPlusOne = static_cast<int>((i + 1) % polygon.size());
        const int iMinusOne = static_cast<int>((i - 1) % polygon.size());
        const double angleCurrentVertex = angleDiff(getInitialBearing(polygon[i],
                                                                      polygon[iMinusOne]),
                                                    getInitialBearing(polygon[i],
                                                                      polygon[iPlusOne]));
        //std::cout << i << " : " << angleCurrentVertex << "\n";
        sumAngle += angleCurrentVertex;
    }
    //std::cout << sumAngle << "\n";
    return (sumAngle-(polygon.size()-2)*M_PI)*EARTH_RADIUS*EARTH_RADIUS;
}

double CollisionAvoidanceBehaviour::signedDistanceFromLine(
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

Eigen::Vector2d CollisionAvoidanceBehaviour::getClosestVertex(
        std::vector<Eigen::Vector2d> polygon,
        Eigen::Vector2d point){
    int idClosestPoint = -1;
    double minDistance = 40000000;
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

Eigen::Vector2d CollisionAvoidanceBehaviour::getClosestPoint(
        std::vector<Eigen::Vector2d> polygon,
        Eigen::Vector2d point){

    bool computeMidPoint; // Used to lighten the computations.
    int closestPointId0;
    int closestPointId1;
    double min = 40000000; // 40M should be sufficient (~>perimeter of Earth)
    Eigen::Vector2d closestPoint;
    const Eigen::Vector3d worldOrigin(0,0,0);

    for(int i = 0;i<polygon.size();i++){
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

Eigen::Vector2d CollisionAvoidanceBehaviour::getPointWithDistanceAndBearing(
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

std::vector<double> CollisionAvoidanceBehaviour::distanceFromSegment(
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

Eigen::Vector2d CollisionAvoidanceBehaviour::cartesianToLatLon(
        Eigen::Vector3d vector){
    const Eigen::Vector2d gpsCoord(atan2(vector(1),vector(0)),
                                   asin(vector(2)/vector.norm()));
    return gpsCoord;
}

Eigen::Vector3d CollisionAvoidanceBehaviour::latLonToCartesian(
        Eigen::Vector2d vector){
    const Eigen::Vector3d cartesianVector(EARTH_RADIUS*cos(vector(1))*cos(vector(0)),
                                          EARTH_RADIUS*cos(vector(1))*sin(vector(0)),
                                          EARTH_RADIUS*sin(vector(1)));
    return cartesianVector;
}

bool CollisionAvoidanceBehaviour::projectionInsideSlice(
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

boostPolygon CollisionAvoidanceBehaviour::eigenPolyToBoostPoly(
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

Obstacle CollisionAvoidanceBehaviour::updateObstacleWithBoostPoly(
        Obstacle obstacle,
        boostPolygon poly){
    // From clockwise to counterClockwise
    boost::geometry::reverse(poly);

    // The old obstacle is updated without the last point
    obstacle.polygon.clear();
    for (int i = 0 ; i< poly.outer().size()-1; i++){
        const Eigen::Vector2d pt(poly.outer()[i].get<0>(), poly.outer()[i].get<1>());
        obstacle.polygon.push_back(pt);
    }

    // Computes the new center
    obstacle.center = findCenter(obstacle.polygon);

    return obstacle;

}

std::vector<Obstacle> CollisionAvoidanceBehaviour::cleanObstacles(
        SensorData sensorData,
        std::vector<Obstacle> seenObstacles){
    int i = 0;
//    std::cout << "Check_obstacle reached\n";
    std::cout << "Before clean seenObstacles.size() = " << seenObstacles.size() << "\n";
    if (seenObstacles.size() != 0) {
        // for each memorized obstacles
        while (i < seenObstacles.size()) {
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
                const double headingCenterOfDetectedObstacle =
                        angleDiff(sensDatObstacle.leftBoundHeadingRelativeToBoat,
                                  sensDatObstacle.rightBoundHeadingRelativeToBoat)
                        + sensorData.compHeading;
                const double widthOfCurrentDetectedObstacleAtClosest =
                        sensDatObstacle.minDistanceToObstacle
                        * tan(angleDiff(sensDatObstacle.leftBoundHeadingRelativeToBoat,
                                        sensDatObstacle.rightBoundHeadingRelativeToBoat)
                              / 2);

                //Computation of the best center for the detected obstacle
                const double centerDistance = widthOfCurrentDetectedObstacleAtClosest / 2
                                              + sensDatObstacle.minDistanceToObstacle;
                const Eigen::Vector2d centerDetectedObstacle =
                        getPointWithDistanceAndBearing(centerDistance,
                                                       headingCenterOfDetectedObstacle,
                                                       sensorData.gpsPos);

                // 1rst Condition : current obstacle is not too close from the boat
                const bool currentObstaclesIsNotTooClose = calculateGPSDistance(seenObstacles[i].center,
                                                                                centerDetectedObstacle)
                                                           - widthOfCurrentDetectedObstacleAtClosest
                                                           > DISTANCE_NOT_THE_SAME_OBSTACLE;
                std::cout << "reduced dist = " <<  calculateGPSDistance(seenObstacles[i].center,
                                                                        centerDetectedObstacle)
                                                   - widthOfCurrentDetectedObstacleAtClosest << "\n";
                obstaclesAreNotTooCloseWithCurrentMemorizedObstacle =
                        obstaclesAreNotTooCloseWithCurrentMemorizedObstacle
                        && currentObstaclesIsNotTooClose;
            }
            // 2nd condition : the obstacle should have been detected
            const bool obstacleInsideRange =  // Same error given by the IDE
                    calculateGPSDistance(seenObstacles[i].center, sensorData.gpsPos) < MAXIMUM_SENSOR_RANGE;
            const bool obstacleInsideArc =
                    std::abs(angleDiff(
                            atan2(seenObstacles[i].center(1) - sensorData.gpsPos(1),
                                  seenObstacles[i].center(0) - sensorData.gpsPos(0)),
                            sensorData.compHeading + SENSOR_HEADING_RELATIVE_TO_BOAT))
                    < SENSOR_ARC_ANGLE;
            const bool obstacleShouldHaveBeenDetected =
                    (obstacleInsideRange)
                    && (obstacleInsideArc);

            // Where the conditions are used
            std::cout << "Condition 1 obstacleShouldHaveBeenDetected = " << obstacleShouldHaveBeenDetected << "\n";
            std::cout << "Condition 2 obstaclesAreNotTooCloseWithCurrentMemorizedObstacle = " << obstaclesAreNotTooCloseWithCurrentMemorizedObstacle << "\n";
            if (obstacleShouldHaveBeenDetected && obstaclesAreNotTooCloseWithCurrentMemorizedObstacle) {
                // Remove the undetected obstacle from the memory
                seenObstacles.erase(seenObstacles.begin() + i);
            }
            i++;
        }
    }
    return seenObstacles;
}

std::vector<Obstacle> CollisionAvoidanceBehaviour::registerObstacles(
        SensorData sensorData,
        std::vector<Obstacle> seenObstacles){
    std::vector<Obstacle> upToDateObstacles;
    for (auto &sensDatObstacle : sensorData.detectedObstacles) {
        // Currently obstacles are added without merge.
        // polygon is initialized anti-clockwise

        // Init for polygon creation
        Obstacle newObstacle;
        const double halfAngleObsWidth = angleDiff(sensDatObstacle.leftBoundHeadingRelativeToBoat,
                                                   sensDatObstacle.rightBoundHeadingRelativeToBoat) / 2;
        const double rightAngle = sensDatObstacle.rightBoundHeadingRelativeToBoat
                                  + sensorData.compHeading
                                  + SENSOR_HEADING_RELATIVE_TO_BOAT;
        const double leftAngle = sensDatObstacle.leftBoundHeadingRelativeToBoat
                                 + sensorData.compHeading
                                 + SENSOR_HEADING_RELATIVE_TO_BOAT;
        const double minDistance = sensDatObstacle.minDistanceToObstacle
                                   / cos(halfAngleObsWidth);
        const double maxDistance = sensDatObstacle.maxDistanceToObstacle
                                   / cos(halfAngleObsWidth);
//        std::cout << "cosAngleFromObsWidth = " << cosAngleFromObsWidth << "\n";
//        std::cout << "sensDatObstacle.maxDistanceToObstacle * cosAngleFromObsWidth = " << maxDistance << "\n";

        // Polygon creation
        const Eigen::Vector2d pt1 = getPointWithDistanceAndBearing(minDistance, rightAngle,
                                                                   sensorData.gpsPos);
        newObstacle.polygon.push_back(pt1);

        const Eigen::Vector2d pt2 = getPointWithDistanceAndBearing(maxDistance, rightAngle,
                                                                   sensorData.gpsPos);
        newObstacle.polygon.push_back(pt2);

        const Eigen::Vector2d pt3 = getPointWithDistanceAndBearing(maxDistance, leftAngle,
                                                                   sensorData.gpsPos);
        newObstacle.polygon.push_back(pt3);

        const Eigen::Vector2d pt4 = getPointWithDistanceAndBearing(minDistance, leftAngle,
                                                                   sensorData.gpsPos);
        newObstacle.polygon.push_back(pt4);

        //Computation of the best center (longer but more accurate than the other)
        const double centerDistance = calculateGPSDistance(pt1, pt4) / 2;
        const double centerAngle = angleDiff(atan2(pt4(1) - pt1(1), pt4(0) - pt1(0)),
                                             M_PI / 2);

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
    return upToDateObstacles;
}

std::vector<Obstacle> CollisionAvoidanceBehaviour::mergeObstacles(
        std::vector<Obstacle> upToDateObstacles,
        std::vector<Obstacle> seenObstacles){

    int i2 = 0;
    if (upToDateObstacles.size() != 0) {
        while (i2 < upToDateObstacles.size()) {
            std::cout << "Got into for each up to date obstacle\n";
            // Init
            bool deletion = false;

            // Creation of the boost polygon
            boostPolygon poly = eigenPolyToBoostPoly(upToDateObstacles[i2]);

            for (auto &otherObstacle : seenObstacles) {

                std::cout << "Verification of up to date " << otherObstacle.upToDate << "\n";
                std::cout << "Got into for each other obstacle\n";
                // Modify the old obstacle if there is an intersection with an upToDateObstacle

                // TODO : investigate if we should modify everything to use boost c++ lib ?
                // TODO : investigate to make the intersection code more efficient.

                // Creation of the boost polygon
                boostPolygon poly2 = eigenPolyToBoostPoly(otherObstacle);

                // Intersection
                std::vector<boostPolygon> output;
                boost::geometry::intersection(poly, poly2, output);

                if (!output.empty()) {
                    otherObstacle = updateObstacleWithBoostPoly(otherObstacle, output[0]);

                    // The detected obstacle is set to be deleted
                    deletion = true;
                }
            }
            // Delete the up to date obstacle if it has been used to create an intersection
            if (deletion) {
                upToDateObstacles.erase(upToDateObstacles.begin() + i2);
            }
            i2++;
        }

    }

    // put back the up to date obstacles and the old ones into seen_obstacles
    for(auto & upToDateObs : upToDateObstacles){
        seenObstacles.push_back(upToDateObs);
    }

    return seenObstacles;
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
        //The latitude and longitude are easier to compute in radians.
        sensorData.gpsPos(0) = // x
                Utility::degreeToRadian(systemStateModel.gpsModel.positionModel.longitude);
        sensorData.gpsPos(1) = // y
                Utility::degreeToRadian(systemStateModel.gpsModel.positionModel.latitude);
        sensorData.gpsSpeed = systemStateModel.gpsModel.speed;

        //Heading
        //compHeading : degree from north -> radian from east
        sensorData.compHeading = -Utility::degreeToRadian(systemStateModel.compassModel.heading)
                                 + M_PI / 2;
        //gpsHeading : degree from north -> radian from east
        sensorData.gpsHeading = -Utility::degreeToRadian(systemStateModel.gpsModel.heading)
                                + M_PI / 2;

        //Wind
        //windDirection : degree from east -> radian from east
        sensorData.windDirection = -Utility::degreeToRadian(systemStateModel.windsensorModel.direction)
                                   ;
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

        //Obstacle settings
        const double obstacleRadius = 5; //meters
        std::vector<Eigen::Vector2d> obsVec;
        const Eigen::Vector2d obs0(0+(-5)*CONVERSION_FACTOR_METER_TO_GPS,
                                   0+(50)*CONVERSION_FACTOR_METER_TO_GPS); obsVec.push_back(obs0);
        const Eigen::Vector2d obs1(1+(0)*CONVERSION_FACTOR_METER_TO_GPS,
                                   1+(-20)*CONVERSION_FACTOR_METER_TO_GPS); obsVec.push_back(obs1);
        const Eigen::Vector2d obs2(1+(-20)*CONVERSION_FACTOR_METER_TO_GPS,
                                   1+(0)*CONVERSION_FACTOR_METER_TO_GPS); obsVec.push_back(obs2);
        const Eigen::Vector2d obs3(1+(0)*CONVERSION_FACTOR_METER_TO_GPS,
                                   1+(0)*CONVERSION_FACTOR_METER_TO_GPS); obsVec.push_back(obs3);

        //Sensor settings
        const double maxDetectionRange = 1000;

        //Simulation
        // TODO : implement more precisely obstacle simulation.
        for(auto & obs : obsVec){
            const double obstacleHeadingRelativeToBoat = angleDiff(atan2(obs(1)-sensorData.gpsPos(1),
                                                                         obs(0)-sensorData.gpsPos(0)),
                                                                   sensorData.compHeading);
            //std::cout << "obstacleHeadingRelativeToBoat = " << obstacleHeadingRelativeToBoat<< "\n";
            const double distFromObstacle = calculateGPSDistance(sensorData.gpsPos,
                                                                 obs);
            if(std::abs(obstacleHeadingRelativeToBoat)<=M_PI/4 && distFromObstacle<=maxDetectionRange) {

                const double leftHeadingRelativeToBoat =  atan2(obstacleRadius, distFromObstacle)
                                                          +obstacleHeadingRelativeToBoat;
                const double rightHeadingRelativeToBoat = -atan2(obstacleRadius, distFromObstacle)
                                                          +obstacleHeadingRelativeToBoat;
                ObstacleData obstacle = {
                        distFromObstacle - obstacleRadius, //double minDistanceToObstacle;
                        maxDetectionRange,                 //double maxDistanceToObstacle;
                        leftHeadingRelativeToBoat,         //double LeftBoundheadingRelativeToBoat;
                        rightHeadingRelativeToBoat};       //double RightBoundheadingRelativeToBoat;
                //std::cout << "I push back\n";
                sensorData.detectedObstacles.push_back(obstacle);
            }
        }
    }
    else {
        // TODO : when the sensors wil be ready, put the code to get everything here
    }

    return sensorData;
}

/*
 * Update waypoints or compute an easier way to handle them.
 * Update m_followedLine ?
 */
//    void update_waypoints(){}

std::vector<Obstacle> CollisionAvoidanceBehaviour::check_obstacles(SensorData sensorData,
                                                                   std::vector<Obstacle> seenObstacles) {
    // clean the obstacles thar should have been detected and are not
    seenObstacles = cleanObstacles(sensorData,seenObstacles);

    std::cout << "After clean seenObstacles.size() = " << seenObstacles.size() << "\n";

    // register the new obstacles inside upToDateObstacles (convert from sensorData)
    std::vector<Obstacle> upToDateObstacles = registerObstacles(sensorData, seenObstacles);

    std::cout << "After registering upToDateObstacles.size() = " << upToDateObstacles.size() << "\n";
    std::cout << "Just for info : seenObstacles.size() = " << seenObstacles.size() << "\n";

    // for each up to date obstacle we clean the areas where nothing has been detected
    seenObstacles = mergeObstacles(upToDateObstacles,seenObstacles);

    return seenObstacles;
}

/*
 *
 */
//map update_map(){}

bool CollisionAvoidanceBehaviour::these_obstacles_are_a_problem(
        std::vector<Obstacle> m_seenObstacles) { // OUTPUT if these obstacles are a problem
    bool theseObstaclesAreAProblem = false;
    // We have polygons. We need to see if there is an intersection between
    // the channel and the polygons. That is to say : one of the points of the
    // polygon is inside the channel

    for(auto & obstacle : m_seenObstacles){
        for(auto & point : obstacle.polygon){
            // compute the distance between the line and the point.
            double signedDistance = signedDistanceFromLine(m_followedLine.startPoint,
                                                           m_followedLine.endPoint,
                                                           point);
            bool thisObstacleIsAProblem = false;
            if(std::abs(signedDistance)< CHANNEL_RADIUS
               && calculateGPSDistance(m_sensorOutput.gpsPos,
                                       getClosestPoint(obstacle.polygon,
                                                       m_sensorOutput.gpsPos) ) < SAFE_DISTANCE){

                thisObstacleIsAProblem = true;
            }
            theseObstaclesAreAProblem = theseObstaclesAreAProblem || thisObstacleIsAProblem;
        }
    }
    return theseObstaclesAreAProblem;
}

PotentialMap CollisionAvoidanceBehaviour::compute_potential_field(
        std::vector<Obstacle> m_seenObstacles,
        std::vector<Eigen::Vector2d> m_sailingZone,
        FollowedLine) {
    // TODO : add this code
    /*
    else if(only_direction_mode==1){
        if( have_to_avoid_obstacle == 1){
             isObsPEmpty = 0;
             bearingObstacle=boat_state(0,2)+direction_boat_obstacle[0]-M_PI/2;
             x1 = point_x.array()-boat_state(0,0);
             y1 = point_y.array()-boat_state(0,1);
             xb =  x1*cos(bearingObstacle) + y1*sin(bearingObstacle);
             yb = -x1*sin(bearingObstacle) + y1*cos(bearingObstacle);
             ObsP = 15*rectangularPulse(-lengthObstacle,lengthObstacle,xb).array() * heavisideMat(yb).array()-5*rectangularPulse(-lengthObstacle*2,lengthObstacle*2,xb).array() *  heavisideMat(yb).array();
        }
    }
     */


    // Init

    // Set up of the matrix of 100*100 around the boat.
    //TODO : get these values from the database/elsewhere ?
    const int matrixHeight   = 100; const int matrixWidth    = 100;
    const int yUpperMapBound = 100; const int xUpperMapBound = 100; // in meters
    const int yLowerMapBound = 100; const int xLowerMapBound = 100; // in meters

    // Since the distance computed here is only on one axis,
    // a simple conversion factor is sufficient
    PotentialMap PotField = {
            m_sensorOutput.gpsPos(1)
             - (xLowerMapBound * CONVERSION_FACTOR_METER_TO_GPS / 180 * M_PI),
            m_sensorOutput.gpsPos(1)
             +(xUpperMapBound*CONVERSION_FACTOR_METER_TO_GPS/180*M_PI),
            m_sensorOutput.gpsPos(1)
             -(yLowerMapBound*CONVERSION_FACTOR_METER_TO_GPS/180*M_PI),
            m_sensorOutput.gpsPos(1)
             +(yUpperMapBound*CONVERSION_FACTOR_METER_TO_GPS/180*M_PI),
            Eigen::ArrayXXd::Zero(matrixHeight, matrixWidth)
    };

    const Eigen::ArrayXXd Px = Eigen::RowVectorXd::LinSpaced(matrixWidth,
                                                             PotField.xMin,
                                                             PotField.xMax).replicate(matrixHeight,1);
    const Eigen::ArrayXXd Py = Eigen::VectorXd::LinSpaced(   matrixHeight,
                                                             PotField.yMin,
                                                             PotField.yMax).replicate(1, matrixWidth);
    // On a distance of 150m there is an error of approximatively 2 meters, so a linear conversion from
    // GPS coordinates to meters is possible at this scale

    // Obstacle potential function
    /*
     * Since the obstacle is a polygon, only the closest point from
     * the polygon will be counted as an obstacle
     *
     * Maybe we should do a parameter file where we could configure everything
     * Values found after extensive testing on matlab
     * The values are scaled for meters.
     */
    // TODO : conversion to gps coordinate                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      s
    // TODO : modify obstacle according to its width. ?
    const double scaleHole = 50;
    const double scalePike = 550;
    const double scale = 0.5;
    const double strengthHoles = 2;
    const double strengthPike = 4;
    const double strength = 5;
    const double offsetObstacle = 15;

    Eigen::ArrayXXd obsPot = Eigen::ArrayXXd::Zero(matrixHeight,matrixWidth);
    std::vector<Eigen::Vector2d> closestObstaclesPoints;

    for(auto & obstacle : m_seenObstacles){
        Eigen::Vector2d closestObstaclePoint = getClosestPoint(obstacle.polygon,
                                                               m_sensorOutput.gpsPos);
        closestObstaclesPoints.push_back(closestObstaclePoint);
    }

    // Add the pikes
    for(auto & obstacle : closestObstaclesPoints){ //for each seen_obstacle add the pikes

        const Eigen::ArrayXXd xObs = Px-obstacle(0);
        const Eigen::ArrayXXd yObs = Px-obstacle(0);
        const Eigen::ArrayXXd tPike = ( (xObs*1).square()
                                       +(yObs-offsetObstacle*scale).square())
                                      /scalePike/scale;

        obsPot = obsPot.max(strength*strengthPike*(-(tPike.square())).exp());
    }

    // Add the holes
    for(auto & obstacle : closestObstaclesPoints){ //for each seen_obstacle add the holes
        const double obstacleHeading = fmod(m_sensorOutput.compHeading,2*M_PI)*0.3
                                       + fmod( atan2(m_followedLine.endPoint(1)-obstacle(1),
                                                     m_followedLine.endPoint(1)-obstacle(0)),
                                               2*M_PI)*0.7
                                       + M_PI/2;
        const Eigen::ArrayXXd xObsBeforeRot = Px-obstacle(0);
        const Eigen::ArrayXXd yObsBeforeRot = Px-obstacle(0);
        const Eigen::ArrayXXd xObsAfterRot =   xObsBeforeRot * cos(obstacleHeading)
                                             + yObsBeforeRot * sin(obstacleHeading);
        const Eigen::ArrayXXd yObsAfterRot = - xObsBeforeRot * sin(obstacleHeading)
                                             + yObsBeforeRot * cos(obstacleHeading);
        const Eigen::ArrayXXd tRightHole = ((xObsAfterRot-35*scale).square()
                                            +(yObsAfterRot-offsetObstacle*scale).square())
                                           /scaleHole/scale;
        const Eigen::ArrayXXd tLeftHole  = ((xObsAfterRot+35*scale).square()
                                            +(yObsAfterRot-offsetObstacle*scale).square())
                                           /scaleHole/scale;

        obsPot = obsPot
                 - strength*strengthHoles*(-(tRightHole).square()).exp()
                 - strength*strengthHoles*(-(tLeftHole).square()).exp();
    }

    // Objective potential function
        // Values found after extensive testing
    const double objectiveSpread = 25000;
    const double objectiveStrength = 1.6;
    const Eigen::ArrayXXd objPot = objectiveStrength*((- (Px-m_followedLine.endPoint(0)).square()
                                                       - (Py-m_followedLine.endPoint(1)).square())
                                                      /objectiveSpread).exp();

    // Windpreference potential function
    const double noGoZoneAngle = 3*M_PI/4.0;

    const Eigen::ArrayXXd xWindBeforeRot = Px-m_sensorOutput.gpsPos(0);
    const Eigen::ArrayXXd yWindBeforeRot = Py-m_sensorOutput.gpsPos(1);
    const double noGoZoneHeading = m_sensorOutput.windDirection + noGoZoneAngle;
    const Eigen::ArrayXXd xWindAfterRot =   xWindBeforeRot*cos(noGoZoneHeading)
                                          + yWindBeforeRot*sin(noGoZoneAngle);
    const Eigen::ArrayXXd yWindAfterRot = - xWindBeforeRot*sin(noGoZoneHeading)
                                          + yWindBeforeRot*cos(noGoZoneHeading);

        // Change from atan to 1/(1+exp(-2*t))
    const double windChangeSlope = 10; // Higher it is the slopiest it is
    const Eigen::ArrayXXd windPot = 3 * (1/(1+(-windChangeSlope*xWindAfterRot).exp()))
                                      * (1/(1+(-windChangeSlope*yWindAfterRot).exp()));

    // Boat preference
    const double strengthBoat = 3;
    const double strengthHoleBoat = 1.5;
    const double spreadHoleBoat = 4000;
    const double strengthPikeBoat = 5;
    const double spreadPikeBoat = 200;

    const Eigen::ArrayXXd boatHolePot = strengthHoleBoat
                                        * ((-(Px-m_sensorOutput.gpsPos(0)).square()
                                            -(Py-m_sensorOutput.gpsPos(1)).square())
                                           / spreadHoleBoat).exp();
    const Eigen::ArrayXXd boatPikePot = strengthPikeBoat
                                        * ((-(Px-m_sensorOutput.gpsPos(0)).square()
                                            -(Py-m_sensorOutput.gpsPos(1)).square())
                                           / spreadPikeBoat).exp();
    const Eigen::ArrayXXd boatPot = strengthBoat*( -boatHolePot + boatPikePot );

    // Sailing Zone
    Eigen::ArrayXXd sailingZoneMatrix = Eigen::ArrayXXd::Zero(matrixHeight,matrixWidth);//New blank matrix
    for(int i = 0;i<m_sailingZone.size();i++){
        // The lines of the polygon are added incrementally

        const int j = fmod(i+1,m_sailingZone.size()); // index of the next point
        const Eigen::Vector2d linePt0 = m_sailingZone[i];
        const Eigen::Vector2d linePt1 = m_sailingZone[j];
        const Eigen::Vector2d origin(0,0);

        // Distance between the origin and the line (AB)
        // I'm not sure using gsp distance here is the best idea
        // const double offsetFromOrigin = signedDistanceFromLine(linePt0,linePt1,origin);
        Eigen::Matrix2d mat;
        mat << (linePt1-linePt0)/(linePt1-linePt0).norm(),(origin-linePt0);
        const double offsetFromOrigin = mat.determinant();
        const double lineHeading = atan2(linePt1(1)-linePt0(1),
                                         linePt1(0)-linePt0(0)); // Heading to the line (AB)

        // const Eigen::ArrayXXd xLine = (Px)*cos(lineHeading)+(Py)*sin(lineHeading); // saved in a comment
        const Eigen::ArrayXXd yLine = (Px)*cos(lineHeading+M_PI/2)+(Py)*sin(lineHeading+M_PI/2);
        // On one side of the line there is 1, 0 on the other
        // offsetFromOrigin is used to move the line from the origin

        const double sailingZoneChangeSlope = 10;
        const Eigen::ArrayXXd line = 1 - (1 / ( 1 + ( - sailingZoneChangeSlope
                                                      *(  yLine
                                                        + offsetFromOrigin)
                                                    ).exp()
                                              ));

        sailingZoneMatrix = sailingZoneMatrix*line;

        sailingZoneMatrix = 1-sailingZoneMatrix;
    }
    const double strengthSailingZone = 10;

    const Eigen::ArrayXXd sailZonePot = strengthSailingZone*sailingZoneMatrix;

    // Computation of the field
    PotField.field = obsPot - objPot + boatPot + windPot + sailZonePot;
    return PotField;
}

Eigen::Vector2d CollisionAvoidanceBehaviour::find_minimum_potential_field(
        PotentialMap potField) {
    double I_row = 0, I_col = 0;
    const double minZ = potField.field.minCoeff(&I_row, &I_col);
    double I_x = I_col*(potField.xMax-potField.xMin)
                 /potField.field.cols()-(abs(potField.yMin));
    double I_y = I_row*(potField.yMax-potField.yMin)
                 /potField.field.rows()-(abs(potField.xMin));
    const Eigen::Vector2d collision_avoidance_point(I_x,I_y);
    return collision_avoidance_point;
}

FollowedLine CollisionAvoidanceBehaviour::compute_new_path(
        Eigen::Vector2d collision_avoidance_point) {
    const double avoidDist = 30;

    // Initialize the 3 wps
        // Behind the obstacle, not sure that using the heading of the boat is the best idea
        // TODO some test on issue above (using heading of the boat in startCollPoint)
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
    // TODO add 3 wp to the database

    // Set m_followedLine accordingly
    m_followedLine.startPoint = startCollPoint;
    m_followedLine.endPoint = collision_avoidance_point;

    //The waypointNode gives which wps to follow.
    return m_followedLine; //return is not necessary if m_followedLine is still a class variable. Delete it otherwise
}

CommandOutput CollisionAvoidanceBehaviour::compute_commands(
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
    const double ngzAngleBack = 0;          // rad   back wind angle
    // ngzAngleOUT = ngzAngle+pi/8;         // rad   out of the no-go zone angle
    // NGZHeading = mod(psi+pi,2*pi);       // rad   no-go zone angle

    // Step 3 : compute lineHeading
    double lineHeading = atan2(line.endPoint(1) - line.startPoint(1),
                               line.endPoint(0) - line.startPoint(0));
    if (std::abs(angleDiff(atan2(m_sensorOutput.gpsPos(1) - line.endPoint(1),
                                 m_sensorOutput.gpsPos(0) - line.endPoint(0)), lineHeading))
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
    m_sensorOutput = update_sensors(systemStateModel,
                                  sim);

    //update_waypoints(); //Update waypoints or compute an easier way to handle them

    m_seenObstacles = check_obstacles(m_sensorOutput);
    //    update_map();
    if (these_obstacles_are_a_problem(m_seenObstacles)) {
        PotentialMap potential_field = compute_potential_field(m_seenObstacles,
                                                               m_sailingZone,
                                                               m_followedLine);
        Eigen::Vector2d min = find_minimum_potential_field(potential_field);
        m_followedLine = compute_new_path(min);
    }
    return compute_commands(m_followedLine);
}

//PUBLIC FUNCTIONS (TRASH)

// TODO : replace hardcoded sailing zone by an import from the database
bool CollisionAvoidanceBehaviour::init() {

    //SailingZone initialization clockwise (x,y)
    Eigen::Vector2d GPSpoint0(-1, 1); Eigen::Vector2d GPSpoint3( 1, 1);
    Eigen::Vector2d GPSpoint1(-1,-1); Eigen::Vector2d GPSpoint2( 1,-1);
    m_sailingZone = {GPSpoint0, GPSpoint1, GPSpoint2, GPSpoint3};

}

void CollisionAvoidanceBehaviour::computeCommands(
        SystemStateModel &systemStateModel,
        std::unique_ptr<Position> const &position,
        double trueWindDirection,
        bool mockPosition,
        bool getHeadingFromCompass) {
    CommandOutput out = run(systemStateModel);
    m_rudderCommand = out.deltaRudder;
    m_sailCommand   = out.deltaSail;
}






