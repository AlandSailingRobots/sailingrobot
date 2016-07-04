#include "LineFollowBehave.h"


LineFollowBehave::LineFollowBehave(DBHandler *db):
  BehavingClass(db),//super class call
  m_previousWaypointModel(PositionModel(1.5,2.7), 100, "", 6),
  m_nextWaypointModel(PositionModel(1.5,2.7), 100, "", 6)
{
    m_wayPointCount = 0;
    m_tackingDirection = 1;
    m_maxCommandAngle = M_PI / 6;
    m_maxSailAngle = M_PI / 32;
    m_minSailAngle = M_PI / 5.2f;
    m_tackAngle = M_PI / 3;
    desiredHeadingTackMode = 0;
}

bool LineFollowBehave::init()
{
    printf("Entered init...\n");                                                          
    m_wayPointCount = 0;                                                                 //|   If changed back to LineFollow during run, 
    m_previousWaypointModel.id = "";                                                     //|   reset values.

    printf("Setting up next Waypoint\n");
	setNextWaypoint();
    printf("Setting up: OK\n");

    printf("*SailingRobot::run() LineFollow started.\n");
    std::cout << "Waypoint target - ID: " << m_nextWaypointModel.id << " lon: " <<
    m_nextWaypointModel.positionModel.longitude	<< " lat : " <<
    m_nextWaypointModel.positionModel.latitude << std::endl;

    return true;
}

double LineFollowBehave::getSailCommand(){
  return m_sailCommand;
}

double LineFollowBehave::getRudderCommand(){
  return m_rudderCommand;
}

double LineFollowBehave::calculateSignedDistance(std::unique_ptr<Position> const& position)
{

    int earthRadius = 6371000;
    std::array<double, 3> a =
     {  earthRadius * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(sin(m_previousWaypointModel.positionModel.latitude))};
    std::array<double, 3> b =
     {  earthRadius * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(sin(m_nextWaypointModel.positionModel.latitude))};
    std::array<double, 3> m = 
    {   earthRadius * Utility::degreeToRadian(cos(position->getModel().latitude)) * Utility::degreeToRadian(cos(position->getModel().longitude)),
        earthRadius * Utility::degreeToRadian(cos(position->getModel().latitude)) * Utility::degreeToRadian(cos(position->getModel().longitude)),
        earthRadius * Utility::degreeToRadian(sin(position->getModel().latitude))};
                // a^b / ||a|| * ||b||
    double n = ( (a[1]*b[2] - a[2]*b[1]) + (a[2]*b[0] - a[0]*b[2]) + (a[0]*b[2] - a[1]*b[0]) ) /   //Vector product: A^B divided by
                ( sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])   *   sqrt(b[0]*b[0] + b[1]*b[1] + b[2]*b[2])); // norm a * norm b

    double e = m[0]*n + m[1]*n + m[2]*n; //dot product       

    return e;
}

double LineFollowBehave::calculateAngleOfDesiredTrajectory(std::unique_ptr<Position> const& position)
{
    int earthRadius = 6371000;
    std::array<double, 3> a =
     {  earthRadius * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_previousWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(sin(m_previousWaypointModel.positionModel.latitude))};
    std::array<double, 3> b =
     {  earthRadius * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.latitude)) * Utility::degreeToRadian(cos(m_nextWaypointModel.positionModel.longitude)),
        earthRadius * Utility::degreeToRadian(sin(m_nextWaypointModel.positionModel.latitude))};

    double M[2][3] = 
    {   {Utility::degreeToRadian(-sin(position->getModel().longitude)), Utility::degreeToRadian(cos(position->getModel().longitude)), 0},
        {Utility::degreeToRadian(-cos(position->getModel().longitude))*Utility::degreeToRadian(sin(position->getModel().latitude)),
         Utility::degreeToRadian(-sin(position->getModel().longitude))*Utility::degreeToRadian(sin(position->getModel().latitude)), 
         Utility::degreeToRadian(cos(position->getModel().latitude))}};

    std::array<double, 3> bMinusA = { b[0]-a[0], b[1]-a[1], b[2]-a[2]};

                        // 2x3 * 1x3
    double phi = atan2(M[0][0]*bMinusA[0] + M[0][1]*bMinusA[1] + M[0][2]*bMinusA[2],   M[1][0]*bMinusA[0] + M[1][1]*bMinusA[1] + M[1][2]*bMinusA[2]);

    return phi;
}

bool LineFollowBehave::computeCommands(SystemStateModel &m_systemStateModel,std::unique_ptr<Position> const& position,
                                    std::vector<float> &twdBuffer,
                                    const unsigned int twdBufferMaxSize,bool m_mockPosition,bool m_getHeadingFromCompass){

    if(m_previousWaypointModel.id == "")
        setPreviousWayPoint(m_systemStateModel);
    
    printf("next waypoint ID: "); printf(m_nextWaypointModel.id.c_str()); 
    printf("     next waypoint lat: "); printf(std::to_string(m_nextWaypointModel.positionModel.longitude).c_str()); 
    printf("     next waypoint long: "); printf(std::to_string(m_nextWaypointModel.positionModel.latitude).c_str());
    printf("\n");
    printf("previous waypoint ID: "); printf(m_previousWaypointModel.id.c_str()); 
    printf("     previous waypoint lat: "); printf(std::to_string(m_previousWaypointModel.positionModel.longitude).c_str()); 
    printf("     previous waypoint long: "); printf(std::to_string(m_previousWaypointModel.positionModel.latitude).c_str());
    printf("\n");

    position->updatePosition();
    double trueWindDirection = Utility::meanOfAngles(twdBuffer);
    bearingToNextWaypoint = m_courseMath.calculateBTW(position->getModel(), m_nextWaypointModel.positionModel); //calculated for database
    distanceToNextWaypoint = m_courseMath.calculateDTW(position->getModel(), m_nextWaypointModel.positionModel);

    if(distanceToNextWaypoint < m_nextWaypointModel.radius)
    {
        //When arrive at next waypoint
        std::cout << "Waypoint reached!" << std::endl;
        try {
		    m_dbHandler->changeOneValue("waypoints", m_nextWaypointModel.id,"1","harvested");
	    } catch (const char * error) {
		    m_logger.error(error);
	    }
        m_wayPointCount++;
        setPreviousWayPoint(m_systemStateModel);
        setNextWaypoint();
    }

    //GOTTA CHECK IF GPS IS ONLINE
    if (m_systemStateModel.gpsModel.online) {
        //GET DIRECTION - From the book Robotic Sailing 2012 and Robotic Sailing 2015
        double currentHeading = getHeading(m_systemStateModel,m_mockPosition,m_getHeadingFromCompass,position);
        double signedDistance = calculateSignedDistance(position); // 'e'
        int maxTackDistance = 10; //'r'
        double phi = calculateAngleOfDesiredTrajectory(position);
        desiredHeading = phi - (2 * (M_PI / 4)/M_PI) * atan2(signedDistance,maxTackDistance);

        //CHECK IF TACKING IS NEEDED line 139 - 161.
        if(abs(signedDistance) > maxTackDistance)
        {
            m_tackingDirection = Utility::sgn(signedDistance);
            m_tack = false;
        }

        //To stop the boat from oscillating during tack, we set the boat to tack from one maxTackDistance of the line to the other, not letting it switch direction in between.
        double desiredHeading2 = desiredHeading; //Save desiredHeading into a temp variable so we don't have faulty values for the if-statements
        if(cos(trueWindDirection - desiredHeading) + cos(m_tackAngle + M_PI/8) > 0)
            m_tack = false;
        else if(m_tack)
            desiredHeading2 = desiredHeadingTackMode;
        else if(cos(trueWindDirection - desiredHeading) + cos(m_tackAngle) < 0)
        {
            m_tack = true;
            desiredHeading2 = M_PI + trueWindDirection - m_tackAngle * m_tackingDirection;
            desiredHeadingTackMode = desiredHeading2;
        }

        if(abs(signedDistance) > maxTackDistance)
            desiredHeading2 = M_PI + trueWindDirection - m_tackAngle * m_tackingDirection;
        desiredHeading = desiredHeading2; //set the value back from the temp variable


        //SET RUDDER
        if(cos(currentHeading - desiredHeading) < 0) //if boat is going the wrong direction
            m_rudderAngle = Utility::sgn(m_systemStateModel.gpsModel.speed) * m_maxCommandAngle * Utility::sgn(sin(currentHeading - desiredHeading));
        else                                
            m_rudderAngle = Utility::sgn(m_systemStateModel.gpsModel.speed) * m_maxCommandAngle * sin(currentHeading - desiredHeading);
        //m_rudderCommand = m_commandHandler.rudderCommand(m_rudderAngle, desiredHeading, m_maxCommandAngle); //Unsure about how this should be...
        m_rudderCommand = m_rudderAngle; //Is the angle already perfectly fixed or should I do as the line above??!?! 

        //SET SAIL                          //Calculate apparent wind from true wind.
        std::array<double, 2> wcaw = { m_systemStateModel.windsensorModel.speed * cos(trueWindDirection - currentHeading) - m_systemStateModel.gpsModel.speed, 
                                        m_systemStateModel.windsensorModel.speed * sin(trueWindDirection - currentHeading)}; 
        double apparentWindAngle = atan2(wcaw[0], wcaw[1]); 

        m_sailAngle = -Utility::sgn(apparentWindAngle) * ( ((m_minSailAngle - m_maxSailAngle) / M_PI) * abs(apparentWindAngle) + m_maxSailAngle);
        //m_sailCommand = m_commandHandler.sailCommand(m_sailAngle); 
        m_sailCommand = m_sailAngle;

        printf("SailAngle: "); printf(std::to_string(m_sailAngle).c_str()); printf("    : rudderAngle: "); printf(std::to_string(m_rudderAngle).c_str()); printf("\n");

    } else {
        m_logger.error("SailingRobot::run(), gps NaN. Using values from last iteration.\n");
        printf("GPS not online\n");
    }
    
    return true;
}


void LineFollowBehave::manageDatabase(std::vector<float> &twdBuffer,SystemStateModel &m_systemStateModel){
  //logging
  bool routeStarted = false;
  m_dbHandler->insertDataLog(
    m_systemStateModel,
    0,
    0,                                               
    distanceToNextWaypoint,
    bearingToNextWaypoint,
    desiredHeading,
    m_tack,
    getGoingStarboard(),
    atoi(m_nextWaypointModel.id.c_str()),
    Utility::meanOfAngles(twdBuffer),
    routeStarted
  );
}

void LineFollowBehave::setNextWaypoint(){
	try {
		m_dbHandler->getWaypointFromTable(m_nextWaypointModel);
	} catch (const char * error) {
		m_logger.error(error);
	}
	try {
		if (m_nextWaypointModel.id.empty() ) {
			std::cout << "No waypoint found!"<< std::endl;
			throw "No waypoint found!";
		}
		else{
			std::cout << "New waypoint picked! ID:" << m_nextWaypointModel.id <<" lat: "
			<< m_nextWaypointModel.positionModel.latitude
			<< " lon: " << m_nextWaypointModel.positionModel.longitude << " rad: "
			<< m_nextWaypointModel.radius << std::endl;
		}
	} catch (const char * error) {
		m_logger.error(error);
	}

	m_logger.info("setupWaypoint() done");
}

void LineFollowBehave::setPreviousWayPoint(SystemStateModel &m_systemStateModel)
{
    if(m_wayPointCount == 0) //if no waypoints have been passed yet
    {
        //Check list if any waypoints have been passed earlier (incase hardreset has been made earlier, resulting in wayPointCount resetting)
        WaypointModel waypointModel = m_dbHandler->getPreviouslyHarvestedWaypoint();
        
        if(waypointModel.id == ""){//If no waypoints had been harvested, set previouspoint to boats startingposition
            m_previousWaypointModel.positionModel.longitude = m_systemStateModel.gpsModel.positionModel.longitude;
            m_previousWaypointModel.positionModel.latitude = m_systemStateModel.gpsModel.positionModel.latitude;
        }
        else 
            m_previousWaypointModel = waypointModel;
    }
    else if(m_wayPointCount > 0) //if waypoints passed, set previous waypoint to the one recently passed
        m_previousWaypointModel = m_nextWaypointModel;
}

bool LineFollowBehave::getGoingStarboard()
{
    if(m_tackingDirection == 1) return true;
    else return false;
}

int LineFollowBehave::getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position) {

	int useGpsForHeadingKnotSpeed = 1;
	bool acceptedCompassSpeed = Utility::directionAdjustedSpeed(m_systemStateModel.gpsModel.heading, m_systemStateModel.compassModel.heading, m_systemStateModel.gpsModel.speed) < useGpsForHeadingKnotSpeed;

    if(m_mockPosition) {
        return position->getHeading();
    }

    //only accept getHeadingFromCompass if speed less than 1
	if (m_getHeadingFromCompass && acceptedCompassSpeed) {
    	return Utility::addDeclinationToHeading(m_systemStateModel.compassModel.heading, m_nextWaypointModel.declination);
	}
    return m_systemStateModel.gpsModel.heading;
}