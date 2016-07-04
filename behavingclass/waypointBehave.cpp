#include "waypointBehave.h"


waypointBehave::waypointBehave(DBHandler *db):
  BehavingClass(db),//super class call
  m_waypointModel(PositionModel(1.5,2.7), 100, "", 6),

  m_waypointRouting(m_waypointModel,
    atof(db->retrieveCell("waypoint_routing_config", "1", "radius_ratio").c_str()),
    atof(db->retrieveCell("course_calculation_config", "1", "tack_angle").c_str()),
    atof(db->retrieveCell("course_calculation_config", "1", "tack_max_angle").c_str()),
    atof(db->retrieveCell("course_calculation_config", "1", "tack_min_speed").c_str()),
    atof(db->retrieveCell("course_calculation_config", "1", "sector_angle").c_str()),
     atof(db->retrieveCell("waypoint_routing_config", "1", "max_command_angle ").c_str()),
     atof(db->retrieveCell("waypoint_routing_config", "1", "rudder_speed_min").c_str())
   ),
  insertScanOnce(0)
{
  m_waypointRouting.setUpdateInterval(
    atof(db->retrieveCell("waypoint_routing_config", "1", "sail_adjust_time").c_str()));

  m_waypointRouting.setMinimumDegreeLimit(
    atof(db->retrieveCell("waypoint_routing_config", "1", "adjust_degree_limit").c_str()));
}

bool waypointBehave::init(){

  printf(" Starting Waypoint\t\t");
	setupWaypoint();
	printf("OK\n");

	m_waypointRouting.setWaypoint(m_waypointModel);

  printf("*SailingRobot::run() started.\n");
  std::cout << "Waypoint target - ID: " << m_waypointModel.id << " lon: " <<
  m_waypointModel.positionModel.longitude	<< " lat : " <<
  m_waypointModel.positionModel.latitude << std::endl;

 return true;
}


bool waypointBehave::computeCommands(SystemStateModel &m_systemStateModel,std::unique_ptr<Position> const& position,
                                    std::vector<float> &twdBuffer,
                                    const unsigned int twdBufferMaxSize,bool m_mockPosition,bool m_getHeadingFromCompass){

  double heading = getHeading(m_systemStateModel,m_mockPosition,m_getHeadingFromCompass,position);

  //double windDir = m_systemStateModel.windsensorModel.direction; // outComment if use of tureWindDirCalculation

  std::cout << "heading: " << heading << "\n";
  std::cout << "heading ssm compass:" << m_systemStateModel.compassModel.heading<<"\n";

      if (m_mockPosition) {
          position->setCourseToSteer(m_waypointRouting.getCTS());
      }

      position->updatePosition();

  if (m_systemStateModel.gpsModel.online) {

    m_waypointRouting.getCommands(m_rudderCommand, m_sailCommand,
      position->getModel(),
      Utility::meanOfAngles(twdBuffer), heading, m_systemStateModel);



  } else {
    m_logger.error("SailingRobot::run(), gps NaN. Using values from last iteration.");
  }



  // check if we are within the radius of the waypoint
  // and move to next wp in that case
  if (m_waypointRouting.nextWaypoint(position->getModel() ) ) {

    // check if m_waypointModel.id exists in waypoint_index
    int i = m_dbHandler->retrieveCellAsInt("waypoint_index", m_waypointModel.id, "id");
    if (m_dbHandler->retrieveCellAsInt("sailing_robot_config", "1", "scanning") && i != 0 && insertScanOnce != i)
    {
      insertScanOnce = i;
      try {
        m_dbHandler->insertScan(m_waypointModel.id,position->getModel(),
          m_systemStateModel.windsensorModel.temperature,
          m_systemStateModel.gpsModel.utc_timestamp);
      } catch (const char * error) {
        m_logger.error(error);
        std::cout << error << std::endl;
      }
    }

    nextWaypoint();
    setupWaypoint();
    m_waypointRouting.setWaypoint(m_waypointModel);
    return true;
  }
  return true;
}

double waypointBehave::getSailCommand(){
  return m_sailCommand;
}

double waypointBehave::getRudderCommand(){
  return m_rudderCommand;
}

void waypointBehave::manageDatabase(std::vector<float> &twdBuffer,SystemStateModel &m_systemStateModel){
  //logging
  bool routeStarted = false;
  m_dbHandler->insertDataLog(
    m_systemStateModel,
    0,
    0,
    m_waypointRouting.getDTW(),
    m_waypointRouting.getBTW(),
    m_waypointRouting.getCTS(),
    m_waypointRouting.getTack(),
    m_waypointRouting.getGoingStarboard(),
    atoi(m_waypointModel.id.c_str()),
    Utility::meanOfAngles(twdBuffer),
    routeStarted
  );
}


void waypointBehave::nextWaypoint() {

	try {
		m_dbHandler->changeOneValue("waypoints", m_waypointModel.id,"1","harvested");
	} catch (const char * error) {
		m_logger.error(error);
	}
	m_logger.info("SailingRobot::nextWaypoint(), waypoint reached");
	std::cout << "Waypoint reached!" << std::endl;

}

void waypointBehave::setupWaypoint() {

	try {
		m_dbHandler->getWaypointFromTable(m_waypointModel);
	} catch (const char * error) {
		m_logger.error(error);
	}
	try {
		if (m_waypointModel.id.empty() ) {
			std::cout << "No waypoint found!"<< std::endl;
			throw "No waypoint found!";
		}
		else{
			std::cout << "New waypoint picked! ID:" << m_waypointModel.id <<" lat: "
			<< m_waypointModel.positionModel.latitude
			<< " lon: " << m_waypointModel.positionModel.longitude << " rad: "
			<< m_waypointModel.radius << std::endl;
		}
	} catch (const char * error) {
		m_logger.error(error);
		//m_dbHandler->insertMessageLog("NOTIME", "NOTYPE", "NO WAYPOINT FOUND!");
		//throw;
	}

	m_logger.info("setupWaypoint() done");
}

int waypointBehave::getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position) {

	int useGpsForHeadingKnotSpeed = 1;
	bool acceptedCompassSpeed = Utility::directionAdjustedSpeed(m_systemStateModel.gpsModel.heading, m_systemStateModel.compassModel.heading, m_systemStateModel.gpsModel.speed) < useGpsForHeadingKnotSpeed;

    if(m_mockPosition) {
        return position->getHeading();
    }

    //only accept getHeadingFromCompass if speed less than 1
	if (m_getHeadingFromCompass && acceptedCompassSpeed) {
    	return Utility::addDeclinationToHeading(m_systemStateModel.compassModel.heading, m_waypointModel.declination);
	}
    return m_systemStateModel.gpsModel.heading;
}
