#include "WaypointBehaviour.h"


WaypointBehaviour::WaypointBehaviour(DBHandler *db):
  RoutingBehaviour(db),//super class call
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

bool WaypointBehaviour::init(){

  printf("Starting Waypoint\n");
	setNextWaypoint(m_waypointModel);
	printf("OK\n");

	m_waypointRouting.setWaypoint(m_waypointModel);

  Logger::info("Waypoint Behaviour started");
  Logger::info("Waypoint target - ID: %s, Lon: %f, Lat: %f",  m_waypointModel.id.c_str(), 
                                                              m_waypointModel.positionModel.longitude, 
                                                              m_waypointModel.positionModel.latitude);

 return true;
}


void WaypointBehaviour::computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                      double trueWindDirection, bool mockPosition, bool getHeadingFromCompass){

  double heading = getHeading(systemStateModel,mockPosition,getHeadingFromCompass,position, m_waypointModel);

  //double windDir = systemStateModel.windsensorModel.direction; // outComment if use of tureWindDirCalculation

  std::cout << "heading: " << heading << "\n";
  std::cout << "heading ssm compass:" << systemStateModel.compassModel.heading<<"\n";

  if(waypointsChanged)
  {
    setNextWaypoint(m_waypointModel);
    waypointsChanged = false;
  }

  if (mockPosition) {
      position->setCourseToSteer(m_waypointRouting.getCTS());
  }

  position->updatePosition();

  if (systemStateModel.gpsModel.online) {

      m_waypointRouting.getCommands(m_rudderCommand, m_sailCommand,
        position->getModel(),
      trueWindDirection, heading, systemStateModel);

  } else {
    Logger::warning("WaypointBehaviour::computeCommands gps NaN. Using values from last iteration");
  }



  // check if we are within the radius of the waypoint
  // and move to next wp in that case
 if (m_waypointRouting.nextWaypoint(position->getModel() ) )
 {

    // check if m_waypointModel.id exists in waypoint_index
    /*int i = m_dbHandler->retrieveCellAsInt("waypoint_index", m_waypointModel.id, "id");
    if (m_dbHandler->retrieveCellAsInt("sailing_robot_config", "1", "scanning") && i != 0 && insertScanOnce != i)
    {
      insertScanOnce = i;
      try {
          m_dbHandler->insertScan(m_waypointModel.id,position->getModel(),
              systemStateModel.windsensorModel.temperature,
              systemStateModel.gpsModel.utc_timestamp);
      } catch (const char * error) {
          Logger::error("%s Error: %s", __PRETTY_FUNCTION__, error);
      }
    }*/

    harvestWaypoint(m_waypointModel);
    setNextWaypoint(m_waypointModel);
    m_waypointRouting.setWaypoint(m_waypointModel);
  }
}


void WaypointBehaviour::manageDatabase(double trueWindDirection, SystemStateModel &systemStateModel){
  //logging
  bool routeStarted = false;
  m_dbHandler->insertDataLog(
    systemStateModel,
    0,
    0,
    m_waypointRouting.getDTW(),
    m_waypointRouting.getBTW(),
    m_waypointRouting.getCTS(),
    m_waypointRouting.getTack(),
    m_waypointRouting.getGoingStarboard(),
    atoi(m_waypointModel.id.c_str()),
    trueWindDirection,
    routeStarted
  );
}
