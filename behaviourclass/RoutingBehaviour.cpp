#include "RoutingBehaviour.h"

RoutingBehaviour::RoutingBehaviour(DBHandler *db):  m_dbHandler(db){
}

double RoutingBehaviour::getSailCommand()
{
    return m_sailCommand;
}

double RoutingBehaviour::getRudderCommand()
{
    return m_rudderCommand;
}

void RoutingBehaviour::setNextWaypoint(WaypointModel &waypointModel)
{
    try {
    m_dbHandler->getWaypointFromTable(waypointModel);
	} catch (const char * error) {
		m_logger.error(error);
	}

	if (waypointModel.id.empty() ) {
		std::cout << "No waypoint found!"<< std::endl;
	}
	else{
		std::cout << "New waypoint picked! ID:" << waypointModel.id <<" lon: "
		<< waypointModel.positionModel.longitude
		<< " lat: " << waypointModel.positionModel.latitude << " rad: "
		<< waypointModel.radius << std::endl;
	}

	m_logger.info("setupWaypoint() done");
}

void RoutingBehaviour::harvestWaypoint(WaypointModel waypointModel)
{
	try {
		m_dbHandler->changeOneValue("waypoints", waypointModel.id,"1","harvested");
	} catch (const char * error) {
		m_logger.error(error);
	}
	m_logger.info("SailingRobot::nextWaypoint(), waypoint reached");
	std::cout << "Waypoint reached!" << std::endl;
}

int RoutingBehaviour::getHeading(SystemStateModel &systemStateModel,bool mockPosition,bool getHeadingFromCompass,std::unique_ptr<Position> const& position, WaypointModel waypointModel) {

	int useGpsForHeadingKnotSpeed = 1;
	bool acceptedCompassSpeed = Utility::directionAdjustedSpeed(systemStateModel.gpsModel.heading, systemStateModel.compassModel.heading, systemStateModel.gpsModel.speed) < useGpsForHeadingKnotSpeed;

    if(mockPosition) {
        return position->getHeading();
    }

    //only accept getHeadingFromCompass if speed less than 1
	if (getHeadingFromCompass && acceptedCompassSpeed) {
    	return Utility::addDeclinationToHeading(systemStateModel.compassModel.heading, waypointModel.declination);
	}
    return systemStateModel.gpsModel.heading;
}