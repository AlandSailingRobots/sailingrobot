#include "RoutingBehaviour.h"
bool RoutingBehaviour::waypointsChanged = false;

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

void RoutingBehaviour::setWaypointsChanged()
{
	waypointsChanged = true; //if waypoints changed in Database during run, bool set true
}

void RoutingBehaviour::setNextWaypoint(WaypointModel &waypointModel)
{
    try {
    	m_dbHandler->getWaypointFromTable(waypointModel);
	} catch (const char * error) {
		Logger::error("%s Error: %s", __func__, error);
	}

	if (waypointModel.id.empty() ) {
		Logger::warning("%s No Waypoint Found", __func__);
	}
	else{
		Logger::info("New Waypoint picked! ID: %s, Lon: %f, Lat: %f, Rad: %d",  waypointModel.id.c_str(), 
                                                                				waypointModel.positionModel.longitude, 
                                                                				waypointModel.positionModel.latitude,
                                                               					waypointModel.radius);
	}
}

void RoutingBehaviour::harvestWaypoint(WaypointModel waypointModel)
{
	try {
		m_dbHandler->changeOneValue("waypoints", waypointModel.id,"1","harvested");
	} catch (const char * error) {
		Logger::error("%s Error: %s", __func__, error);
	}
	Logger::info("Reached waypoint");
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