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
		Logger::warning("%s No waypoint found, holding last waypoint", __func__);
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
	if(not m_dbHandler->changeOneValue("waypoints", waypointModel.id,"1","harvested"))
	{
		Logger::error("Failed to harvest waypoint");
	}
	Logger::info("Reached waypoint");
}

int RoutingBehaviour::getHeading(SystemStateModel &systemStateModel,bool mockPosition,bool getHeadingFromCompass,std::unique_ptr<Position> const& position, WaypointModel waypointModel) {
	//Use GPS for heading only if speed is higher than 1 knot
	int useGpsForHeadingKnotSpeed = 1;
	bool gpsForbidden = Utility::directionAdjustedSpeed(systemStateModel.gpsModel.heading, systemStateModel.compassModel.heading, systemStateModel.gpsModel.speed) < useGpsForHeadingKnotSpeed;

	getMergedHeading(systemStateModel, waypointModel, true); //decrease compass weight on each iteration

    if(mockPosition) {
        return position->getHeading();
    }

	if (getHeadingFromCompass) {
		//Should return compass heading if below one knot and not currently merging and vice versa
    	return Utility::addDeclinationToHeading(getMergedHeading(systemStateModel, waypointModel, gpsForbidden), waypointModel.declination);
	}
    return systemStateModel.gpsModel.heading;
}

int RoutingBehaviour::getMergedHeading(SystemStateModel &systemStateModel, WaypointModel waypointModel, bool increaseCompassWeight){
	//Shouldn't be hardcoded
	float tickRate = 0.01;

	int headingCompass = Utility::addDeclinationToHeading(systemStateModel.compassModel.heading, waypointModel.declination);
	int headingGps = systemStateModel.gpsModel.heading;

	if (increaseCompassWeight){
		m_gpsHeadingWeight = m_gpsHeadingWeight - tickRate; //Decrease gps weight
		if (m_gpsHeadingWeight < 0.0) m_gpsHeadingWeight = 0;
	}else{
		m_gpsHeadingWeight = m_gpsHeadingWeight + tickRate;
		if (m_gpsHeadingWeight > 1.0) m_gpsHeadingWeight = 1.0;
	}

	//Difference calculation
	float diff = ((headingGps - headingCompass) + 180 + 360);
	while (diff > 360) diff -= 360;
	diff -= 180;

	//Merge angle calculation
	int returnValue = 360 + headingCompass + (diff * m_gpsHeadingWeight);
	while (returnValue > 360) returnValue -= 360;

	return returnValue;
}
