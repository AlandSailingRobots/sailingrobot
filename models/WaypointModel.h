#ifndef __WAYPOINTMODEL_H__
#define __WAYPOINTMODEL_H__

#include "PositionModel.h"
#include <string>

class WaypointModel
{
public:	
	WaypointModel(PositionModel positionModel, int radius, std::string id, int declination) :
		positionModel(positionModel),
		radius(radius),
		declination(declination),
		id(id),
		time(0)
	{};

	~WaypointModel() {};

	PositionModel positionModel;	
	int radius;
	int declination;
	std::string id;
	int time;
};

#endif