#ifndef __GPSMODEL_H__
#define __GPSMODEL_H__

#include "PositionModel.h"
#include <string>

class GPSModel
{
public:
	GPSModel(
		std::string timestamp,
		PositionModel positionModel,
		double altitude,
		double speed,
		double heading,
		int satellitesUsed
	) :
		online(false),		
		timestamp(timestamp),
		utc_timestamp(""),
		positionModel(positionModel),
		altitude(altitude),
		speed(speed),
		heading(heading),
		satellitesUsed(satellitesUsed)
	{};
	
	~GPSModel() {};

	bool online;
	std::string timestamp;
	std::string utc_timestamp;
	PositionModel positionModel;
	double altitude;
	double speed;
	double heading;
	int satellitesUsed;
};

#endif
