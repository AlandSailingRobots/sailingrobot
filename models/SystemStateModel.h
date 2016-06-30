#ifndef __SYSTEMSTATEMODEL_H__
#define __SYSTEMSTATEMODEL_H__

#include "models/GPSModel.h"
#include "models/CompassModel.h"
#include "models/WindsensorModel.h"
#include "models/AnalogArduinoModel.h"

class SystemStateModel {
public:
	SystemStateModel(
		GPSModel gpsModel,
		WindsensorModel windsensorModel,
		CompassModel compassModel,
		AnalogArduinoModel arduinoModel,
		int rudder,
		int sail
	) :
		gpsModel(gpsModel),
		windsensorModel(windsensorModel),
		compassModel(compassModel),
		arduinoModel(arduinoModel),
		rudder(rudder),
		sail(sail)
	{};

	~SystemStateModel() {};

	GPSModel gpsModel;
	WindsensorModel windsensorModel;
	CompassModel compassModel;
	AnalogArduinoModel arduinoModel;
	int rudder;
	int sail;
};

#endif
