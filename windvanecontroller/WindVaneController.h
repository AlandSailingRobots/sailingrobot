#ifndef __WINDVANECONTROLLER_H__
#define __WINDVANECONTROLLER_H__

#include "waypointrouting/WaypointRouting.h"

class WindVaneController {

public:

	WindVaneController();
	~WindVaneController() {};

	void setVaneAngle(double trueWindDirection, double courseToSteer);
        void adjustAngle(double heading, double courseToSteer);
	double getVaneAngle();
	
private:

	double m_vaneAngle;

};

#endif
