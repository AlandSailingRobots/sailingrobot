#ifndef __WAYPOINTMODEL_H__
#define __WAYPOINTMODEL_H__

class WaypointModel
{
public:
	WaypointModel(float latitude, float longitude, int radius, std::string id) :
		latitude(latitude),
		longitude(longitude),
		radius(radius),
		id(id)
	{};

	~WaypointModel() {};

	float latitude;
	float longitude;
	int radius;
	std::string id;
};

#endif

