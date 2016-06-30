#ifndef __WINDSENSORMODEL_H__
#define __WINDSENSORMODEL_H__

class WindsensorModel
{
public:
	WindsensorModel(
		float direction,
		float speed,
		float temperature
	) :
		direction(direction),
		speed(speed),
		temperature(temperature)
	{};

	~WindsensorModel() {};

	float direction;
	float speed;
	float temperature;
};

#endif