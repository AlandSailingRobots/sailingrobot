#ifndef __COMPASSMODEL_H__
#define __COMPASSMODEL_H__

#include "models/AccelerationModel.h"

class CompassModel
{
public:
	CompassModel(int heading, int pitch, int roll, AccelerationModel accelerationModel) :
		heading(heading),
		pitch(pitch),
		roll(roll),
		accelerationModel(accelerationModel)
	{};

	~CompassModel() {};

	int heading;
	int pitch;
	int roll;
        
        AccelerationModel accelerationModel;
};
#endif
