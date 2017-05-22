#pragma once
#include <math.h>
#include <algorithm>
#include <cmath>

class SailCommand{

public:
  SailCommand();
  ~SailCommand();

  int getSailCommand(double apparentWindDirection);

private:
  double m_maxSailAngle, m_minSailAngle;
  const double MAX_SAIL_ANGLE =  M_PI / 4.2f;
	const double  MIN_SAIL_ANGLE =  M_PI / 32.0f;
};
