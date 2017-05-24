/****************************************************************************************
*
* File:
* 		NewSailCommand.h
*
* Purpose:
*		This class computes the SailCommand and return it
*
* Developer Notes: This node should be renamed to SailCommand once the SailCommand
* in waypointrouting is removed as promised.
*
*
***************************************************************************************/

#pragma once

#include <math.h>
#include <algorithm>
#include <cmath>

class NewSailCommand{

public:
  NewSailCommand();
  ~NewSailCommand();

  int getSailCommand(double apparentWindDirection);

private:
  double m_maxSailAngle, m_minSailAngle;
  const double MAX_SAIL_ANGLE =  M_PI / 4.2f;
	const double  MIN_SAIL_ANGLE =  M_PI / 32.0f;
};
