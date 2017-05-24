/****************************************************************************************
*
* File:
* 		NewSailCommand.h
*
* Purpose:
*		This class computes the SailCommand and return it
*
*
*
***************************************************************************************/

#pragma once

#include <math.h>
#include <algorithm>
#include <cmath>

class SoftSailControl{

public:
  SoftSailControl();
  ~SoftSailControl();

  int getSailCommand(double apparentWindDirection);

  // calulates the command to set the sailposition
  int getCommand(double command);

  //sets the commandvalues returned by getSailCommand()
  void setCommandValues(int closeReach, int running);

private:
  double m_maxSailAngle, m_minSailAngle;
  const double MAX_SAIL_ANGLE =  M_PI / 4.2f;
	const double  MIN_SAIL_ANGLE =  M_PI / 32.0f;

  int m_closeReachCommand;
  int m_runningCommand;
};
