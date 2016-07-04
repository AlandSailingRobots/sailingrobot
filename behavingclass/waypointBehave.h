#ifndef __WAYPOINTBEHAVE_H__
#define __WAYPOINTBEHAVE_H__
#include <stdint.h> // uint8_t

#include "behavingclass.h"
#include "waypointrouting/WaypointRouting.h"
//#include "models/WaypointModel.h"





class waypointBehave:public BehavingClass{
public:
	waypointBehave(DBHandler *db);
	~waypointBehave(){};

	bool init(); //if  function doesn't have the data required to function properly it return false

  bool computeCommands(SystemStateModel &m_systemStateModel,std::unique_ptr<Position> const& position,
                                      std::vector<float> &twdBuffer,
                                      const unsigned int twdBufferMaxSize,bool m_mockPosition,
                                      bool m_getHeadingFromCompass);  //make the computation of the commands return true if the computation was successfull
	double getSailCommand();
  double getRudderCommand();
  void manageDatabase(std::vector<float> &twdBuffer,SystemStateModel &m_systemStateModel);

private:
  double m_rudderCommand;
  double m_sailCommand;
  WaypointModel m_waypointModel;
  WaypointRouting m_waypointRouting;
  int insertScanOnce;
  void setupWaypoint();
  void nextWaypoint();
  int getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position);
};

#endif
