#ifndef __WAYPOINTBEHAVE_H__
#define __WAYPOINTBEHAVE_H__
#include <stdint.h> // uint8_t

#include "RoutingBehaviour.h"
#include "waypointrouting/WaypointRouting.h"
//#include "models/WaypointModel.h"





class WaypointBehaviour:public RoutingBehaviour{
public:
	WaypointBehaviour(DBHandler *db);
	~WaypointBehaviour(){};

	bool init(); //if  function doesn't have the data required to function properly it return false

  bool computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                      double trueWindDirection, bool mockPosition,
                                      bool getHeadingFromCompass);  //make the computation of the commands return true if the computation was successfull

  void manageDatabase(std::vector<float> &twdBuffer,SystemStateModel &systemStateModel);

private:
  WaypointModel m_waypointModel;
  WaypointRouting m_waypointRouting;
  int insertScanOnce;
};

#endif
