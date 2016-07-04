#ifndef __BEHAVINGCLASS_H__
#define __BEHAVINGCLASS_H__

#include <stdint.h> // uint8_t
#include "thread/SystemState.h"
#include <memory>
#include "utility/Position.h"
#include "utility/RealPosition.h"
#include "utility/MockPosition.h"
#include "utility/Utility.h"
#include "dbhandler/DBHandler.h"
#include "models/WaypointModel.h"

class RoutingBehaviour{
public:
	RoutingBehaviour(DBHandler *db);
	virtual ~RoutingBehaviour(){};

	virtual bool init()=0; //if  function doesn't have the data required to function properly it return false

  virtual void computeCommands(SystemStateModel &systemStateModel,std::unique_ptr<Position> const& position,
                                      double trueWindDirection, bool mockPosition,
                                      bool getHeadingFromCompass) =0;  //make the computation of the commands return true if the computation was successfull

	virtual double getSailCommand();
  virtual double getRudderCommand();
  virtual void manageDatabase(double trueWindDirection, SystemStateModel &systemStateModel)=0;

protected:
  DBHandler *m_dbHandler;
  Logger m_logger;
  double m_rudderCommand, m_sailCommand;
  virtual void setNextWaypoint(WaypointModel &waypointModel);
  virtual void harvestWaypoint(WaypointModel waypointModel);
  virtual int getHeading(SystemStateModel &systemStateModel,bool mockPosition,bool getHeadingFromCompass,std::unique_ptr<Position> const& position, WaypointModel waypointModel);
};

#endif
