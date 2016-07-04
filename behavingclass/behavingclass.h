#ifndef __BEHAVINGCLASS_H__
#define __BEHAVINGCLASS_H__

#include <stdint.h> // uint8_t
#include "thread/SystemState.h"
#include <memory>
#include "utility/Position.h"
#include "utility/RealPosition.h"
#include "utility/MockPosition.h"
#include "dbhandler/DBHandler.h"

class BehavingClass{
public:
	BehavingClass(DBHandler *db);
	virtual ~BehavingClass(){};

	virtual bool init()=0; //if  function doesn't have the data required to function properly it return false

  virtual bool computeCommands(SystemStateModel &m_systemStateModel,std::unique_ptr<Position> const& position,
                                      std::vector<float> &twdBuffer,
                                      const unsigned int twdBufferMaxSize,bool m_mockPosition,
                                      bool m_getHeadingFromCompass) =0;  //make the computation of the commands return true if the computation was successfull
	virtual double getSailCommand()=0;
  virtual double getRudderCommand()=0;
  virtual void manageDatabase(std::vector<float> &twdBuffer,SystemStateModel &m_systemStateModel)=0;

protected:
  DBHandler *m_dbHandler;
  Logger m_logger;
  //SystemStateModel *m_systemStateModel;
  virtual int getHeading(SystemStateModel &m_systemStateModel,bool m_mockPosition,bool m_getHeadingFromCompass,std::unique_ptr<Position> const& position)=0;
};

#endif
