#include "xBeeSync.h"
#include <unistd.h>		//sleep

xBeeSync::xBeeSync(SystemState *systemState) :
	m_model(
		SystemStateModel(
			GPSModel("",0,0,0,0,0,0),
			WindsensorModel(0,0,0),
			CompassModel(0,0,0),
			0,
			0
		)
	),
	m_system_state(systemState),
	m_running(true)
{
	//	skapa ny xBee och kör xbee.init(baudRate (57600))
	m_xbee_fd = m_xBee.init(57600);
}

void xBeeSync::run()
{
	while(isRunning()) 
	{
		m_system_state->setData(m_model);

		std::string res_xml = m_XML_log.log_xml(
			m_model.gpsModel.timestamp,			// Timestamp
			m_model.windsensorModel.direction,	// winddir degrees
			m_model.windsensorModel.speed, 		// windspeed ms
			m_model.compassModel.heading, 		// Heading deg
			m_model.compassModel.pitch, 		// Pitch deg
			m_model.compassModel.roll, 			// Roll deg
			m_model.gpsModel.latitude, 			// gml:pos arg1, lat
			m_model.gpsModel.longitude, 		// gml:pos arg2, long
			m_model.gpsModel.heading, 			// course over ground(deg)
			m_model.gpsModel.speed, 			// speed over ground(ms)
			m_model.rudder,						// Rudderpos
			m_model.sail						// Sailpos
		);

		// Kan skicka loggen direkt med:
		m_xBee.sendXML(m_xbee_fd,res_xml);
		
		//make sure there are at least one second between each xml message
		sleep(1);
	}
}

void xBeeSync::close()
{
	m_mutex.lock();
	m_running = false;
	m_mutex.unlock();
}

bool xBeeSync::isRunning()
{
	bool running;
	m_mutex.lock();
	running = m_running;
	m_mutex.unlock();
	return running;
}
