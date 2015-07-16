#include "xBeeSync.h"
#include <chrono>
#include <thread>

xBeeSync::xBeeSync(ExternalCommand* externalCommand, SystemState *systemState,
				   bool sending, bool receiving) :
	m_external_command(externalCommand),
	m_model(
		SystemStateModel(
			GPSModel(false,"",0,0,0,0,0,0),
			WindsensorModel(0,0,0),
			CompassModel(0,0,0),
			0,
			0
		)
	),
	m_system_state(systemState),
	m_running(true),
	m_sending(sending),
	m_receiving(receiving)
{
	//	skapa ny xBee och kör xbee.init(baudRate (57600))
	m_xbee_fd = m_xBee.init(57600);
	std::cout << "*xBee initialized - receiving:" << m_receiving
			  << " sending:" << m_sending << std::endl;
	m_logger.info(std::string(
		"*xBee initialized - receiving:") +	std::to_string(m_receiving)	+
		" sending:" + std::to_string(m_sending));
}

void xBeeSync::run()
{
	std::cout << "*xBeeSync thread started." << std::endl;
	m_logger.info("*xBeeSync thread started.");

	while(isRunning()) 
	{
		m_system_state->getData(m_model);

		if(m_sending)
		{
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
		}
		
		if(m_receiving)
		{
			std::string res_xml = m_xBee.readOutput(m_xbee_fd);
 			
			//If return value equals -1, parsing failed...
 			int rudder_cmd = m_XML_log.parse_rudCMD(res_xml);
	 		int sail_cmd = m_XML_log.parse_saiCMD(res_xml);
			std::string timestamp = m_XML_log.parse_time(res_xml);

	 		if(timestamp.length() > 0) {
	 			std::cout << "Timestamp in xBeeSync::run = " << timestamp << std::endl;
				m_logger.info("Timestamp in xBeeSync::run = " + timestamp);
	 		}
	 		if(rudder_cmd != -1) {
	 			std::cout << "Rudder command in xBeeSync::run = " << rudder_cmd << std::endl;	
	 			m_logger.info(std::string("Rudder command in xBeeSync::run = ")
	 				+ std::to_string(rudder_cmd));
	 			m_model.rudder = rudder_cmd;
	 		}
	 		if(sail_cmd != -1) {
	 			std::cout << "Sail command in xBeeSync::run = " << sail_cmd << std::endl;	
	 			m_logger.info(std::string("Sail command in xBeeSync::run = ")
	 				+ std::to_string(sail_cmd));
	 			m_model.sail = sail_cmd;
	 		}

	 		bool autorun = false;
	 		
	 		if(sail_cmd != -1 && rudder_cmd != -1){
	 			m_external_command->setData(timestamp, autorun, rudder_cmd, sail_cmd);
	 		}
		}

		//make sure there are at least 0.4 seconds between each xml message
		std::this_thread::sleep_for(
			std::chrono::milliseconds(400));
	}
	std::cout << "*xBeeSync thread exited." << std::endl;
	m_logger.info("*xBeeSync thread exited.");
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
