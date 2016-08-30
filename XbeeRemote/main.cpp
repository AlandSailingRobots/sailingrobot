//#include "xBeeRelay.h"
#include "../xBee/Xbee.h"
#include "Network/XbeePacketNetwork.h"
#include "Network/LinuxSerialDataLink.h"
#include <stdio.h>
#include "../SystemServices/Logger.h"
#include "../utility/SysClock.h"
#include "../Messages/VesselStateMsg.h"
#include "../Messages/DataRequestMsg.h"
#include "../Messages/WindDataMsg.h"
#include "../Messages/CompassDataMsg.h"
#include "../Messages/GPSDataMsg.h"
#include "../Messages/ActuatorPositionMsg.h"
#include "../Messages/WaypointDataMsg.h"
#include "../Messages/CourseDataMsg.h"
#include "udpclient.h"
#include "XbeeRemote.h"



int main() {
	std::string portName = "";
	#ifdef _WIN32
		portName = "COM3";
	#elif __linux__
		portName = "/dev/ttyUSB0";
	#endif

	XbeeRemote xbeeRemote(portName);

	if(not xbeeRemote.initialise())
	{
		Logger::error("No Xbee");
	}

	while(true)
	{
		xbeeRemote.run();
	}
}
