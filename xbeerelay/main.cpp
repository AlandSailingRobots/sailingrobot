#include "xBeeRelay.h"
#include <stdio.h>

int main() {
	xBeeRelay relay;
	std::string data = "";

	while (true) { 
	
		// receive data from r-pi
		data = relay.receiveData();
		std::cout << "Sent data: " << data << std::endl;

		// parse extract data between <message></message>
		data = relay.extractMessage(data);
		std::cout << "Extracted data: " << data << "\n" << std::endl;	
		
		// insert data in local database


		// push data to server
		relay.pushLogsToServer(data);

		if (relay.checkForConfigUpdates()) {
			std::string configs = relay.getConfigsFromWeb();
			relay.relayData(configs);
		}
		
		usleep(3000000);
	}

}