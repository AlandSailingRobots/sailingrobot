#include <iostream>
#include <string>
#include <unistd.h>
#include <fstream>
#include "xBee/xBee.h"

using namespace std;

int main(int argc, char** argv) {

	xBee xbee;
	int port = -1;
	int option = -1;
	string data;

	while (option != 0 && option != 1 && option != 2) {

		cout << "Please select read (0), manual write (1) or XML-write (2)" << endl;
		cin >> option;
		cout << "Selected: " << option << endl;
		
	}

	try {
		port = xbee.init();
	} catch (const char* exception) {
		cout << exception << endl;
	}

	if (port != -1){

		cout << "Connection successful." << endl;

	} else {

		cout << "Connection failed!" << endl;
		cout << "- are you running on r-pi with the xBee connected?" << endl;
		cout << "- does the running device have a static reference to /dev/xbee?" << endl;

	}

	if (option == 1){

		while(true){

			cout << "Please enter a message" << endl;
			cin >> data;
			xbee.transmitData(port, data);
			cout << data + " printed to xBee receiver" << endl;

		}

	} else if (option == 0) {

		string outPut = "0";

		while (true) {

			outPut = xbee.receiveData(port);
			cout << outPut << endl;
			usleep(1000000);
		}

	} else if (option == 2) {

		while (true) {

			std::string stringfile, tmp;

			std::ifstream input("../log_output.xml");

			while(!input.eof()) {

    			getline(input, tmp);
    			stringfile += tmp;
			}

			xbee.transmitData(port, stringfile);
			cout << "Sent a predefined XML-string" << endl;
			usleep(1000000);
		}
	}
	cout << "Done" << endl;
} 
