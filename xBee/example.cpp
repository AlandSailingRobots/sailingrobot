#include <iostream>
#include <string>
#include <unistd.h>
#include "xBee.h"

using namespace std;
int main(int argc, char** argv){

	xBee xbee;
	int port = -1;

	try {
		port = xbee.init();
	}
	catch (const char* exception) {
		cout << exception << endl;
	}

	
	cout << port << endl;

	int tics = 10;
	string outPut = "0";

	while (tics > 0){

		outPut = xbee.readOutput(port);
		cout << outPut << endl;
		tics--;
		usleep(1000000);


	}
	

	
	cout << "Done" << endl;








}