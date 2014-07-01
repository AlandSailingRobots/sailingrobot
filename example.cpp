#include "SailingRobot.h"




int main(int argc, char *argv[]) {

	SailingRobot sr;
	string path, db, errorLog;
	if (argc < 2) {
		path = "";
		db = "asr.db";
		errorLog = "errors.log";
	} else {
		path = std::string(argv[1]);
		db = "/asr.db";
		errorLog = "/errors.log";
	}

	try {
		sr.init(path, db, errorLog);
		sr.run();
	} catch (const char * e) {
		sr.shutdown();
		return 1;
	}
	sr.shutdown();
	return 0;
}
