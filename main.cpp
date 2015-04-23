#include "SailingRobot.h"



int main(int argc, char *argv[]) {

	printf("\n");
	printf("  Sailing Robot\n");
	printf("=================\n");

	SailingRobot sr;

	std::string path, db, errorLog;
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
		printf("-Initializing...\n");
		sr.init(path, db, errorLog);
		printf("-OK\n");
		printf("-Executing...\n");
		sr.run();
		printf("-DONE\n");
	} catch (const char * e) {
		printf("ERROR[%s]\n\n",e);
		sr.shutdown();
		return 1;
	}
	sr.shutdown();
	printf("-Finished.\n");
	return 0;
}
