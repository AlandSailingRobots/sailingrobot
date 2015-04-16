#include "SailingRobot.h"



int main(int argc, char *argv[]) {

	printf("Sailing robot example\n");
	printf("=====================\n");

	SailingRobot sr;
	printf("CONFIG: ");
	std::string path, db, errorLog;
	if (argc < 2) {
		printf("1\n");
		path = "";
		db = "asr.db";
		errorLog = "errors.log";
	} else {
		printf("2\n");
		path = std::string(argv[1]);
		db = "/asr.db";
		errorLog = "/errors.log";
	}
	printf("START\n");
	try {
		printf("-INIT\n");
		sr.init(path, db, errorLog);
		printf("-RUN\n");
		sr.run();
	} catch (const char * e) {
		printf("-ERR\n");
		printf("-:%s\n",e);
		sr.shutdown();
		return 1;
	}
	sr.shutdown();
	printf("END\n");
	return 0;
}
