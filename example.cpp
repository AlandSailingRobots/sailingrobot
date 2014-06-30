#include "SailingRobot.h"

int main(int argc, char *argv[]) {
	SailingRobot sr;
	if (argc < 2) {
		sr.init("", "asr.db", "errors.log");
	} else {
		sr.init(std::string(argv[1]), "/asr.db", "/errors.log");
	}
	sr.run();
	sr.shutdown();
}
