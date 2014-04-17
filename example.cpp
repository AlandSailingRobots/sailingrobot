#include "SailingRobot.h"

int main() {
	SailingRobot sr;
	try {
		sr.init();
		sr.run();
		sr.shutdown();
	} catch (char const* e) {
		std::cout << e << "\n";
	}
	std::cout << "mainuloop end\n";
}