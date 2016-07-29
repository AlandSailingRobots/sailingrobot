#ifndef EXTERNAL_COMMAND_H
#define EXTERNAL_COMMAND_H

#include <mutex>
#include <string>


class ExternalCommand {
public:
	ExternalCommand(std::string timestamp, bool autorun, int rudderCommand, int sailCommand);
	~ExternalCommand() {};

	bool setData(std::string timestamp, bool autorun, int rudderCommand, int sailCommand);
	
	bool getAutorun();
	int getRudderCommand();
	int getSailCommand();

private:
	std::mutex mtx; // mutex for critical section

	std::string m_timestamp;
	bool m_autorun;
	int m_rudderCommand;
	int m_sailCommand;
};

#endif
