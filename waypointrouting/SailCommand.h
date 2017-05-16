#ifndef __SAILCOMMAND_H__
#define __SAILCOMMAND_H__


class SailCommand {

public:
	SailCommand();
	~SailCommand();

	// calulates the command to set the sailposition
	int getCommand(double command);

	//sets the commandvalues returned by getSailCommand()
	void setCommandValues(int closeReach, int running);

	int getSailCommand(double apparentWindDirection);

private:
	int m_closeReachCommand;
	int m_runningCommand;
	double m_maxSailAngle, m_minSailAngle;

};

#endif
