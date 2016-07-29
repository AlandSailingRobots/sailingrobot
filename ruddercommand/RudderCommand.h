#ifndef __RUDDERCOMMAND_H__
#define __RUDDERCOMMAND_H__


class RudderCommand {

	/* RudderCommand handles the calculations for the rudder, the calculations is based
	 from a course to steer value and returns a value between 97-103 to be used with the servo 
	 module. */

public:

	RudderCommand();
	~RudderCommand();

	/* Returns a value between 97-103 for the rudder. Takes a courseToSteer value as 
	 parameter. */
	int getCommand(double command);

	// sets the values returned by getCommand()
	void setCommandValues(int starboardExtreme, int midships);

	//returns m_midshipsCommand
	int getMidShipsCommand();

private:

	int m_steeringValue;
	int m_offCourse;
	int m_course;

	int m_extremeCommand;
	int m_midshipsCommand;
};

#endif
