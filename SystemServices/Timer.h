#ifndef __TIMER_H__
#define __TIMER_H__

#include <chrono>

class Timer {

public:
	Timer();
	~Timer() {};

	/*
	 * activates timer & sets start time to current time
	 * unless it's already running
	 */
	void start();

	/*
	 * activates timer & sets start time to current time
	 */
	void reset();

	/*
	 * deactivates timer
	 */
	void stop();

	/*
	 * returns seconds passed since timer started
	 */
	double timePassed();
	/*
	 * returns difference between timer running time (from start)
	 * and provided seconds, in seconds
	 */
	double timeUntil(double seconds);
	/*
	 * sleeps until timer reaches provided time 
	 */
	void sleepUntil(double seconds);

	/*
	 * checks if timer is running & if it reached the time in arg
	 */
	bool timeReached(double seconds);

	bool started() { return m_running; }

private:
	std::chrono::steady_clock::time_point m_start;
	bool m_running;
	double m_timePassed;
};

#endif