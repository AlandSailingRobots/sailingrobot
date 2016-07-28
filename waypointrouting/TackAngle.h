#ifndef __TACKANGLE_H__
#define __TACKANGLE_H__

class SystemStateModel;

class TackAngle
{
public:
	TackAngle(double tackAngle, double maxTackAngle, double minTackSpeed);
	~TackAngle();

	double adjustedTackAngle(double gpsHeading, double gpsSpeed, double compassHeading);

private:
	double m_tackAngle, m_maxTackAngle, m_minTackSpeed;
	
};

#endif