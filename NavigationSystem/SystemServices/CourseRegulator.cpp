#include "CourseRegulator.h"
#include "Math/Utility.h"

CourseRegulator::CourseRegulator(double maxRudderAngle, double maxCourseAngleDiff) : 
    m_MaxRudderAngle(maxRudderAngle), m_VesselCourse(0), m_CourseToSteer(0),
    m_MaxCourseAngleDiff(maxCourseAngleDiff)
{  }

void CourseRegulator::setVesselCourse(double vesselCourse) {
    m_VesselCourse = vesselCourse;
}

void CourseRegulator::setCourseToSteer(double courseToSteer) {
    m_CourseToSteer = courseToSteer;
}
    
double CourseRegulator::calculateRudderAngle() {
    double offCourse = Utility::limitAngleRange(m_CourseToSteer - m_VesselCourse);

	if (offCourse > 180) {
		offCourse -= 360;
	}

	if (offCourse > -m_MaxCourseAngleDiff && offCourse <= m_MaxCourseAngleDiff) {
		return -m_MaxRudderAngle + (offCourse + m_MaxCourseAngleDiff ) * (m_MaxRudderAngle + m_MaxRudderAngle) / (m_MaxCourseAngleDiff*2.0);
	} else {

		if (offCourse > m_MaxCourseAngleDiff) {
			offCourse = m_MaxRudderAngle;
		}
		else {
			offCourse = -m_MaxRudderAngle;
		}

	}

	return offCourse;
}