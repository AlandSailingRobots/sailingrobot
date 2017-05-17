#pragma once

class CourseRegulator {
public:
    CourseRegulator(double maxRudderAngle, double maxCourseAngleDiff);
    void setVesselCourse(double vesselCourse);
    void setCourseToSteer(double courseToSteer);

    double calculateRudderAngle();
private:
    double m_MaxRudderAngle;
    double m_VesselCourse;
    double m_CourseToSteer;
    double m_MaxCourseAngleDiff;
};
