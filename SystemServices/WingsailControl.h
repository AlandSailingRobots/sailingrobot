


class WingsailControl {
public:

    WingsailControl(double ServoSailMinAngle, double MaxServoAngle);

    void setTrueWindDirection(double direction);
    void setVesselHeading(double heading);
    float calculateServoAngle();

private:

    double m_TrueWindDirection;
    double m_VesselHeading;
    double m_CurrentServoAngle;

    double m_ServoSailMinAngleDiff;
    double m_MaxServoSailAngle;

};