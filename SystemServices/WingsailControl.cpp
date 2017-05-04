

#include "WingsailControl.h"
#include "Math/Utility.h"

 WingsailControl::WingsailControl(double ServoSailMinAngle, double MaxServoAngle) :
    m_TrueWindDirection(0), m_VesselHeading(0), m_CurrentServoAngle(0), 
    m_ServoSailMinAngleDiff(ServoSailMinAngle), m_MaxServoSailAngle(MaxServoAngle)
{  }    

void WingsailControl::setTrueWindDirection(double direction) {
    m_TrueWindDirection = direction;
}

void WingsailControl::setVesselHeading(double heading) {
    m_VesselHeading = heading;
}

float WingsailControl::calculateServoAngle() {

    float headingWindDiff = Utility::limitAngleRange(m_VesselHeading - m_TrueWindDirection);

    if(m_CurrentServoAngle == 0){
        // >= or >      ?? 
        if(headingWindDiff > 180 && headingWindDiff < 360){
            m_CurrentServoAngle = m_MaxServoSailAngle;
        } else {
            m_CurrentServoAngle = -m_MaxServoSailAngle;
        }
    } 
    
    else if(m_CurrentServoAngle < 0) {
        if(headingWindDiff > (180+m_ServoSailMinAngleDiff) && headingWindDiff < (360 - m_ServoSailMinAngleDiff)) {
            m_CurrentServoAngle = m_MaxServoSailAngle;
        }
    }

    else if(m_CurrentServoAngle > 0) {
        if(headingWindDiff > m_ServoSailMinAngleDiff && headingWindDiff < (180 - m_ServoSailMinAngleDiff)) {
            m_CurrentServoAngle = -m_MaxServoSailAngle;
        }
    }

    return m_CurrentServoAngle;

}