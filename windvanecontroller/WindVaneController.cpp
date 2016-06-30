#include "WindVaneController.h"
#include "utility/Utility.h"

#include <iostream>

WindVaneController::WindVaneController() {

}

void WindVaneController::setVaneAngle(double trueWindDirection, double courseToSteer) {
	m_vaneAngle = Utility::limitAngleRange(trueWindDirection - courseToSteer);
}

void WindVaneController::adjustAngle(double heading, double courseToSteer) {
        m_vaneAngle += heading - courseToSteer;
        m_vaneAngle = Utility::limitAngleRange(m_vaneAngle);
}

double WindVaneController::getVaneAngle() {
	return m_vaneAngle;
}

