/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RealPosition.cpp
 * Author: sailbot
 * 
 * Created on April 27, 2016, 11:47 AM
 */

#include "MockPosition.h"

#include <cmath>
#include "Utility.h"

MockPosition::MockPosition() : 
                m_positionModel(STARTLATPOSITION,STARTLONGPOSITION),
                m_heading(0),
                m_courseToSteer(0.0){
    
}

void MockPosition::setHeading(int heading) {
    m_heading =  heading; 
    
}
    
void MockPosition::setCourseToSteer(double cts) { 

    m_courseToSteer = cts;
    
    if (m_heading > m_courseToSteer) {
        m_heading--;
        return;
    } 
    if (m_heading < m_courseToSteer) {
        m_heading++;
        return;
    } 
 
    m_heading = m_courseToSteer;    
}

int MockPosition::getHeading() { 
    return m_heading;
    
}
    
void MockPosition::updatePosition() {
        mockLatitude();
        mockLongitude();
        
}

PositionModel MockPosition::getModel() {
    return m_positionModel;
}

MockPosition::~MockPosition() {
}

void MockPosition::mockLatitude() {
    m_positionModel.latitude += std::cos(Utility::degreeToRadian(m_courseToSteer)) * LATUPDATE;
}

void MockPosition::mockLongitude() {
    m_positionModel.longitude += std::cos(Utility::degreeToRadian(m_courseToSteer)) * LONGUPDATE;
}
