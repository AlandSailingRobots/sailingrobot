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

#include "RealPosition.h"

RealPosition::RealPosition(const SystemStateModel& systemStateModel) : 
                            m_systemStateModel(systemStateModel),
                            m_positionModel(0.0,0.0){
    
}

void RealPosition::setHeading(int heading) { }
    
void RealPosition::setCourseToSteer(double cts) {  }

int RealPosition::getHeading() { return 0; }
    
void RealPosition::updatePosition() {
    m_positionModel.longitude =
            m_systemStateModel.gpsModel.positionModel.longitude;
    
    m_positionModel.latitude =
            m_systemStateModel.gpsModel.positionModel.latitude;    
}

PositionModel RealPosition::getModel() {
    return m_positionModel;
}

RealPosition::~RealPosition() {
}

