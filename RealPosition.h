/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RealPosition.h
 * Author: sailbot
 *
 * Created on April 27, 2016, 11:47 AM
 */

#ifndef REALPOSITION_H
#define REALPOSITION_H

#include "Position.h"

class RealPosition : public Position {
public:
    RealPosition(const SystemStateModel& systemStateModel);
    ~RealPosition();
    
    void setHeading(int heading);
    
    void setCourseToSteer(double cts);
    
    int getHeading();
    
    void updatePosition();
    
    PositionModel getModel();
    
private:
    const SystemStateModel& m_systemStateModel;
    PositionModel m_positionModel;
};
#endif /* REALPOSITION_H */

