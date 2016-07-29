/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Position.h
 * Author: sailbot
 *
 * Created on April 27, 2016, 11:46 AM
 */

#ifndef POSITION_H
#define POSITION_H

#include "models/SystemStateModel.h"
#include "models/PositionModel.h"

class Position {
public:
    Position() {}
    virtual ~Position() {}
    
    /* Only used in mock */
    virtual void setHeading(int heading) = 0;
    
    /* Only used in mock */
    virtual void setCourseToSteer(double cts) = 0;
    
    /* Only used in mock */
    virtual int getHeading() = 0;
    
    virtual void updatePosition() = 0;
    
    virtual PositionModel getModel() = 0;
}; 
#endif /* POSITION_H */

