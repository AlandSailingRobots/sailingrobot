/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   AccelerationModel.h
 * Author: sailbot
 *
 * Created on April 19, 2016, 12:15 PM
 */

#ifndef ACCELERATIONMODEL_H
#define ACCELERATIONMODEL_H

class AccelerationModel
{
public:
    AccelerationModel(int accelX,int accelY, int accelZ) :
            accelX(accelX),
            accelY(accelY),
            accelZ(accelZ)
    {};

    ~AccelerationModel() {};

        int accelX;
        int accelY; 
        int accelZ;
};
#endif /* ACCELERATIONMODEL_H */

