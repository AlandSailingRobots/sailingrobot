/* 
 * File:   main.cpp
 * Author: Joakim
 *
 * Created on 20 March 2014, 09:25
 */

/*
MAX 7120
MID 5520
MIN 3920

sheet
0 4215
end 5294

*/


#include <unistd.h>

#include <cstdlib>
#include <iostream>

#include "servocontroller/ServoObject.h"
#include "servocontroller/SensorObject.h"
#include "servocontroller/MaestroController.h"
#include "AnalogArduino/AR_UNO.h"

#define MIDDLE 5520

void servoExample() {
  MaestroController* maestro = new MaestroController();
  ServoObject* servo = new ServoObject();
  AR_UNO arduino;
  AnalogArduino* a_ardu = &arduino;
  a_ardu->init();

  try {
    maestro->setPort("/dev/ttyACM0");
    
    servo->setController(maestro);

    // Ruder=4, Sheet=3
    servo->setChannel(4);
    while(true) {
        int position, fbPosition, width, wiggles, thresh, i=0, dir=1;
        std::cout << "Enter wiggle width (between 1 and 1600):";
        std::cin >> width;
        std::cout << "Enter number of times to wiggle (from 2 up):";
        std::cin >> wiggles;
        std::cout << "Enter threshold for wiggle error:";
        std::cin >> thresh;
        while (i<wiggles)
        {
          position = dir*width+MIDDLE;
          servo->setPosition(position);
          
          arduino.readValues();
          fbPosition = (arduino.getValue1()-285)*1600/235+MIDDLE;
          std::cout << "#Servo position> " << fbPosition << 
          "|| Desired position> " << position << std::endl;

          if (abs(position-fbPosition)<thresh)
          {
            dir=-dir;
            i++;
          }
        }

    }
  } catch(const char* error) {
      std::cout << error << "\n";
  }
  delete servo;
  delete maestro;
}


void sensorExample() {
  MaestroController* maestro = new MaestroController();
  SensorObject* sensor = new SensorObject();
  try {
    maestro->setPort("/dev/ttyACM0");
    sensor->setController(maestro);
    sensor->setChannel(5);
    while(true) {
        int position;
        std::cin >> position;
        std::cout << sensor->getDirection() << std::endl;
    }
  } catch(const char* error) {
      std::cout << error << "\n";
  }
  delete sensor;
  delete maestro;
}

int main(int argc, char** argv) {

  servoExample();
  //sensorExample();
  return 0;

}
