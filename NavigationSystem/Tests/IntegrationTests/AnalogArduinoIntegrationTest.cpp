#include <iostream>
#include <unistd.h> // close
#include "AnalogArduino/AR_UNO.h"
#include "AnalogArduino/AnalogArduino.h"
#include "AnalogArduino/MockAnalogArduino.h"

using namespace std;

int main(){
	cout<< "Pressuresensor  Example" << endl;
	cout<< "-----------------------" <<  endl;
  cout<< "creating object" << endl;
  AR_UNO p;
  //MockPressureSensor p;
  AnalogArduino* a_ardu = &p;
  cout<< "object created" << endl;

  cout << "initialization: " << a_ardu->init() << endl;

  while(true) {
		p.readValues();
    cout << "Pressure : "<< a_ardu->getValue0();
		cout << "  Rudder : "<< a_ardu->getValue1();
		cout << "  Sheet : "<< a_ardu->getValue2();
		cout << "  Battery : "<< a_ardu->getValue3()<< endl;
    usleep(100000);
  }
  return EXIT_SUCCESS;
}
