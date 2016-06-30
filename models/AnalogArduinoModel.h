#ifndef __ANALOGARDUINOMODEL_H__
#define __ANALOGARDUINOMODEL_H__

class AnalogArduinoModel
{
public:
	AnalogArduinoModel(int analogValue0,int analogValue1,int analogValue2,int analogValue3) :
		analogValue0(analogValue0),
		analogValue1(analogValue1),
		analogValue2(analogValue2),
		analogValue3(analogValue3)
	{};

	~AnalogArduinoModel() {};

	int analogValue0;
	int analogValue1;
	int analogValue2;
	int analogValue3;
};

#endif
