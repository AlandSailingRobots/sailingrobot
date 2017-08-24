/****************************************************************************************
*
* File:
* 		MarineSensorNode.h
*
* Purpose:
*		 Reads the marine sensors and process the data at a data request from the rest of the system 
*
* Developer Notes:
*		 
*			
*
***************************************************************************************/




#pragma once


#include "Hardwares/AtlasScientificController/AtlasScientific.h"
#include "SystemServices/Timer.h"
#include "MessageBus/MessageBus.h"

class MarineSensorNode : public Node {
public:
	MarineSensorNode(MessageBus& msgBus, int miniWaitTime = 20);

	virtual ~MarineSensorNode(); 

	bool init();

	///----------------------------------------------------------------------------------
	/// Process the data requsest message.
	///----------------------------------------------------------------------------------
	void processMessage(const Message* msg);

	///----------------------------------------------------------------------------------
	/// Reads the senosors
	///----------------------------------------------------------------------------------
	bool readData(float& temp, float& conductivity, float& ph);

protected:

	
	Timer 		m_timer;
	int 		m_miniWaitTime;	// in seconds. Periode during witch the measuements are performed.
	float 		m_ph;
	float		m_conductivety;
	float		m_temp;
	float 		m_salidety;
	AtlasI2C 	m_I2C_TEMP;
	AtlasI2C 	m_I2C_COND;
	AtlasI2C 	m_I2C_PH;
	bool		m_Initialised;
};
