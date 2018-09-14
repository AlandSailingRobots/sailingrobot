/****************************************************************************************
 *
 * File:
 * 		PowerTrackNode.h
 *
 * Purpose:
 *      Collects voltage and current data from the actuators, ECUs, and solar
 *      panel to track the power, sends it to the logger.
 *		
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/


#ifndef NAVIGATIONSYSTEM_POWERTRACKNODE_H
#define NAVIGATIONSYSTEM_POWERTRACKNODE_H

#include <stdint.h>
#include "Database/DBHandler.h"
#include "Messages/CurrentSensorDataMsg.h"
#include "MessageBus/ActiveNode.h"

class PowerTrackNode : public ActiveNode {
    public:
    	PowerTrackNode(MessageBus& msgBus, DBHandler& dbhandler, double loopTime);
    	~PowerTrackNode();


    	///---------------------------------------------------------------------------
    	/// Initialises the server.
    	///---------------------------------------------------------------------------
    	bool init();

    	///---------------------------------------------------------------------------
    	/// Starts the PowerTrackNode thread to pump out PowerTrackMsg.
    	///---------------------------------------------------------------------------
    	void start();

    	void processMessage(const Message* msg);

    	///---------------------------------------------------------------------------
    	/// Stores CurrentSensor data from CurrentSensorDataMsg.
    	///---------------------------------------------------------------------------
    	void processCurrentSensorDataMessage(CurrentSensorDataMsg* msg);

    private:
    	///---------------------------------------------------------------------------
    	/// Begin the PowerTrackNode to get output from PowerTrackMsg as it contains
    	/// data collected from both the vessel's actuator sensors and arduino battery
    	///---------------------------------------------------------------------------
    	static void PowerTrackThreadFunc(ActiveNode* nodePtr);

    	float m_CurrentSensorDataCurrent;
    	float m_CurrentSensorDataVoltage;
    	SensedElement m_CurrentSensorDataElement;
        float m_Power;
        float m_PowerBalance;
        double m_Looptime;
        int m_lastElementRead;

    	DBHandler& m_db;

        const int POWER_STATE_INITIAL_SLEEP = 20000;


};




#endif //NAVIGATIONSYSTEM_POWERTRACK_H