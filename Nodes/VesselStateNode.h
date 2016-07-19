/****************************************************************************************
 *
 * File:
 * 		VesselStateNode.h
 *
 * Purpose:
 *		Maintains the "current" state of the vessel. Collects data from sensor messages
 *		and then resends a collected copy of that data back out for further processing.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

 #pragma once

#include "ActiveNode.h"


class VesselStateNode : public ActiveNode {
public:
	VesselStateNode(MessageBus& msgBus);

	///----------------------------------------------------------------------------------
	///
	///----------------------------------------------------------------------------------
	bool init() { return true; }

	///----------------------------------------------------------------------------------
 	/// Starts the VesselStateNode's thread that pumps out VesselStateMsg's
 	///----------------------------------------------------------------------------------
	void start();

	void processMessage(const Message* msg);

private:
	static void VesselStateThreadFunc(void* nodePtr);
};
