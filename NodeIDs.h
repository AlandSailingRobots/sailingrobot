/****************************************************************************************
 *
 * File:
 * 		NodeIDs.h
 *
 * Purpose:
 *		Contains all the Node IDs. A NodeID is a logically name ot describe what a node
 *		does. ITs primary purpose is to allow nodes to send messages to one another 
 *		directly instead of via the subscription system. It is also used for message 
 *		tracking purposes.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#pragma once
 

enum class NodeID {
	None = 0,
	MessageLogger,
	WindSensor,
	Compass,
	GPS,
	VesselState
};
