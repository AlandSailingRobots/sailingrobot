/****************************************************************************************
 *
 * File:
 * 		CANService.h
 *
 * Purpose:
 *		The message bus manages message distribution to nodes allowing nodes to
 *		communicate with one another.
 *
 * Developer Notes:
 *		Nodes can only be added before the run function is called currently. This is to
 *		reduce the number of thread locks in place and because once the system has
 *		started its very rare that a node should be registered afterwards on the fly.
 *
 *
 ***************************************************************************************/

class CANService {

};
