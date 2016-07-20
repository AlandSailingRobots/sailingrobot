/*
 * File:   ActuatorNode.h
 * Author: pophaax
 *
 * Mostly a rewrite of servoObject
 *
 */

#ifndef __ACTUATORNODE_H__
#define	__ACTUATORNODE_H__

#include <memory>
#include "MaestroController.h"

class ActuatorNode {
public:

	ActuatorNode();
	~ActuatorNode();

	void processMessage(const Message* message);
    void processPositionData(ActuatorPositionMessage* msg);
	bool init();

private:
	int m_channel;
	int m_range;
	int m_speed;
	int m_acceleration;

	std::shared_ptr<Actuator> m_maestro;

};

#endif
