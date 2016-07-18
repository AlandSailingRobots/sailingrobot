#include "ActuatorNode.h"


//m_maestro might change
//removed setmaestro and set channel, now set in constructor
ActuatorNode::ActuatorNode(NodeID id, MessageBus &msgBus, Actuator* maestro, int channel) 
        : Node(id, msgBus), m_maestro.reset(maestro), m_channel(channel)
    {}

ActuatorNode::~ActuatorNode() {

}

ActuatorNode::init(){
    
}

//Mostly done, given we don't need to change acceleration or speed on the go
void ActuatorNode::processMessage(const Message* message){
    MessageType msgType = message->getType();

    switch(msgType){
        case MessageType::ActuatorPosition:
            processPositionData( (ActuatorPositionMessage*)message );
            break;
    }
}

void ActuatorNode::processPositionData(ActuatorPositionMessage* msg){

    m_maestro->writeCommand(SET_POSITION, m_channel, msg.position());
}


// void ActuatorNode::setSpeed(unsigned short speed) {
// 	m_speed = speed;
// 	m_maestro->writeCommand(SET_SPEED, m_channel, speed);
// }

// void ActuatorNode::setAcceleration(unsigned short acceleration) {
// 	m_acceleration = acceleration;
// 	m_maestro->writeCommand(SET_ACCELERATION, m_channel, acceleration);
// }

// void ActuatorNode::setPosition(unsigned short position) {
// 	m_maestro->writeCommand(SET_POSITION, m_channel, position);

// }

//Port to MAESTRO class?
// int ActuatorNode::getPosition() {
// 	m_maestro->writeCommand(GET_POSITION, m_channel, -1);
// 	return m_maestro->readRespons();
// }

