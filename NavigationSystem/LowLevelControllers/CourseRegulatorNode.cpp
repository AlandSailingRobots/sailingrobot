/****************************************************************************************
 *
 * File:
 * 		CourseRegulatorNode.cpp
 *
 * Purpose:
 *      This file realize the regulation of the rudder by the desired course.
 *      It sends the value to the rudder actuator Node
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/
#include "CourseRegulatorNode.h"
#include <chrono>
#include <thread>
#include <math.h>
//#include <cstdlib>
#include "Math/Utility.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/StateMessage.h"
#include "SystemServices/Logger.h"
#include "SystemServices/Timer.h"

#define HEADING_ERROR_VALUE 370

CourseRegulatorNode::CourseRegulatorNode( MessageBus& msgBus,  DBHandler& dbhandler, double loopTime, double maxRudderAngle,
    double configPGain, double configIGain):ActiveNode(NodeID::CourseRegulatorNode,msgBus), m_VesselHeading(HEADING_ERROR_VALUE),
    m_MaxRudderAngle(maxRudderAngle),m_DesiredHeading(HEADING_ERROR_VALUE),m_db(dbhandler), m_LoopTime(loopTime),pGain(configPGain),iGain(configIGain) 
{
    msgBus.registerNode( *this, MessageType::StateMessage);
    msgBus.registerNode( *this, MessageType::DesiredCourse);
    msgBus.registerNode( *this, MessageType::NavigationControl);
} 

///----------------------------------------------------------------------------------
CourseRegulatorNode::~CourseRegulatorNode(){}

///----------------------------------------------------------------------------------
bool CourseRegulatorNode::init(){ return true;}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::start()
{
    runThread(CourseRegulatorNodeThreadFunc);
}

///----------------------------------------------------------------------------------
double CourseRegulatorNode::getFrequencyThread()
{
    return m_LoopTime;
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::processMessage( const Message* msg )
{
    MessageType type = msg->messageType();
    //std::cout << std::endl << " COURSE REG NODE ####### Message processing ######## " << msgToString(type) << std::endl ;
    switch(type)
    {
        case MessageType::StateMessage:
        processStateMessage(static_cast< const StateMessage*>(msg));
        break;
        case MessageType::DesiredCourse:
        processDesiredCourseMessage(static_cast< const DesiredCourseMsg*>(msg)); //verify
        break;
        case MessageType::NavigationControl:
        processNavigationControlMessage(static_cast< const NavigationControlMsg*>(msg));
        break;
        default:
        return;
        //Logger::info("Desired Course: %d Heading: %d", desiredHeading, heading);
    }
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::processStateMessage(const StateMessage* msg)
{
    //std::lock_guard<std::mutex> lock_guard(m_lock);
    //std::cout << std::endl << "COURSE REG NODE ########## In process ######## " << std::endl; 
    m_VesselHeading = msg->heading();
    // Not use
    //m_VesselLatitude = msg->latitude();
    //m_VesselLongitude = msg->longitude();
    m_VesselSpeed = msg->speed(); 
    //m_VesselCourse = msg->course();
    //std::cout << std::endl << "COURSE REG NODE ######### Receive State Message : H " << msg->heading() << " , V " << msg->speed();
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::processDesiredCourseMessage(const DesiredCourseMsg* msg)
{
    //std::lock_guard<std::mutex> lock_guard(m_lock);
    m_DesiredHeading = static_cast<double>(msg->desiredCourse());
    //std::cout << std::endl << "COURSE REG NODE ######### Receive DesiredCourse Message : DC " << msg->desiredCourse();
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::processNavigationControlMessage(const NavigationControlMsg* msg)
{
    //std::lock_guard<std::mutex> lock_guard(m_lock);
    m_NavigationState = msg->navigationState();
    // Not use
    //m_CourseToSteer = msg->courseToSteer();
    //m_TargetSpeed = msg->targetSpeed();
    //Windvane?
    //m_Tack = msg->tack(); // useful ?
    // No definition for the starboard
}

///----------------------------------------------------------------------------------
double CourseRegulatorNode::calculateRudderAngle()
{
    if((m_DesiredHeading != HEADING_ERROR_VALUE) and (m_VesselHeading != HEADING_ERROR_VALUE)){

        double difference_Heading = Utility::limitAngleRange(m_VesselHeading) - Utility::limitAngleRange(m_DesiredHeading);
        //std::cout << std::endl << "COURSE REG NODE ######### Calc : DiffH " << difference_Heading;
        // Equation from book "Robotic Sailing 2015 ", page 141
        // The MAX_RUDDER_ANGLE is a parameter configuring the variation around the desired heading.
        // Also the reaction could be configure by the frequence of the thread.
        if(cos(Utility::degreeToRadian(difference_Heading)) < 0) // Wrong sense because over 90°
        {
            // Max Rudder angle in the opposite way 
            return Utility::sgn(m_VesselSpeed)*(Utility::sgn(sin(Utility::degreeToRadian(difference_Heading))))*m_MaxRudderAngle;
        }
        // Regulation of the rudder
        return Utility::sgn(m_VesselSpeed)*(sin(Utility::degreeToRadian(difference_Heading))*m_MaxRudderAngle);
    
    }
    return 0;
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::updateFrequencyThread()
{
    m_LoopTime = m_db.retrieveCellAsDouble("course_regulator","1","loopTime");
}

///----------------------------------------------------------------------------------
void CourseRegulatorNode::CourseRegulatorNodeThreadFunc(ActiveNode* nodePtr)
{
    CourseRegulatorNode* node = dynamic_cast<CourseRegulatorNode*> (nodePtr);
    
    // An initial sleep, its purpose is to ensure that most if not all the sensor data arrives
    // at the start before we send out the state message.
    std::this_thread::sleep_for(std::chrono::milliseconds(node->STATE_INITIAL_SLEEP));

    Timer timer;
    timer.start();

    while(true)
    {
        std::lock_guard<std::mutex> lock_guard(node->m_lock);
        // TODO : Modify Actuator Message for adapt to this Node
        MessagePtr actuatorMessage = std::make_unique<ActuatorPositionMsg>(node->calculateRudderAngle(),0);
        //std::cout << std::endl << "COURSE REG NODE ######### Send : CalcR " << node->calculateRudderAngle();
        node->m_MsgBus.sendMessage(std::move(actuatorMessage));
    
        // Broadcast() or selected sent???
        timer.sleepUntil(node->m_LoopTime); //insert updateFrequencyThread in the function ?
        timer.reset();
        //node->updateFrequencyThread(); 
    }
}
