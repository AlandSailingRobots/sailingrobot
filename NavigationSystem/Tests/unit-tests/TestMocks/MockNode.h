/****************************************************************************************
*
* File:
* 		MockNode.h
*
* Purpose:
*		A mock node for testing whether message passing is working correctly.
*
* Developer Notes:
*
*
***************************************************************************************/


#pragma once

//TODO - Jordan: Improve include paths so they aren't sooo long
#include "MessageBus/Node.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/WindStateMsg.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/NavigationControlMsg.h"
#include "Messages/DesiredCourseMsg.h"




class MockNode : public Node {
public:
	MockNode(MessageBus& msgBus, bool& registered)
	:Node(NodeID::MessageLogger, msgBus), m_MessageReceived(false), m_HasFix(false),
	m_Online(false), m_WindDir(0), m_WindSpeed(0), m_CourseToSteer(0.5), m_TargetSpeed(0),
	m_WindvaneSelfSteeringOn(0)
	{
		if(msgBus.registerNode(*this, MessageType::GPSData)
		&& msgBus.registerNode(*this, MessageType::WindData)
		&& msgBus.registerNode(*this, MessageType::WindState)
		&& msgBus.registerNode(*this, MessageType::CompassData)
		&& msgBus.registerNode(*this, MessageType::WaypointData)
		&& msgBus.registerNode(*this, MessageType::StateMessage)
		&& msgBus.registerNode(*this, MessageType::NavigationControl)
		&& msgBus.registerNode(*this, MessageType::ServerConfigsReceived)
		&& msgBus.registerNode(*this, MessageType::DesiredCourse)
		&& msgBus.registerNode(*this, MessageType::ActuatorPosition))
		{
			registered = true;
		}
	}


	virtual ~MockNode() {}

	bool init() { return true; }

	void processMessage(const Message* message)
	{
		MessageType type = message->messageType();

		switch(type)
		{
			case MessageType::GPSData:
			{
				m_MessageReceived = true;
				GPSDataMsg* gpsMsg = (GPSDataMsg*)message;
				m_HasFix = gpsMsg->hasFix();
				m_Online = gpsMsg->gpsOnline();
				m_Lat = gpsMsg->latitude();
				m_Lon = gpsMsg->longitude();
				m_UnixTime = gpsMsg->unixTime();
				m_Speed = gpsMsg->speed();
				m_Course = gpsMsg->heading();
				m_SatCount = gpsMsg->satelliteCount();
				m_Mode = gpsMsg->gpsMode();
			}
			break;
			case MessageType::WindData:
			{
				m_MessageReceived = true;
				WindDataMsg* windMsg = (WindDataMsg*)message;
				m_WindDir = windMsg->windDirection();
				m_WindSpeed = windMsg->windSpeed();
				m_WindTemp = windMsg->windTemp();
			}
			break;
			case MessageType::WindState:
			{
				m_MessageReceived = true;
				WindStateMsg* windState = (WindStateMsg*)message;
				m_trueWindDir = windState->trueWindSpeed();
				m_trueWindSpeed = windState->trueWindDirection();
				m_apparentWindSpeed = windState->apparentWindSpeed();
				m_apparentWindDir = windState->apparentWindDirection();
			}
			break;
			case MessageType::CompassData:
			{
				m_MessageReceived = true;
				CompassDataMsg* compassData = (CompassDataMsg*)message;
				m_compassHeading = compassData->heading();
				m_compassPitch = compassData->pitch();
				m_compassRoll = compassData->roll();
			}
			break;
			case MessageType::WaypointData:
			{
				m_MessageReceived = true;
				WaypointDataMsg* waypointData = (WaypointDataMsg*)message;
				m_waypointNextId = waypointData->nextId();
				m_waypointNextLongitude = waypointData->nextLongitude();
				m_waypointNextLatitude = waypointData->nextLatitude();
				m_waypointNextDeclination = waypointData->nextDeclination();
				m_waypointNextRadius = waypointData->nextRadius();
				m_waypointStayTime = waypointData->stayTime();
				m_waypointPrevId = waypointData->prevId();
				m_waypointPrevLongitude = waypointData->prevLongitude();
				m_waypointPrevLatitude = waypointData->prevLatitude();
				m_waypointPrevDeclination = waypointData->prevDeclination();
				m_waypointPrevRadius = waypointData->prevRadius();

			}
			break;
			case MessageType::StateMessage:
			{
				m_MessageReceived = true;
				StateMessage* stateMsg = (StateMessage*)message;
				m_StateMsgHeading = stateMsg->heading();
				m_StateMsgLat = stateMsg->latitude();
				m_StateMsgLon = stateMsg->longitude();
				m_StateMsgSpeed = stateMsg->speed();
				m_StateMsgCourse = stateMsg->course();
			}
			break;
			case MessageType::ActuatorPosition:
			{
				m_MessageReceived = true;
				ActuatorPositionMsg* actuatorMsg = (ActuatorPositionMsg*)message;
				m_rudderPosition = actuatorMsg->rudderPosition();
				m_sailPosition = actuatorMsg->sailPosition();
			}
			break;
			case MessageType::NavigationControl:
			{
				m_MessageReceived = true;
				NavigationControlMsg* navigationControlMsg = (NavigationControlMsg*)message;
				m_CourseToSteer = navigationControlMsg->courseToSteer();
				m_TargetSpeed = navigationControlMsg->targetSpeed();
				m_WindvaneSelfSteeringOn = navigationControlMsg->windvaneSelfSteeringOn();
			}
			break;
			case MessageType::ServerConfigsReceived:
			{
				m_MessageReceived = true;
			}
			break;
			case MessageType::DesiredCourse:
			{
				m_MessageReceived = true;
				DesiredCourseMsg* desiredCourseMsg = (DesiredCourseMsg*)message;
				m_DesiredCourse = desiredCourseMsg->desiredCourse();
			}
			break;
			default:
			return;
		}
	}

	bool 	m_MessageReceived;

	  //GPSData variables
//=========================
	bool	m_HasFix;
	bool	m_Online;
	double 	m_Lat;
	double 	m_Lon;
	double 	m_UnixTime;
	double 	m_Speed;
	double 	m_Course;
	int 	m_SatCount;
	GPSMode m_Mode;

	  //WindData variables
//=========================
	float 	m_WindDir;
	float 	m_WindSpeed;
	float 	m_WindTemp;
	float 	m_Heading;

	//WindState variables
//=========================
	double m_trueWindDir;
	double m_trueWindSpeed;
	double m_apparentWindSpeed;
	double m_apparentWindDir;

	//CompassData variables
//=========================
	double m_compassHeading;
	double m_compassPitch;
	double m_compassRoll;

	//WaypointData variables
//=========================
	int 	m_waypointNextId;
	double	m_waypointNextLongitude;
	double 	m_waypointNextLatitude;
	int 	m_waypointNextDeclination;
	int 	m_waypointNextRadius;
	int 	m_waypointStayTime;
	int 	m_waypointPrevId;
	double 	m_waypointPrevLongitude;
	double 	m_waypointPrevLatitude;
	int 	m_waypointPrevDeclination;
	int 	m_waypointPrevRadius;

    //StateMessage variables
//=========================
	float 	m_StateMsgHeading;
	double	m_StateMsgLat;
	double	m_StateMsgLon;
	double	m_StateMsgSpeed;
	double  m_StateMsgCourse;

	  //ActuatorPosition variables
//=========================
	int m_rudderPosition;
	int m_sailPosition;

	//ActuatorPosition variables
//=========================
	int16_t m_DesiredCourse;

	  //NavigationControlMsg variables
//=========================
	int m_CourseToSteer;
	int m_TargetSpeed;
	int m_WindvaneSelfSteeringOn;

};
