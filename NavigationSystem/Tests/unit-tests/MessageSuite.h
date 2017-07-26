/****************************************************************************************
 *
 * File:
 * 		MessageSuite.h
 *
 * Purpose:
 *		A set of unit tests for ensuring messages are created correctly.
 *
 * Developer Notes:
 *
 *							12.4.17 JM
 *
 *	Functions that have tests:		Functions that does not have tests:
 *
 *	All Message-functions have tests
 *
 ***************************************************************************************/

#pragma once


#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/CompassDataMsg.h"
#include "Messages/GPSDataMsg.h"
#include "Messages/WindDataMsg.h"
#include "Messages/DataRequestMsg.h"
#include "Messages/WaypointDataMsg.h"
#include "Messages/ActuatorPositionMsg.h"
#include "Messages/ArduinoDataMsg.h"
#include "Messages/VesselStateMsg.h"
#include "Messages/CourseDataMsg.h"
#include "Messages/ServerConfigsReceivedMsg.h"
#include "Messages/ServerWaypointsReceivedMsg.h"
#include "Messages/LocalConfigChangeMsg.h"
#include "Messages/LocalWaypointChangeMsg.h"
#include "Messages/StateMessage.h"
#include "Messages/SolarDataMsg.h"
#include "Messages/AISDataMsg.h"
// #include "Hardwares/CANAISNode.h"


class MessageSuite : public CxxTest::TestSuite {
public:
	void setUp() { }

	void tearDown() { }

	void test_CompassDataMsg()
	{
		CompassDataMsg msg(100, 80, 60);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::CompassData);
		TS_ASSERT_EQUALS(msg.heading(), 100);
		TS_ASSERT_EQUALS(msg.pitch(), 80);
		TS_ASSERT_EQUALS(msg.roll(), 60);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		CompassDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::CompassData);
		TS_ASSERT_EQUALS(msgTwo.heading(), 100);
		TS_ASSERT_EQUALS(msgTwo.pitch(), 80);
		TS_ASSERT_EQUALS(msgTwo.roll(), 60);
	}

	void test_GPSDataMsg()
	{
		GPSDataMsg msg(true, false, 52.0, 48.2, 1993.6, 5.1, 100.8, 11, GPSMode::NoFix);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::GPSData);
		TS_ASSERT_EQUALS(msg.hasFix(), true);
		TS_ASSERT_EQUALS(msg.gpsOnline(), false);
		TS_ASSERT_DELTA(msg.latitude(), 52.0, 1e-7);
		TS_ASSERT_DELTA(msg.longitude(), 48.2, 1e-7);
		TS_ASSERT_DELTA(msg.unixTime(), 1993.6, 1e-7);
		TS_ASSERT_DELTA(msg.speed(), 5.1, 1e-7);
		TS_ASSERT_DELTA(msg.heading(), 100.8, 1e-7);
		TS_ASSERT_EQUALS(msg.satelliteCount(), 11);
		TS_ASSERT_EQUALS(msg.gpsMode(), GPSMode::NoFix);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		GPSDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::GPSData);
		TS_ASSERT_EQUALS(msgTwo.hasFix(), true);
		TS_ASSERT_EQUALS(msgTwo.gpsOnline(), false);
		TS_ASSERT_DELTA(msgTwo.latitude(), 52.0, 1e-7);
		TS_ASSERT_DELTA(msgTwo.longitude(), 48.2, 1e-7);
		TS_ASSERT_DELTA(msgTwo.unixTime(), 1993.6, 1e-7);
		TS_ASSERT_DELTA(msgTwo.speed(), 5.1, 1e-7);
		TS_ASSERT_DELTA(msgTwo.heading(), 100.8, 1e-7);
		TS_ASSERT_EQUALS(msgTwo.satelliteCount(), 11);
		TS_ASSERT_EQUALS(msgTwo.gpsMode(), GPSMode::NoFix);
	}

	void test_WindDataMsg()
	{
		WindDataMsg msg(100.5f, 40.2f, 20.8f);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::WindData);
		TS_ASSERT_DELTA(msg.windDirection(), 100.5f, 1e-7);
		TS_ASSERT_DELTA(msg.windSpeed(), 40.2f, 1e-7);
		TS_ASSERT_DELTA(msg.windTemp(), 20.8f, 1e-7);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		WindDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::WindData);
		TS_ASSERT_DELTA(msgTwo.windDirection(), 100.5f, 1e-7);
		TS_ASSERT_DELTA(msgTwo.windSpeed(), 40.2f, 1e-7);
		TS_ASSERT_DELTA(msgTwo.windTemp(), 20.8f, 1e-7);
	}

	void test_DataRequestMsg()
	{
		DataRequestMsg msg(NodeID::GPS, NodeID::MessageLogger);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::DataRequest);
		TS_ASSERT_EQUALS(msg.sourceID(), NodeID::MessageLogger);
		TS_ASSERT_EQUALS(msg.destinationID(), NodeID::GPS);
	}

	void test_WaypointDataMsg()
	{
		WaypointDataMsg msg(2, 19.81, 60.2, 0, 6, 15,  1, 19.82, 60.1, 6, 15);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::WaypointData);
		TS_ASSERT_EQUALS(msg.nextId(), 2);
		TS_ASSERT_DELTA(msg.nextLongitude(), 19.81, 1e-7);
		TS_ASSERT_DELTA(msg.nextLatitude(), 60.2, 1e-7);
		TS_ASSERT_EQUALS(msg.nextDeclination(), 0);
		TS_ASSERT_EQUALS(msg.nextRadius(), 6);
		TS_ASSERT_EQUALS(msg.stayTime(), 15);
		TS_ASSERT_EQUALS(msg.prevId(), 1);
		TS_ASSERT_DELTA(msg.prevLongitude(), 19.82, 1e-7);
		TS_ASSERT_DELTA(msg.prevLatitude(), 60.1, 1e-7);
		TS_ASSERT_EQUALS(msg.prevDeclination(), 6);
		TS_ASSERT_EQUALS(msg.prevRadius(), 15);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		WaypointDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::WaypointData);
		TS_ASSERT_EQUALS(msgTwo.nextId(), 2);
		TS_ASSERT_DELTA(msgTwo.nextLongitude(), 19.81, 1e-7);
		TS_ASSERT_DELTA(msgTwo.nextLatitude(), 60.2, 1e-7);
		TS_ASSERT_EQUALS(msgTwo.nextDeclination(), 0);
		TS_ASSERT_EQUALS(msgTwo.nextRadius(), 6);
		TS_ASSERT_EQUALS(msgTwo.stayTime(), 15);
		TS_ASSERT_EQUALS(msgTwo.prevId(), 1);
		TS_ASSERT_DELTA(msgTwo.prevLongitude(), 19.82, 1e-7);
		TS_ASSERT_DELTA(msgTwo.prevLatitude(), 60.1, 1e-7);
		TS_ASSERT_EQUALS(msgTwo.prevDeclination(), 6);
		TS_ASSERT_EQUALS(msgTwo.prevRadius(), 15);
	}

	void test_ActuatorPositionMsg()
	{
		ActuatorPositionMsg msg(5507, 4765);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::ActuatorPosition);
		TS_ASSERT_EQUALS(msg.rudderPosition(), 5507);
		TS_ASSERT_EQUALS(msg.sailPosition(), 4765);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		ActuatorPositionMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::ActuatorPosition);
		TS_ASSERT_EQUALS(msgTwo.rudderPosition(), 5507);
		TS_ASSERT_EQUALS(msgTwo.sailPosition(), 4765);
	}

	void test_ArduinoDataMsg()
	{
		ArduinoDataMsg msg(10, 5500, 4700, 2, 3);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::ArduinoData);
		TS_ASSERT_EQUALS(msg.pressure(), 10);
		TS_ASSERT_EQUALS(msg.rudder(), 5500);
		TS_ASSERT_EQUALS(msg.sheet(), 4700);
		TS_ASSERT_EQUALS(msg.battery(), 2);
		TS_ASSERT_EQUALS(msg.Radio_Controller(), 3);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		ArduinoDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::ArduinoData);
		TS_ASSERT_EQUALS(msgTwo.pressure(), 10);
		TS_ASSERT_EQUALS(msgTwo.rudder(), 5500);
		TS_ASSERT_EQUALS(msgTwo.sheet(), 4700);
		TS_ASSERT_EQUALS(msgTwo.battery(), 2);
		TS_ASSERT_EQUALS(msgTwo.Radio_Controller(), 3);
	}

	void test_VesselStateMsg()
	{
		VesselStateMsg msg(170, 30, 0, true, true, 19.2, 60.02, 120.04, 2.1, 11, 170.5, 23.5f, 5.4f, 24.5f, 10, 5500, 4700, 2, 3);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::VesselState);
		TS_ASSERT_EQUALS(msg.compassHeading(), 170);
		TS_ASSERT_EQUALS(msg.compassPitch(), 30);
		TS_ASSERT_EQUALS(msg.compassRoll(), 0);
		TS_ASSERT_EQUALS(msg.gpsHasFix(), true);
		TS_ASSERT_EQUALS(msg.gpsOnline(), true);
		TS_ASSERT_DELTA(msg.latitude(), 19.2, 1e-7);
		TS_ASSERT_DELTA(msg.longitude(), 60.02, 1e-7);
		TS_ASSERT_DELTA(msg.unixTime(), 120.04, 1e-7);
		TS_ASSERT_DELTA(msg.speed(), 2.1, 1e-7);
		TS_ASSERT_EQUALS(msg.gpsSatellite(), 11);
		TS_ASSERT_DELTA(msg.gpsHeading(), 170.5, 1e-7);
		TS_ASSERT_DELTA(msg.windDir(), 23.5f, 1e-7);
		TS_ASSERT_DELTA(msg.windSpeed(), 5.4f, 1e-7);
		TS_ASSERT_DELTA(msg.windTemp(), 24.5f, 1e-7);
		TS_ASSERT_EQUALS(msg.arduinoPressure(), 10);
		TS_ASSERT_EQUALS(msg.arduinoRudder(), 5500);
		TS_ASSERT_EQUALS(msg.arduinoSheet(), 4700);
		TS_ASSERT_EQUALS(msg.arduinoBattery(), 2);
		TS_ASSERT_EQUALS(msg.arduinoRC(), 3);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		VesselStateMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::VesselState);
		TS_ASSERT_EQUALS(msgTwo.compassHeading(), 170);
		TS_ASSERT_EQUALS(msgTwo.compassPitch(), 30);
		TS_ASSERT_EQUALS(msgTwo.compassRoll(), 0);
		TS_ASSERT_EQUALS(msgTwo.gpsHasFix(), true);
		TS_ASSERT_EQUALS(msgTwo.gpsOnline(), true);
		TS_ASSERT_DELTA(msgTwo.latitude(), 19.2, 1e-7);
		TS_ASSERT_DELTA(msgTwo.longitude(), 60.02, 1e-7);
		TS_ASSERT_DELTA(msgTwo.unixTime(), 120.04, 1e-7);
		TS_ASSERT_DELTA(msgTwo.speed(), 2.1, 1e-7);
		TS_ASSERT_EQUALS(msgTwo.gpsSatellite(), 11);
		TS_ASSERT_DELTA(msgTwo.gpsHeading(), 170.5, 1e-7);
		TS_ASSERT_DELTA(msgTwo.windDir(), 23.5f, 1e-7);
		TS_ASSERT_DELTA(msgTwo.windSpeed(), 5.4f, 1e-7);
		TS_ASSERT_DELTA(msgTwo.windTemp(), 24.5f, 1e-7);
		TS_ASSERT_EQUALS(msgTwo.arduinoPressure(), 10);
		TS_ASSERT_EQUALS(msgTwo.arduinoRudder(), 5500);
		TS_ASSERT_EQUALS(msgTwo.arduinoSheet(), 4700);
		TS_ASSERT_EQUALS(msgTwo.arduinoBattery(), 2);
		TS_ASSERT_EQUALS(msg.arduinoRC(), 3);
	}

		void test_CourseDataMsg()
	{
		CourseDataMsg msg(30.1f, 40.2f, 50.8f);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::CourseData);
		TS_ASSERT_DELTA(msg.trueWindDir(), 30.1f, 1e-7);
		TS_ASSERT_DELTA(msg.distanceToWP(), 40.2f, 1e-7);
		TS_ASSERT_DELTA(msg.courseToWP(), 50.8f, 1e-7);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		CourseDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::CourseData);
		TS_ASSERT_DELTA(msgTwo.trueWindDir(), 30.1f, 1e-7);
		TS_ASSERT_DELTA(msgTwo.distanceToWP(), 40.2f, 1e-7);
		TS_ASSERT_DELTA(msgTwo.courseToWP(), 50.8f, 1e-7);
	}

	void test_ServerConfigsReceivedMsg()
	{
		ServerConfigsReceivedMsg msg;

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::ServerConfigsReceived);
		TS_ASSERT_EQUALS(msg.sourceID(), NodeID::None);
		TS_ASSERT_EQUALS(msg.destinationID(), NodeID::None);
	}

	void test_ServerWaypointsReceivedMsg()
	{
		ServerWaypointsReceivedMsg msg(NodeID::None, NodeID::HTTPSync);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::ServerWaypointsReceived);
		TS_ASSERT_EQUALS(msg.sourceID(), NodeID::HTTPSync);
		TS_ASSERT_EQUALS(msg.destinationID(), NodeID::None);
	}

	void test_LocalConfigChangeMsg()
	{
		LocalConfigChangeMsg msg;

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::LocalConfigChange);
		TS_ASSERT_EQUALS(msg.sourceID(), NodeID::None);
		TS_ASSERT_EQUALS(msg.destinationID(), NodeID::None);
	}

	void test_LocalWaypointChangeMsg()
	{
		LocalWaypointChangeMsg msg(NodeID::None, NodeID::HTTPSync);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::LocalWaypointChange);
		TS_ASSERT_EQUALS(msg.sourceID(), NodeID::HTTPSync);
		TS_ASSERT_EQUALS(msg.destinationID(), NodeID::None);
	}

	void test_StateDataMsg()
	{
		StateMessage msg(100, 80, 60, 5, 30);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::StateMessage);
		TS_ASSERT_EQUALS(msg.heading(), 100);
		TS_ASSERT_EQUALS(msg.latitude(), 80);
		TS_ASSERT_EQUALS(msg.longitude(), 60);
		TS_ASSERT_EQUALS(msg.speed(), 5);
		TS_ASSERT_EQUALS(msg.course(), 30);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		StateMessage msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::StateMessage);
		TS_ASSERT_EQUALS(msgTwo.heading(), 100);
		TS_ASSERT_EQUALS(msgTwo.latitude(), 80);
		TS_ASSERT_EQUALS(msgTwo.longitude(), 60);
		TS_ASSERT_EQUALS(msgTwo.speed(), 5);
		TS_ASSERT_EQUALS(msgTwo.course(), 30);
	}

	void test_SolarDataMsg() {
		SolarDataMsg msg(60.2f, 19.1f, 200.5f, 13, 25);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::SolarData);
		TS_ASSERT_DELTA(msg.latitude(),60.2f,1e-7);
		TS_ASSERT_DELTA(msg.longitude(),19.1f,1e-7);
		TS_ASSERT_DELTA(msg.heading(),200.5f,1e-7);
		TS_ASSERT_EQUALS(msg.hour(),13);
		TS_ASSERT_EQUALS(msg.min(), 25);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(),serialiser.size());
		SolarDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::SolarData);
		TS_ASSERT_DELTA(msgTwo.latitude(),60.2f,1e-7);
		TS_ASSERT_DELTA(msgTwo.longitude(),19.1f,1e-7);
		TS_ASSERT_DELTA(msgTwo.heading(),200.5f,1e-7);
		TS_ASSERT_EQUALS(msgTwo.hour(),13);
		TS_ASSERT_EQUALS(msgTwo.min(), 25);
	}

	void test_AISDataMsg() {
		std::vector<AISVessel> AISList;
		AISVessel v1, v2, v3;
		v1.MMSI = 1;
		v1.latitude = 60.2f;
		v1.longitude = 19.1f;
		v1.COG = 200;
		v1.SOG = 10;
		v2.MMSI = 2;
		v2.latitude = 62.f;
		v2.longitude = 18.1f;
		v2.COG = 100;
		v2.SOG = 5;
		v3.MMSI = 3;
		v3.latitude = 61.5f;
		v3.longitude = 18.7f;
		v3.COG = 80;
		v3.SOG = 7;
		AISList.push_back(v1);
		AISList.push_back(v2);
		AISList.push_back(v3);

		AISDataMsg msg(AISList);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::AISData);
		TS_ASSERT_EQUALS(msg.MMSI(0),1);
		TS_ASSERT_DELTA(msg.latitude(0), 60.2f, 1e-7);
		TS_ASSERT_DELTA(msg.longitude(0), 19.1f, 1e-7);
		TS_ASSERT_EQUALS(msg.COG(0), 200);
		TS_ASSERT_EQUALS(msg.SOG(0), 10);
		TS_ASSERT_EQUALS(msg.MMSI(1), 2);
		TS_ASSERT_DELTA(msg.latitude(1), 62.f, 1e-7);
		TS_ASSERT_DELTA(msg.longitude(1), 18.1f, 1e-7);
		TS_ASSERT_EQUALS(msg.COG(1), 100);
		TS_ASSERT_EQUALS(msg.SOG(1), 5);
		TS_ASSERT_EQUALS(msg.MMSI(2), 3);
		TS_ASSERT_DELTA(msg.latitude(2), 61.5f, 1e-7);
		TS_ASSERT_DELTA(msg.longitude(2), 18.7f, 1e-7);
		TS_ASSERT_EQUALS(msg.COG(2), 80);
		TS_ASSERT_EQUALS(msg.SOG(2), 7);

		// std::cout << msg.vesselList().size() << '\n';
		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		AISDataMsg msgTwo(deserialiser);

		// std::cout << "Test: " << msgTwo.vesselList().size() << '\n';
		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.MMSI(0),1);
		TS_ASSERT_DELTA(msgTwo.latitude(0), 60.2f, 1e-7);
		TS_ASSERT_DELTA(msgTwo.longitude(0), 19.1f, 1e-7);
		TS_ASSERT_EQUALS(msgTwo.COG(0), 200);
		TS_ASSERT_EQUALS(msgTwo.SOG(0), 10);
		TS_ASSERT_EQUALS(msgTwo.MMSI(1), 2);
		TS_ASSERT_DELTA(msgTwo.latitude(1), 62.f, 1e-7);
		TS_ASSERT_DELTA(msgTwo.longitude(1), 18.1f, 1e-7);
		TS_ASSERT_EQUALS(msgTwo.COG(1), 100);
		TS_ASSERT_EQUALS(msgTwo.SOG(1), 5);
		TS_ASSERT_EQUALS(msgTwo.MMSI(2), 3);
		TS_ASSERT_DELTA(msgTwo.latitude(2), 61.5f, 1e-7);
		TS_ASSERT_DELTA(msgTwo.longitude(2), 18.7f, 1e-7);
		TS_ASSERT_EQUALS(msgTwo.COG(2), 80);
		TS_ASSERT_EQUALS(msgTwo.SOG(2), 7);
	}
};
