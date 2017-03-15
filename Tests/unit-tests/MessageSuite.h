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
		GPSDataMsg msg(true, false, 52.0, 48.2, 1993, 5, 100, 11, GPSMode::NoFix);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::GPSData);
		TS_ASSERT_EQUALS(msg.hasFix(), true);
		TS_ASSERT_EQUALS(msg.gpsOnline(), false);
		TS_ASSERT_EQUALS(msg.latitude(), 52.0);
		TS_ASSERT_EQUALS(msg.longitude(), 48.2);
		TS_ASSERT_EQUALS(msg.unixTime(), 1993);
		TS_ASSERT_EQUALS(msg.speed(), 5);
		TS_ASSERT_EQUALS(msg.heading(), 100);
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
		TS_ASSERT_EQUALS(msgTwo.latitude(), 52.0);
		TS_ASSERT_EQUALS(msgTwo.longitude(), 48.2);
		TS_ASSERT_EQUALS(msgTwo.unixTime(), 1993);
		TS_ASSERT_EQUALS(msgTwo.speed(), 5);
		TS_ASSERT_EQUALS(msgTwo.heading(), 100);
		TS_ASSERT_EQUALS(msgTwo.satelliteCount(), 11);
		TS_ASSERT_EQUALS(msgTwo.gpsMode(), GPSMode::NoFix);
	}

	void test_WindDataMsg()
	{
		WindDataMsg msg(100, 80, 60);

		TS_ASSERT_EQUALS(msg.messageType(), MessageType::WindData);
		TS_ASSERT_EQUALS(msg.windDirection(), 100);
		TS_ASSERT_EQUALS(msg.windSpeed(), 80);
		TS_ASSERT_EQUALS(msg.windTemp(), 60);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		WindDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::WindData);
		TS_ASSERT_EQUALS(msgTwo.windDirection(), 100);
		TS_ASSERT_EQUALS(msgTwo.windSpeed(), 80);
		TS_ASSERT_EQUALS(msgTwo.windTemp(), 60);
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
		TS_ASSERT_EQUALS(msg.nextLatitude(), 60.2f);
		TS_ASSERT_EQUALS(msg.prevId(), 1);
		TS_ASSERT_EQUALS(msg.nextId(), 2);
		TS_ASSERT_EQUALS(msg.nextDeclination(), 6);
		TS_ASSERT_EQUALS(msg.prevDeclination(), 6);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		WaypointDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.messageType(), MessageType::WaypointData);
		TS_ASSERT_EQUALS(msgTwo.nextLatitude(), 60.2f);
		TS_ASSERT_EQUALS(msgTwo.prevId(), 1);
		TS_ASSERT_EQUALS(msgTwo.nextId(), 2);
		TS_ASSERT_EQUALS(msgTwo.nextDeclination(), 6);
		TS_ASSERT_EQUALS(msgTwo.prevDeclination(), 6);
	}

	void test_ActuatorPositionMsg()
	{
		ActuatorPositionMsg msg(5507, 4765);

		TS_ASSERT_EQUALS(msg.rudderPosition(), 5507);
		TS_ASSERT_EQUALS(msg.sailPosition(), 4765);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		ActuatorPositionMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.rudderPosition(), 5507);
		TS_ASSERT_EQUALS(msgTwo.sailPosition(), 4765);
	}

	void test_ArduinoDataMsg()
	{
		TS_FAIL("Test needs fixing!");
		/*ArduinoDataMsg msg(10, 5500, 4700, 2);

		TS_ASSERT_EQUALS(msg.pressure(), 10);
		TS_ASSERT_EQUALS(msg.rudder(), 5500);
		TS_ASSERT_EQUALS(msg.sheet(), 4700);
		TS_ASSERT_EQUALS(msg.battery(), 2);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		ArduinoDataMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.pressure(), 10);
		TS_ASSERT_EQUALS(msgTwo.rudder(), 5500);
		TS_ASSERT_EQUALS(msgTwo.sheet(), 4700);
		TS_ASSERT_EQUALS(msgTwo.battery(), 2);*/
	}
	void test_VesselStateMsg()
	{
		TS_FAIL("Test needs fixing!");

		/*VesselStateMsg msg(170, 30, 0, true, true, 19.2, 60.02, 120.04, 2.1, 11, 170, 23.5f, 5.4f, 24.5f, 10, 5500, 4700, 2);

		TS_ASSERT_EQUALS(msg.speed(), 2.1);
		TS_ASSERT_EQUALS(msg.compassHeading(), 170);
		TS_ASSERT_EQUALS(msg.windDir(), 23.5f);
		TS_ASSERT_EQUALS(msg.arduinoPressure(), 10);
		TS_ASSERT_EQUALS(msg.gpsOnline(), true);

		MessageSerialiser serialiser;
		msg.Serialise(serialiser);

		MessageDeserialiser deserialiser(serialiser.data(), serialiser.size());
		VesselStateMsg msgTwo(deserialiser);

		TS_ASSERT(msgTwo.isValid());
		TS_ASSERT_EQUALS(msgTwo.speed(), 2.1);
		TS_ASSERT_EQUALS(msgTwo.compassHeading(), 170);
		TS_ASSERT_EQUALS(msgTwo.windDir(), 23.5f);
		TS_ASSERT_EQUALS(msgTwo.arduinoPressure(), 10);
		TS_ASSERT_EQUALS(msgTwo.gpsOnline(), true);*/
	}
};
