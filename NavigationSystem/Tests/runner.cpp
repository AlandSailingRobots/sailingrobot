/* Generated file, do not edit */

#ifndef CXXTEST_RUNNING
#define CXXTEST_RUNNING
#endif

#define _CXXTEST_HAVE_STD
#define _CXXTEST_HAVE_EH
#include <cxxtest/TestListener.h>
#include <cxxtest/TestTracker.h>
#include <cxxtest/TestRunner.h>
#include <cxxtest/RealDescriptions.h>
#include <cxxtest/TestMain.h>
#include <cxxtest/ErrorPrinter.h>

int main( int argc, char *argv[] ) {
 int status;
    CxxTest::ErrorPrinter tmp;
    CxxTest::RealWorldDescription::_worldName = "cxxtest";
    status = CxxTest::Main< CxxTest::ErrorPrinter >( tmp, argc, argv );
    return status;
}
bool suite_MessageCoreSuite_init = false;
#include "unit-tests/MessageCoreSuite.h"

static MessageCoreSuite suite_MessageCoreSuite;

static CxxTest::List Tests_MessageCoreSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_MessageCoreSuite( "unit-tests/MessageCoreSuite.h", 44, "MessageCoreSuite", suite_MessageCoreSuite, Tests_MessageCoreSuite );

static class TestDescription_suite_MessageCoreSuite_test_NodeRegisteredAtStart : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageCoreSuite_test_NodeRegisteredAtStart() : CxxTest::RealTestDescription( Tests_MessageCoreSuite, suiteDescription_MessageCoreSuite, 81, "test_NodeRegisteredAtStart" ) {}
 void runTest() { suite_MessageCoreSuite.test_NodeRegisteredAtStart(); }
} testDescription_suite_MessageCoreSuite_test_NodeRegisteredAtStart;

static class TestDescription_suite_MessageCoreSuite_test_FailedNodeRegisterAfterStart : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageCoreSuite_test_FailedNodeRegisterAfterStart() : CxxTest::RealTestDescription( Tests_MessageCoreSuite, suiteDescription_MessageCoreSuite, 87, "test_FailedNodeRegisterAfterStart" ) {}
 void runTest() { suite_MessageCoreSuite.test_FailedNodeRegisterAfterStart(); }
} testDescription_suite_MessageCoreSuite_test_FailedNodeRegisterAfterStart;

static class TestDescription_suite_MessageCoreSuite_test_MessageReceived : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageCoreSuite_test_MessageReceived() : CxxTest::RealTestDescription( Tests_MessageCoreSuite, suiteDescription_MessageCoreSuite, 95, "test_MessageReceived" ) {}
 void runTest() { suite_MessageCoreSuite.test_MessageReceived(); }
} testDescription_suite_MessageCoreSuite_test_MessageReceived;

static class TestDescription_suite_MessageCoreSuite_test_DirectMessage : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageCoreSuite_test_DirectMessage() : CxxTest::RealTestDescription( Tests_MessageCoreSuite, suiteDescription_MessageCoreSuite, 115, "test_DirectMessage" ) {}
 void runTest() { suite_MessageCoreSuite.test_DirectMessage(); }
} testDescription_suite_MessageCoreSuite_test_DirectMessage;

static class TestDescription_suite_MessageCoreSuite_test_DirectMessageToSomeoneElse : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageCoreSuite_test_DirectMessageToSomeoneElse() : CxxTest::RealTestDescription( Tests_MessageCoreSuite, suiteDescription_MessageCoreSuite, 131, "test_DirectMessageToSomeoneElse" ) {}
 void runTest() { suite_MessageCoreSuite.test_DirectMessageToSomeoneElse(); }
} testDescription_suite_MessageCoreSuite_test_DirectMessageToSomeoneElse;

static class TestDescription_suite_MessageCoreSuite_test_MessageWithDataReceived : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageCoreSuite_test_MessageWithDataReceived() : CxxTest::RealTestDescription( Tests_MessageCoreSuite, suiteDescription_MessageCoreSuite, 146, "test_MessageWithDataReceived" ) {}
 void runTest() { suite_MessageCoreSuite.test_MessageWithDataReceived(); }
} testDescription_suite_MessageCoreSuite_test_MessageWithDataReceived;

#include "unit-tests/MessageSuite.h"

static MessageSuite suite_MessageSuite;

static CxxTest::List Tests_MessageSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_MessageSuite( "unit-tests/MessageSuite.h", 39, "MessageSuite", suite_MessageSuite, Tests_MessageSuite );

static class TestDescription_suite_MessageSuite_test_CompassDataMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_CompassDataMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 45, "test_CompassDataMsg" ) {}
 void runTest() { suite_MessageSuite.test_CompassDataMsg(); }
} testDescription_suite_MessageSuite_test_CompassDataMsg;

static class TestDescription_suite_MessageSuite_test_GPSDataMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_GPSDataMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 67, "test_GPSDataMsg" ) {}
 void runTest() { suite_MessageSuite.test_GPSDataMsg(); }
} testDescription_suite_MessageSuite_test_GPSDataMsg;

static class TestDescription_suite_MessageSuite_test_WindDataMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_WindDataMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 101, "test_WindDataMsg" ) {}
 void runTest() { suite_MessageSuite.test_WindDataMsg(); }
} testDescription_suite_MessageSuite_test_WindDataMsg;

static class TestDescription_suite_MessageSuite_test_DataRequestMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_DataRequestMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 123, "test_DataRequestMsg" ) {}
 void runTest() { suite_MessageSuite.test_DataRequestMsg(); }
} testDescription_suite_MessageSuite_test_DataRequestMsg;

static class TestDescription_suite_MessageSuite_test_WaypointDataMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_WaypointDataMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 132, "test_WaypointDataMsg" ) {}
 void runTest() { suite_MessageSuite.test_WaypointDataMsg(); }
} testDescription_suite_MessageSuite_test_WaypointDataMsg;

static class TestDescription_suite_MessageSuite_test_ActuatorPositionMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_ActuatorPositionMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 170, "test_ActuatorPositionMsg" ) {}
 void runTest() { suite_MessageSuite.test_ActuatorPositionMsg(); }
} testDescription_suite_MessageSuite_test_ActuatorPositionMsg;

static class TestDescription_suite_MessageSuite_test_ArduinoDataMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_ArduinoDataMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 190, "test_ArduinoDataMsg" ) {}
 void runTest() { suite_MessageSuite.test_ArduinoDataMsg(); }
} testDescription_suite_MessageSuite_test_ArduinoDataMsg;

static class TestDescription_suite_MessageSuite_test_VesselStateMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_VesselStateMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 216, "test_VesselStateMsg" ) {}
 void runTest() { suite_MessageSuite.test_VesselStateMsg(); }
} testDescription_suite_MessageSuite_test_VesselStateMsg;

static class TestDescription_suite_MessageSuite_test_CourseDataMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_CourseDataMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 270, "test_CourseDataMsg" ) {}
 void runTest() { suite_MessageSuite.test_CourseDataMsg(); }
} testDescription_suite_MessageSuite_test_CourseDataMsg;

static class TestDescription_suite_MessageSuite_test_ServerConfigsReceivedMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_ServerConfigsReceivedMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 292, "test_ServerConfigsReceivedMsg" ) {}
 void runTest() { suite_MessageSuite.test_ServerConfigsReceivedMsg(); }
} testDescription_suite_MessageSuite_test_ServerConfigsReceivedMsg;

static class TestDescription_suite_MessageSuite_test_ServerWaypointsReceivedMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_ServerWaypointsReceivedMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 301, "test_ServerWaypointsReceivedMsg" ) {}
 void runTest() { suite_MessageSuite.test_ServerWaypointsReceivedMsg(); }
} testDescription_suite_MessageSuite_test_ServerWaypointsReceivedMsg;

static class TestDescription_suite_MessageSuite_test_LocalConfigChangeMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_LocalConfigChangeMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 310, "test_LocalConfigChangeMsg" ) {}
 void runTest() { suite_MessageSuite.test_LocalConfigChangeMsg(); }
} testDescription_suite_MessageSuite_test_LocalConfigChangeMsg;

static class TestDescription_suite_MessageSuite_test_LocalWaypointChangeMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_LocalWaypointChangeMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 319, "test_LocalWaypointChangeMsg" ) {}
 void runTest() { suite_MessageSuite.test_LocalWaypointChangeMsg(); }
} testDescription_suite_MessageSuite_test_LocalWaypointChangeMsg;

static class TestDescription_suite_MessageSuite_test_StateDataMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MessageSuite_test_StateDataMsg() : CxxTest::RealTestDescription( Tests_MessageSuite, suiteDescription_MessageSuite, 328, "test_StateDataMsg" ) {}
 void runTest() { suite_MessageSuite.test_StateDataMsg(); }
} testDescription_suite_MessageSuite_test_StateDataMsg;

#include "unit-tests/UtilitySuite.h"

static UtilitySuite suite_UtilitySuite;

static CxxTest::List Tests_UtilitySuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_UtilitySuite( "unit-tests/UtilitySuite.h", 42, "UtilitySuite", suite_UtilitySuite, Tests_UtilitySuite );

static class TestDescription_suite_UtilitySuite_test_CombineBytes : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_CombineBytes() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 48, "test_CombineBytes" ) {}
 void runTest() { suite_UtilitySuite.test_CombineBytes(); }
} testDescription_suite_UtilitySuite_test_CombineBytes;

static class TestDescription_suite_UtilitySuite_test_MedianEven : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_MedianEven() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 56, "test_MedianEven" ) {}
 void runTest() { suite_UtilitySuite.test_MedianEven(); }
} testDescription_suite_UtilitySuite_test_MedianEven;

static class TestDescription_suite_UtilitySuite_test_MedianOdd : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_MedianOdd() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 64, "test_MedianOdd" ) {}
 void runTest() { suite_UtilitySuite.test_MedianOdd(); }
} testDescription_suite_UtilitySuite_test_MedianOdd;

static class TestDescription_suite_UtilitySuite_test_MedianEmptyVector : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_MedianEmptyVector() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 72, "test_MedianEmptyVector" ) {}
 void runTest() { suite_UtilitySuite.test_MedianEmptyVector(); }
} testDescription_suite_UtilitySuite_test_MedianEmptyVector;

static class TestDescription_suite_UtilitySuite_test_Mean : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_Mean() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 79, "test_Mean" ) {}
 void runTest() { suite_UtilitySuite.test_Mean(); }
} testDescription_suite_UtilitySuite_test_Mean;

static class TestDescription_suite_UtilitySuite_test_MeanEmptyVector : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_MeanEmptyVector() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 90, "test_MeanEmptyVector" ) {}
 void runTest() { suite_UtilitySuite.test_MeanEmptyVector(); }
} testDescription_suite_UtilitySuite_test_MeanEmptyVector;

static class TestDescription_suite_UtilitySuite_test_MeanOfAngles : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_MeanOfAngles() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 96, "test_MeanOfAngles" ) {}
 void runTest() { suite_UtilitySuite.test_MeanOfAngles(); }
} testDescription_suite_UtilitySuite_test_MeanOfAngles;

static class TestDescription_suite_UtilitySuite_test_MeanOfAnglesAround0Degrees : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_MeanOfAnglesAround0Degrees() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 103, "test_MeanOfAnglesAround0Degrees" ) {}
 void runTest() { suite_UtilitySuite.test_MeanOfAnglesAround0Degrees(); }
} testDescription_suite_UtilitySuite_test_MeanOfAnglesAround0Degrees;

static class TestDescription_suite_UtilitySuite_test_MeanOfAnglesEmptyVector : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_MeanOfAnglesEmptyVector() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 110, "test_MeanOfAnglesEmptyVector" ) {}
 void runTest() { suite_UtilitySuite.test_MeanOfAnglesEmptyVector(); }
} testDescription_suite_UtilitySuite_test_MeanOfAnglesEmptyVector;

static class TestDescription_suite_UtilitySuite_test_MeanOfAnglesGreaterThan360 : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_MeanOfAnglesGreaterThan360() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 116, "test_MeanOfAnglesGreaterThan360" ) {}
 void runTest() { suite_UtilitySuite.test_MeanOfAnglesGreaterThan360(); }
} testDescription_suite_UtilitySuite_test_MeanOfAnglesGreaterThan360;

static class TestDescription_suite_UtilitySuite_test_MeanOfAnglesLessThan0 : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_MeanOfAnglesLessThan0() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 123, "test_MeanOfAnglesLessThan0" ) {}
 void runTest() { suite_UtilitySuite.test_MeanOfAnglesLessThan0(); }
} testDescription_suite_UtilitySuite_test_MeanOfAnglesLessThan0;

static class TestDescription_suite_UtilitySuite_test_Sgn : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_Sgn() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 130, "test_Sgn" ) {}
 void runTest() { suite_UtilitySuite.test_Sgn(); }
} testDescription_suite_UtilitySuite_test_Sgn;

static class TestDescription_suite_UtilitySuite_test_PolarToCartesianCoordinates : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_PolarToCartesianCoordinates() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 143, "test_PolarToCartesianCoordinates" ) {}
 void runTest() { suite_UtilitySuite.test_PolarToCartesianCoordinates(); }
} testDescription_suite_UtilitySuite_test_PolarToCartesianCoordinates;

static class TestDescription_suite_UtilitySuite_test_AngleInSector : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_AngleInSector() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 151, "test_AngleInSector" ) {}
 void runTest() { suite_UtilitySuite.test_AngleInSector(); }
} testDescription_suite_UtilitySuite_test_AngleInSector;

static class TestDescription_suite_UtilitySuite_test_DifferenceBetweenAngles : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_DifferenceBetweenAngles() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 164, "test_DifferenceBetweenAngles" ) {}
 void runTest() { suite_UtilitySuite.test_DifferenceBetweenAngles(); }
} testDescription_suite_UtilitySuite_test_DifferenceBetweenAngles;

static class TestDescription_suite_UtilitySuite_test_LimitAngleRange : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_LimitAngleRange() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 175, "test_LimitAngleRange" ) {}
 void runTest() { suite_UtilitySuite.test_LimitAngleRange(); }
} testDescription_suite_UtilitySuite_test_LimitAngleRange;

static class TestDescription_suite_UtilitySuite_test_LimitRadianAngleRange : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_LimitRadianAngleRange() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 187, "test_LimitRadianAngleRange" ) {}
 void runTest() { suite_UtilitySuite.test_LimitRadianAngleRange(); }
} testDescription_suite_UtilitySuite_test_LimitRadianAngleRange;

static class TestDescription_suite_UtilitySuite_test_DegreeRadianConversion : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_DegreeRadianConversion() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 201, "test_DegreeRadianConversion" ) {}
 void runTest() { suite_UtilitySuite.test_DegreeRadianConversion(); }
} testDescription_suite_UtilitySuite_test_DegreeRadianConversion;

static class TestDescription_suite_UtilitySuite_test_RadianToDegreeConversion : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_RadianToDegreeConversion() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 209, "test_RadianToDegreeConversion" ) {}
 void runTest() { suite_UtilitySuite.test_RadianToDegreeConversion(); }
} testDescription_suite_UtilitySuite_test_RadianToDegreeConversion;

static class TestDescription_suite_UtilitySuite_test_HeadingDifference : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_HeadingDifference() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 216, "test_HeadingDifference" ) {}
 void runTest() { suite_UtilitySuite.test_HeadingDifference(); }
} testDescription_suite_UtilitySuite_test_HeadingDifference;

static class TestDescription_suite_UtilitySuite_test_WrapAngle : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_WrapAngle() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 228, "test_WrapAngle" ) {}
 void runTest() { suite_UtilitySuite.test_WrapAngle(); }
} testDescription_suite_UtilitySuite_test_WrapAngle;

static class TestDescription_suite_UtilitySuite_test_AddingDeclinationToHeading : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_AddingDeclinationToHeading() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 235, "test_AddingDeclinationToHeading" ) {}
 void runTest() { suite_UtilitySuite.test_AddingDeclinationToHeading(); }
} testDescription_suite_UtilitySuite_test_AddingDeclinationToHeading;

static class TestDescription_suite_UtilitySuite_test_TrueWindDirection : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_UtilitySuite_test_TrueWindDirection() : CxxTest::RealTestDescription( Tests_UtilitySuite, suiteDescription_UtilitySuite, 242, "test_TrueWindDirection" ) {}
 void runTest() { suite_UtilitySuite.test_TrueWindDirection(); }
} testDescription_suite_UtilitySuite_test_TrueWindDirection;

#include "unit-tests/HTTPSyncSuite.h"

static HTTPSyncSuite suite_HTTPSyncSuite;

static CxxTest::List Tests_HTTPSyncSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_HTTPSyncSuite( "unit-tests/HTTPSyncSuite.h", 49, "HTTPSyncSuite", suite_HTTPSyncSuite, Tests_HTTPSyncSuite );

static class TestDescription_suite_HTTPSyncSuite_test_HTTPSyncInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HTTPSyncSuite_test_HTTPSyncInit() : CxxTest::RealTestDescription( Tests_HTTPSyncSuite, suiteDescription_HTTPSyncSuite, 98, "test_HTTPSyncInit" ) {}
 void runTest() { suite_HTTPSyncSuite.test_HTTPSyncInit(); }
} testDescription_suite_HTTPSyncSuite_test_HTTPSyncInit;

static class TestDescription_suite_HTTPSyncSuite_test_HTTPSyncValidURL : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HTTPSyncSuite_test_HTTPSyncValidURL() : CxxTest::RealTestDescription( Tests_HTTPSyncSuite, suiteDescription_HTTPSyncSuite, 104, "test_HTTPSyncValidURL" ) {}
 void runTest() { suite_HTTPSyncSuite.test_HTTPSyncValidURL(); }
} testDescription_suite_HTTPSyncSuite_test_HTTPSyncValidURL;

static class TestDescription_suite_HTTPSyncSuite_test_HTTPSyncWaypoints : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HTTPSyncSuite_test_HTTPSyncWaypoints() : CxxTest::RealTestDescription( Tests_HTTPSyncSuite, suiteDescription_HTTPSyncSuite, 116, "test_HTTPSyncWaypoints" ) {}
 void runTest() { suite_HTTPSyncSuite.test_HTTPSyncWaypoints(); }
} testDescription_suite_HTTPSyncSuite_test_HTTPSyncWaypoints;

static class TestDescription_suite_HTTPSyncSuite_test_HTTPSyncConfigs : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HTTPSyncSuite_test_HTTPSyncConfigs() : CxxTest::RealTestDescription( Tests_HTTPSyncSuite, suiteDescription_HTTPSyncSuite, 142, "test_HTTPSyncConfigs" ) {}
 void runTest() { suite_HTTPSyncSuite.test_HTTPSyncConfigs(); }
} testDescription_suite_HTTPSyncSuite_test_HTTPSyncConfigs;

static class TestDescription_suite_HTTPSyncSuite_test_HTTPSyncPushDataLogs : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HTTPSyncSuite_test_HTTPSyncPushDataLogs() : CxxTest::RealTestDescription( Tests_HTTPSyncSuite, suiteDescription_HTTPSyncSuite, 157, "test_HTTPSyncPushDataLogs" ) {}
 void runTest() { suite_HTTPSyncSuite.test_HTTPSyncPushDataLogs(); }
} testDescription_suite_HTTPSyncSuite_test_HTTPSyncPushDataLogs;

#include "unit-tests/WaypointNodeSuite.h"

static WaypointNodeSuite suite_WaypointNodeSuite;

static CxxTest::List Tests_WaypointNodeSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_WaypointNodeSuite( "unit-tests/WaypointNodeSuite.h", 38, "WaypointNodeSuite", suite_WaypointNodeSuite, Tests_WaypointNodeSuite );

static class TestDescription_suite_WaypointNodeSuite_test_WaypointNodeInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_WaypointNodeSuite_test_WaypointNodeInit() : CxxTest::RealTestDescription( Tests_WaypointNodeSuite, suiteDescription_WaypointNodeSuite, 84, "test_WaypointNodeInit" ) {}
 void runTest() { suite_WaypointNodeSuite.test_WaypointNodeInit(); }
} testDescription_suite_WaypointNodeSuite_test_WaypointNodeInit;

#include "unit-tests/LineFollowSuite.h"

static LineFollowSuite suite_LineFollowSuite;

static CxxTest::List Tests_LineFollowSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_LineFollowSuite( "unit-tests/LineFollowSuite.h", 47, "LineFollowSuite", suite_LineFollowSuite, Tests_LineFollowSuite );

static class TestDescription_suite_LineFollowSuite_test_LineFollowInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LineFollowSuite_test_LineFollowInit() : CxxTest::RealTestDescription( Tests_LineFollowSuite, suiteDescription_LineFollowSuite, 98, "test_LineFollowInit" ) {}
 void runTest() { suite_LineFollowSuite.test_LineFollowInit(); }
} testDescription_suite_LineFollowSuite_test_LineFollowInit;

static class TestDescription_suite_LineFollowSuite_test_LineFollowCalculateActuatorPosition : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LineFollowSuite_test_LineFollowCalculateActuatorPosition() : CxxTest::RealTestDescription( Tests_LineFollowSuite, suiteDescription_LineFollowSuite, 104, "test_LineFollowCalculateActuatorPosition" ) {}
 void runTest() { suite_LineFollowSuite.test_LineFollowCalculateActuatorPosition(); }
} testDescription_suite_LineFollowSuite_test_LineFollowCalculateActuatorPosition;

#include "unit-tests/WindStateNodeSuite.h"

static WindStateNodeSuite suite_WindStateNodeSuite;

static CxxTest::List Tests_WindStateNodeSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_WindStateNodeSuite( "unit-tests/WindStateNodeSuite.h", 18, "WindStateNodeSuite", suite_WindStateNodeSuite, Tests_WindStateNodeSuite );

static class TestDescription_suite_WindStateNodeSuite_test_NodeSendsMessage : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_WindStateNodeSuite_test_NodeSendsMessage() : CxxTest::RealTestDescription( Tests_WindStateNodeSuite, suiteDescription_WindStateNodeSuite, 66, "test_NodeSendsMessage" ) {}
 void runTest() { suite_WindStateNodeSuite.test_NodeSendsMessage(); }
} testDescription_suite_WindStateNodeSuite_test_NodeSendsMessage;

static class TestDescription_suite_WindStateNodeSuite_test_verifyCorrectMsgData : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_WindStateNodeSuite_test_verifyCorrectMsgData() : CxxTest::RealTestDescription( Tests_WindStateNodeSuite, suiteDescription_WindStateNodeSuite, 78, "test_verifyCorrectMsgData" ) {}
 void runTest() { suite_WindStateNodeSuite.test_verifyCorrectMsgData(); }
} testDescription_suite_WindStateNodeSuite_test_verifyCorrectMsgData;

#include "unit-tests/StateEstimationNodeSuite.h"

static StateEstimationNodeSuite suite_StateEstimationNodeSuite;

static CxxTest::List Tests_StateEstimationNodeSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_StateEstimationNodeSuite( "unit-tests/StateEstimationNodeSuite.h", 32, "StateEstimationNodeSuite", suite_StateEstimationNodeSuite, Tests_StateEstimationNodeSuite );

static class TestDescription_suite_StateEstimationNodeSuite_test_StateEstimationNodeInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_StateEstimationNodeSuite_test_StateEstimationNodeInit() : CxxTest::RealTestDescription( Tests_StateEstimationNodeSuite, suiteDescription_StateEstimationNodeSuite, 97, "test_StateEstimationNodeInit" ) {}
 void runTest() { suite_StateEstimationNodeSuite.test_StateEstimationNodeInit(); }
} testDescription_suite_StateEstimationNodeSuite_test_StateEstimationNodeInit;

static class TestDescription_suite_StateEstimationNodeSuite_test_StateEstimationNodeGPSNotOnline : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_StateEstimationNodeSuite_test_StateEstimationNodeGPSNotOnline() : CxxTest::RealTestDescription( Tests_StateEstimationNodeSuite, suiteDescription_StateEstimationNodeSuite, 104, "test_StateEstimationNodeGPSNotOnline" ) {}
 void runTest() { suite_StateEstimationNodeSuite.test_StateEstimationNodeGPSNotOnline(); }
} testDescription_suite_StateEstimationNodeSuite_test_StateEstimationNodeGPSNotOnline;

static class TestDescription_suite_StateEstimationNodeSuite_test_StateMessageListener : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_StateEstimationNodeSuite_test_StateMessageListener() : CxxTest::RealTestDescription( Tests_StateEstimationNodeSuite, suiteDescription_StateEstimationNodeSuite, 113, "test_StateMessageListener" ) {}
 void runTest() { suite_StateEstimationNodeSuite.test_StateMessageListener(); }
} testDescription_suite_StateEstimationNodeSuite_test_StateMessageListener;

static class TestDescription_suite_StateEstimationNodeSuite_test_StateEstimationStateMsgHeading : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_StateEstimationNodeSuite_test_StateEstimationStateMsgHeading() : CxxTest::RealTestDescription( Tests_StateEstimationNodeSuite, suiteDescription_StateEstimationNodeSuite, 133, "test_StateEstimationStateMsgHeading" ) {}
 void runTest() { suite_StateEstimationNodeSuite.test_StateEstimationStateMsgHeading(); }
} testDescription_suite_StateEstimationNodeSuite_test_StateEstimationStateMsgHeading;

static class TestDescription_suite_StateEstimationNodeSuite_test_StateEstimationStateMessageGPSData : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_StateEstimationNodeSuite_test_StateEstimationStateMessageGPSData() : CxxTest::RealTestDescription( Tests_StateEstimationNodeSuite, suiteDescription_StateEstimationNodeSuite, 168, "test_StateEstimationStateMessageGPSData" ) {}
 void runTest() { suite_StateEstimationNodeSuite.test_StateEstimationStateMessageGPSData(); }
} testDescription_suite_StateEstimationNodeSuite_test_StateEstimationStateMessageGPSData;

static class TestDescription_suite_StateEstimationNodeSuite_test_StateEstStateMsgSpeedAndDeclZero : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_StateEstimationNodeSuite_test_StateEstStateMsgSpeedAndDeclZero() : CxxTest::RealTestDescription( Tests_StateEstimationNodeSuite, suiteDescription_StateEstimationNodeSuite, 191, "test_StateEstStateMsgSpeedAndDeclZero" ) {}
 void runTest() { suite_StateEstimationNodeSuite.test_StateEstStateMsgSpeedAndDeclZero(); }
} testDescription_suite_StateEstimationNodeSuite_test_StateEstStateMsgSpeedAndDeclZero;

static class TestDescription_suite_StateEstimationNodeSuite_test_StateEstStateMsgSpeedAndDeclOverZero : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_StateEstimationNodeSuite_test_StateEstStateMsgSpeedAndDeclOverZero() : CxxTest::RealTestDescription( Tests_StateEstimationNodeSuite, suiteDescription_StateEstimationNodeSuite, 217, "test_StateEstStateMsgSpeedAndDeclOverZero" ) {}
 void runTest() { suite_StateEstimationNodeSuite.test_StateEstStateMsgSpeedAndDeclOverZero(); }
} testDescription_suite_StateEstimationNodeSuite_test_StateEstStateMsgSpeedAndDeclOverZero;

static class TestDescription_suite_StateEstimationNodeSuite_test_StateEstStateMsgSpeedLessThanZero : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_StateEstimationNodeSuite_test_StateEstStateMsgSpeedLessThanZero() : CxxTest::RealTestDescription( Tests_StateEstimationNodeSuite, suiteDescription_StateEstimationNodeSuite, 247, "test_StateEstStateMsgSpeedLessThanZero" ) {}
 void runTest() { suite_StateEstimationNodeSuite.test_StateEstStateMsgSpeedLessThanZero(); }
} testDescription_suite_StateEstimationNodeSuite_test_StateEstStateMsgSpeedLessThanZero;

#include "unit-tests/CourseRegulatorNodeSuite.h"

static CourseRegulatorNodeSuite suite_CourseRegulatorNodeSuite;

static CxxTest::List Tests_CourseRegulatorNodeSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_CourseRegulatorNodeSuite( "unit-tests/CourseRegulatorNodeSuite.h", 31, "CourseRegulatorNodeSuite", suite_CourseRegulatorNodeSuite, Tests_CourseRegulatorNodeSuite );

static class TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeInit() : CxxTest::RealTestDescription( Tests_CourseRegulatorNodeSuite, suiteDescription_CourseRegulatorNodeSuite, 97, "test_CourseRegulatorNodeInit" ) {}
 void runTest() { suite_CourseRegulatorNodeSuite.test_CourseRegulatorNodeInit(); }
} testDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeInit;

static class TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorMsgListener : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorMsgListener() : CxxTest::RealTestDescription( Tests_CourseRegulatorNodeSuite, suiteDescription_CourseRegulatorNodeSuite, 104, "test_CourseRegulatorMsgListener" ) {}
 void runTest() { suite_CourseRegulatorNodeSuite.test_CourseRegulatorMsgListener(); }
} testDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorMsgListener;

static class TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorCourseMsgRudder : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorCourseMsgRudder() : CxxTest::RealTestDescription( Tests_CourseRegulatorNodeSuite, suiteDescription_CourseRegulatorNodeSuite, 120, "test_CourseRegulatorCourseMsgRudder" ) {}
 void runTest() { suite_CourseRegulatorNodeSuite.test_CourseRegulatorCourseMsgRudder(); }
} testDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorCourseMsgRudder;

static class TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorCourseMsgActuatorData : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorCourseMsgActuatorData() : CxxTest::RealTestDescription( Tests_CourseRegulatorNodeSuite, suiteDescription_CourseRegulatorNodeSuite, 149, "test_CourseRegulatorCourseMsgActuatorData" ) {}
 void runTest() { suite_CourseRegulatorNodeSuite.test_CourseRegulatorCourseMsgActuatorData(); }
} testDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorCourseMsgActuatorData;

static class TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeOppositeDesireHeading : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeOppositeDesireHeading() : CxxTest::RealTestDescription( Tests_CourseRegulatorNodeSuite, suiteDescription_CourseRegulatorNodeSuite, 166, "test_CourseRegulatorNodeOppositeDesireHeading" ) {}
 void runTest() { suite_CourseRegulatorNodeSuite.test_CourseRegulatorNodeOppositeDesireHeading(); }
} testDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeOppositeDesireHeading;

static class TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeNegSpeed : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeNegSpeed() : CxxTest::RealTestDescription( Tests_CourseRegulatorNodeSuite, suiteDescription_CourseRegulatorNodeSuite, 189, "test_CourseRegulatorNodeNegSpeed" ) {}
 void runTest() { suite_CourseRegulatorNodeSuite.test_CourseRegulatorNodeNegSpeed(); }
} testDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeNegSpeed;

static class TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeOppositeAndNegSpeed : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeOppositeAndNegSpeed() : CxxTest::RealTestDescription( Tests_CourseRegulatorNodeSuite, suiteDescription_CourseRegulatorNodeSuite, 213, "test_CourseRegulatorNodeOppositeAndNegSpeed" ) {}
 void runTest() { suite_CourseRegulatorNodeSuite.test_CourseRegulatorNodeOppositeAndNegSpeed(); }
} testDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorNodeOppositeAndNegSpeed;

static class TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorUpdateFrequency : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorUpdateFrequency() : CxxTest::RealTestDescription( Tests_CourseRegulatorNodeSuite, suiteDescription_CourseRegulatorNodeSuite, 236, "test_CourseRegulatorUpdateFrequency" ) {}
 void runTest() { suite_CourseRegulatorNodeSuite.test_CourseRegulatorUpdateFrequency(); }
} testDescription_suite_CourseRegulatorNodeSuite_test_CourseRegulatorUpdateFrequency;

#include "unit-tests/CANMessageSuite.h"

static CANMessageSuite suite_CANMessageSuite;

static CxxTest::List Tests_CANMessageSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_CANMessageSuite( "unit-tests/CANMessageSuite.h", 7, "CANMessageSuite", suite_CANMessageSuite, Tests_CANMessageSuite );

static class TestDescription_suite_CANMessageSuite_test_CANMessageConversion : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CANMessageSuite_test_CANMessageConversion() : CxxTest::RealTestDescription( Tests_CANMessageSuite, suiteDescription_CANMessageSuite, 13, "test_CANMessageConversion" ) {}
 void runTest() { suite_CANMessageSuite.test_CANMessageConversion(); }
} testDescription_suite_CANMessageSuite_test_CANMessageConversion;

#include "unit-tests/DBLoggerNodeSuite.h"

static DBLoggerNodeSuite suite_DBLoggerNodeSuite;

static CxxTest::List Tests_DBLoggerNodeSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_DBLoggerNodeSuite( "unit-tests/DBLoggerNodeSuite.h", 21, "DBLoggerNodeSuite", suite_DBLoggerNodeSuite, Tests_DBLoggerNodeSuite );

static class TestDescription_suite_DBLoggerNodeSuite_test_LoggingToDB : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_DBLoggerNodeSuite_test_LoggingToDB() : CxxTest::RealTestDescription( Tests_DBLoggerNodeSuite, suiteDescription_DBLoggerNodeSuite, 52, "test_LoggingToDB" ) {}
 void runTest() { suite_DBLoggerNodeSuite.test_LoggingToDB(); }
} testDescription_suite_DBLoggerNodeSuite_test_LoggingToDB;

#include "unit-tests/LowLevelControllerNodeJanetSuite.h"

static LowLevelControllerNodeJanetSuite suite_LowLevelControllerNodeJanetSuite;

static CxxTest::List Tests_LowLevelControllerNodeJanetSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_LowLevelControllerNodeJanetSuite( "unit-tests/LowLevelControllerNodeJanetSuite.h", 30, "LowLevelControllerNodeJanetSuite", suite_LowLevelControllerNodeJanetSuite, Tests_LowLevelControllerNodeJanetSuite );

static class TestDescription_suite_LowLevelControllerNodeJanetSuite_test_LowLevelControllerNodeJanetRegistered : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LowLevelControllerNodeJanetSuite_test_LowLevelControllerNodeJanetRegistered() : CxxTest::RealTestDescription( Tests_LowLevelControllerNodeJanetSuite, suiteDescription_LowLevelControllerNodeJanetSuite, 78, "test_LowLevelControllerNodeJanetRegistered" ) {}
 void runTest() { suite_LowLevelControllerNodeJanetSuite.test_LowLevelControllerNodeJanetRegistered(); }
} testDescription_suite_LowLevelControllerNodeJanetSuite_test_LowLevelControllerNodeJanetRegistered;

static class TestDescription_suite_LowLevelControllerNodeJanetSuite_test_LowLevelControllerNodeJanetInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LowLevelControllerNodeJanetSuite_test_LowLevelControllerNodeJanetInit() : CxxTest::RealTestDescription( Tests_LowLevelControllerNodeJanetSuite, suiteDescription_LowLevelControllerNodeJanetSuite, 82, "test_LowLevelControllerNodeJanetInit" ) {}
 void runTest() { suite_LowLevelControllerNodeJanetSuite.test_LowLevelControllerNodeJanetInit(); }
} testDescription_suite_LowLevelControllerNodeJanetSuite_test_LowLevelControllerNodeJanetInit;

static class TestDescription_suite_LowLevelControllerNodeJanetSuite_test_LowLeveControllerNodeProcessMsg : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LowLevelControllerNodeJanetSuite_test_LowLeveControllerNodeProcessMsg() : CxxTest::RealTestDescription( Tests_LowLevelControllerNodeJanetSuite, suiteDescription_LowLevelControllerNodeJanetSuite, 86, "test_LowLeveControllerNodeProcessMsg" ) {}
 void runTest() { suite_LowLevelControllerNodeJanetSuite.test_LowLeveControllerNodeProcessMsg(); }
} testDescription_suite_LowLevelControllerNodeJanetSuite_test_LowLeveControllerNodeProcessMsg;

#include "unit-tests/LowLevelControllersFunctionsTestSuite.h"

static LowLevelControllersFunctionsTestSuite suite_LowLevelControllersFunctionsTestSuite;

static CxxTest::List Tests_LowLevelControllersFunctionsTestSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_LowLevelControllersFunctionsTestSuite( "unit-tests/LowLevelControllersFunctionsTestSuite.h", 30, "LowLevelControllersFunctionsTestSuite", suite_LowLevelControllersFunctionsTestSuite, Tests_LowLevelControllersFunctionsTestSuite );

static class TestDescription_suite_LowLevelControllersFunctionsTestSuite_test_CourseRegulatorRudderCalculation : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LowLevelControllersFunctionsTestSuite_test_CourseRegulatorRudderCalculation() : CxxTest::RealTestDescription( Tests_LowLevelControllersFunctionsTestSuite, suiteDescription_LowLevelControllersFunctionsTestSuite, 44, "test_CourseRegulatorRudderCalculation" ) {}
 void runTest() { suite_LowLevelControllersFunctionsTestSuite.test_CourseRegulatorRudderCalculation(); }
} testDescription_suite_LowLevelControllersFunctionsTestSuite_test_CourseRegulatorRudderCalculation;

static class TestDescription_suite_LowLevelControllersFunctionsTestSuite_test_WingsailControllerServoAngleCalculation : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_LowLevelControllersFunctionsTestSuite_test_WingsailControllerServoAngleCalculation() : CxxTest::RealTestDescription( Tests_LowLevelControllersFunctionsTestSuite, suiteDescription_LowLevelControllersFunctionsTestSuite, 62, "test_WingsailControllerServoAngleCalculation" ) {}
 void runTest() { suite_LowLevelControllersFunctionsTestSuite.test_WingsailControllerServoAngleCalculation(); }
} testDescription_suite_LowLevelControllersFunctionsTestSuite_test_WingsailControllerServoAngleCalculation;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
