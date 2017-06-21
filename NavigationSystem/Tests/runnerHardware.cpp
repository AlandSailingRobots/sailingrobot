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
bool suite_HardwareXbeeSuite_init = false;
#include "unit-tests/HardwareXbeeSuite.h"

static HardwareXbeeSuite suite_HardwareXbeeSuite;

static CxxTest::List Tests_HardwareXbeeSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_HardwareXbeeSuite( "unit-tests/HardwareXbeeSuite.h", 83, "HardwareXbeeSuite", suite_HardwareXbeeSuite, Tests_HardwareXbeeSuite );

static class TestDescription_suite_HardwareXbeeSuite_test_BrexitVotePassed : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_BrexitVotePassed() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 132, "test_BrexitVotePassed" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_BrexitVotePassed(); }
} testDescription_suite_HardwareXbeeSuite_test_BrexitVotePassed;

static class TestDescription_suite_HardwareXbeeSuite_test_EndOfUK : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_EndOfUK() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 137, "test_EndOfUK" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_EndOfUK(); }
} testDescription_suite_HardwareXbeeSuite_test_EndOfUK;

static class TestDescription_suite_HardwareXbeeSuite_test_Init : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_Init() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 142, "test_Init" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_Init(); }
} testDescription_suite_HardwareXbeeSuite_test_Init;

static class TestDescription_suite_HardwareXbeeSuite_test_SinglePacketData : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_SinglePacketData() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 147, "test_SinglePacketData" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_SinglePacketData(); }
} testDescription_suite_HardwareXbeeSuite_test_SinglePacketData;

static class TestDescription_suite_HardwareXbeeSuite_test_MultiPacketData : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_MultiPacketData() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 157, "test_MultiPacketData" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_MultiPacketData(); }
} testDescription_suite_HardwareXbeeSuite_test_MultiPacketData;

static class TestDescription_suite_HardwareXbeeSuite_test_PacketCountIncrement : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_PacketCountIncrement() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 170, "test_PacketCountIncrement" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_PacketCountIncrement(); }
} testDescription_suite_HardwareXbeeSuite_test_PacketCountIncrement;

static class TestDescription_suite_HardwareXbeeSuite_test_PacketReceive : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_PacketReceive() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 190, "test_PacketReceive" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_PacketReceive(); }
} testDescription_suite_HardwareXbeeSuite_test_PacketReceive;

static class TestDescription_suite_HardwareXbeeSuite_test_MultiPacketReceive : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_MultiPacketReceive() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 199, "test_MultiPacketReceive" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_MultiPacketReceive(); }
} testDescription_suite_HardwareXbeeSuite_test_MultiPacketReceive;

static class TestDescription_suite_HardwareXbeeSuite_test_CallbackSinglePacket : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_CallbackSinglePacket() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 209, "test_CallbackSinglePacket" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_CallbackSinglePacket(); }
} testDescription_suite_HardwareXbeeSuite_test_CallbackSinglePacket;

static class TestDescription_suite_HardwareXbeeSuite_test_CallbackMultiPacket : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_CallbackMultiPacket() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 223, "test_CallbackMultiPacket" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_CallbackMultiPacket(); }
} testDescription_suite_HardwareXbeeSuite_test_CallbackMultiPacket;

static class TestDescription_suite_HardwareXbeeSuite_test_FletcherChecksum : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareXbeeSuite_test_FletcherChecksum() : CxxTest::RealTestDescription( Tests_HardwareXbeeSuite, suiteDescription_HardwareXbeeSuite, 239, "test_FletcherChecksum" ) {}
 void runTest() { suite_HardwareXbeeSuite.test_FletcherChecksum(); }
} testDescription_suite_HardwareXbeeSuite_test_FletcherChecksum;

#include "unit-tests/HardwareCompassSuite.h"

static HardwareCompassSuite suite_HardwareCompassSuite;

static CxxTest::List Tests_HardwareCompassSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_HardwareCompassSuite( "unit-tests/HardwareCompassSuite.h", 41, "HardwareCompassSuite", suite_HardwareCompassSuite, Tests_HardwareCompassSuite );

static class TestDescription_suite_HardwareCompassSuite_test_CompassInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareCompassSuite_test_CompassInit() : CxxTest::RealTestDescription( Tests_HardwareCompassSuite, suiteDescription_HardwareCompassSuite, 84, "test_CompassInit" ) {}
 void runTest() { suite_HardwareCompassSuite.test_CompassInit(); }
} testDescription_suite_HardwareCompassSuite_test_CompassInit;

static class TestDescription_suite_HardwareCompassSuite_test_CompassRead : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareCompassSuite_test_CompassRead() : CxxTest::RealTestDescription( Tests_HardwareCompassSuite, suiteDescription_HardwareCompassSuite, 89, "test_CompassRead" ) {}
 void runTest() { suite_HardwareCompassSuite.test_CompassRead(); }
} testDescription_suite_HardwareCompassSuite_test_CompassRead;

static class TestDescription_suite_HardwareCompassSuite_test_CompassThread : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareCompassSuite_test_CompassThread() : CxxTest::RealTestDescription( Tests_HardwareCompassSuite, suiteDescription_HardwareCompassSuite, 106, "test_CompassThread" ) {}
 void runTest() { suite_HardwareCompassSuite.test_CompassThread(); }
} testDescription_suite_HardwareCompassSuite_test_CompassThread;

#include "unit-tests/HardwareCV7Suite.h"

static HardwareCV7Suite suite_HardwareCV7Suite;

static CxxTest::List Tests_HardwareCV7Suite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_HardwareCV7Suite( "unit-tests/HardwareCV7Suite.h", 42, "HardwareCV7Suite", suite_HardwareCV7Suite, Tests_HardwareCV7Suite );

static class TestDescription_suite_HardwareCV7Suite_test_CV7Init : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareCV7Suite_test_CV7Init() : CxxTest::RealTestDescription( Tests_HardwareCV7Suite, suiteDescription_HardwareCV7Suite, 87, "test_CV7Init" ) {}
 void runTest() { suite_HardwareCV7Suite.test_CV7Init(); }
} testDescription_suite_HardwareCV7Suite_test_CV7Init;

static class TestDescription_suite_HardwareCV7Suite_test_CV7ParseSuccesfully : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareCV7Suite_test_CV7ParseSuccesfully() : CxxTest::RealTestDescription( Tests_HardwareCV7Suite, suiteDescription_HardwareCV7Suite, 92, "test_CV7ParseSuccesfully" ) {}
 void runTest() { suite_HardwareCV7Suite.test_CV7ParseSuccesfully(); }
} testDescription_suite_HardwareCV7Suite_test_CV7ParseSuccesfully;

static class TestDescription_suite_HardwareCV7Suite_test_CV7ParseFail : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareCV7Suite_test_CV7ParseFail() : CxxTest::RealTestDescription( Tests_HardwareCV7Suite, suiteDescription_HardwareCV7Suite, 108, "test_CV7ParseFail" ) {}
 void runTest() { suite_HardwareCV7Suite.test_CV7ParseFail(); }
} testDescription_suite_HardwareCV7Suite_test_CV7ParseFail;

static class TestDescription_suite_HardwareCV7Suite_test_CV7Thread : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareCV7Suite_test_CV7Thread() : CxxTest::RealTestDescription( Tests_HardwareCV7Suite, suiteDescription_HardwareCV7Suite, 118, "test_CV7Thread" ) {}
 void runTest() { suite_HardwareCV7Suite.test_CV7Thread(); }
} testDescription_suite_HardwareCV7Suite_test_CV7Thread;

#include "unit-tests/HardwareGPSDSuite.h"

static HardwareGPSDSuite suite_HardwareGPSDSuite;

static CxxTest::List Tests_HardwareGPSDSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_HardwareGPSDSuite( "unit-tests/HardwareGPSDSuite.h", 40, "HardwareGPSDSuite", suite_HardwareGPSDSuite, Tests_HardwareGPSDSuite );

static class TestDescription_suite_HardwareGPSDSuite_test_GPSInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareGPSDSuite_test_GPSInit() : CxxTest::RealTestDescription( Tests_HardwareGPSDSuite, suiteDescription_HardwareGPSDSuite, 83, "test_GPSInit" ) {}
 void runTest() { suite_HardwareGPSDSuite.test_GPSInit(); }
} testDescription_suite_HardwareGPSDSuite_test_GPSInit;

static class TestDescription_suite_HardwareGPSDSuite_test_GPSThread : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_HardwareGPSDSuite_test_GPSThread() : CxxTest::RealTestDescription( Tests_HardwareGPSDSuite, suiteDescription_HardwareGPSDSuite, 88, "test_GPSThread" ) {}
 void runTest() { suite_HardwareGPSDSuite.test_GPSThread(); }
} testDescription_suite_HardwareGPSDSuite_test_GPSThread;

#include "unit-tests/MaestroControllerSuite.h"

static MaestroControllerSuite suite_MaestroControllerSuite;

static CxxTest::List Tests_MaestroControllerSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_MaestroControllerSuite( "unit-tests/MaestroControllerSuite.h", 34, "MaestroControllerSuite", suite_MaestroControllerSuite, Tests_MaestroControllerSuite );

static class TestDescription_suite_MaestroControllerSuite_test_MaestroControllerInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MaestroControllerSuite_test_MaestroControllerInit() : CxxTest::RealTestDescription( Tests_MaestroControllerSuite, suiteDescription_MaestroControllerSuite, 72, "test_MaestroControllerInit" ) {}
 void runTest() { suite_MaestroControllerSuite.test_MaestroControllerInit(); }
} testDescription_suite_MaestroControllerSuite_test_MaestroControllerInit;

static class TestDescription_suite_MaestroControllerSuite_test_MaestroControllerWriteCommand : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MaestroControllerSuite_test_MaestroControllerWriteCommand() : CxxTest::RealTestDescription( Tests_MaestroControllerSuite, suiteDescription_MaestroControllerSuite, 77, "test_MaestroControllerWriteCommand" ) {}
 void runTest() { suite_MaestroControllerSuite.test_MaestroControllerWriteCommand(); }
} testDescription_suite_MaestroControllerSuite_test_MaestroControllerWriteCommand;

static class TestDescription_suite_MaestroControllerSuite_test_MaestroControllerGetError : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_MaestroControllerSuite_test_MaestroControllerGetError() : CxxTest::RealTestDescription( Tests_MaestroControllerSuite, suiteDescription_MaestroControllerSuite, 82, "test_MaestroControllerGetError" ) {}
 void runTest() { suite_MaestroControllerSuite.test_MaestroControllerGetError(); }
} testDescription_suite_MaestroControllerSuite_test_MaestroControllerGetError;

#include "unit-tests/ActuatorSailNodeSuite.h"

static ActuatorSailNodeSuite suite_ActuatorSailNodeSuite;

static CxxTest::List Tests_ActuatorSailNodeSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_ActuatorSailNodeSuite( "unit-tests/ActuatorSailNodeSuite.h", 35, "ActuatorSailNodeSuite", suite_ActuatorSailNodeSuite, Tests_ActuatorSailNodeSuite );

static class TestDescription_suite_ActuatorSailNodeSuite_test_ActuatorSailNodeInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_ActuatorSailNodeSuite_test_ActuatorSailNodeInit() : CxxTest::RealTestDescription( Tests_ActuatorSailNodeSuite, suiteDescription_ActuatorSailNodeSuite, 79, "test_ActuatorSailNodeInit" ) {}
 void runTest() { suite_ActuatorSailNodeSuite.test_ActuatorSailNodeInit(); }
} testDescription_suite_ActuatorSailNodeSuite_test_ActuatorSailNodeInit;

#include "unit-tests/ActuatorRudderNodeSuite.h"

static ActuatorRudderNodeSuite suite_ActuatorRudderNodeSuite;

static CxxTest::List Tests_ActuatorRudderNodeSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_ActuatorRudderNodeSuite( "unit-tests/ActuatorRudderNodeSuite.h", 34, "ActuatorRudderNodeSuite", suite_ActuatorRudderNodeSuite, Tests_ActuatorRudderNodeSuite );

static class TestDescription_suite_ActuatorRudderNodeSuite_test_ActuatorRudderNodeInit : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_ActuatorRudderNodeSuite_test_ActuatorRudderNodeInit() : CxxTest::RealTestDescription( Tests_ActuatorRudderNodeSuite, suiteDescription_ActuatorRudderNodeSuite, 78, "test_ActuatorRudderNodeInit" ) {}
 void runTest() { suite_ActuatorRudderNodeSuite.test_ActuatorRudderNodeInit(); }
} testDescription_suite_ActuatorRudderNodeSuite_test_ActuatorRudderNodeInit;

#include "unit-tests/CANServiceSuite.h"

static CANServiceSuite suite_CANServiceSuite;

static CxxTest::List Tests_CANServiceSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_CANServiceSuite( "unit-tests/CANServiceSuite.h", 21, "CANServiceSuite", suite_CANServiceSuite, Tests_CANServiceSuite );

static class TestDescription_suite_CANServiceSuite_test_SendAndMissedMessages : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CANServiceSuite_test_SendAndMissedMessages() : CxxTest::RealTestDescription( Tests_CANServiceSuite, suiteDescription_CANServiceSuite, 35, "test_SendAndMissedMessages" ) {}
 void runTest() { suite_CANServiceSuite.test_SendAndMissedMessages(); }
} testDescription_suite_CANServiceSuite_test_SendAndMissedMessages;

#include "unit-tests/ActuatorNodeASPireSuite.h"

static ActuatorNodeASPireSuite suite_ActuatorNodeASPireSuite;

static CxxTest::List Tests_ActuatorNodeASPireSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_ActuatorNodeASPireSuite( "unit-tests/ActuatorNodeASPireSuite.h", 17, "ActuatorNodeASPireSuite", suite_ActuatorNodeASPireSuite, Tests_ActuatorNodeASPireSuite );

static class TestDescription_suite_ActuatorNodeASPireSuite_test_NodeSendsFrame : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_ActuatorNodeASPireSuite_test_NodeSendsFrame() : CxxTest::RealTestDescription( Tests_ActuatorNodeASPireSuite, suiteDescription_ActuatorNodeASPireSuite, 34, "test_NodeSendsFrame" ) {}
 void runTest() { suite_ActuatorNodeASPireSuite.test_NodeSendsFrame(); }
} testDescription_suite_ActuatorNodeASPireSuite_test_NodeSendsFrame;

#include "unit-tests/CANFeedbackReceiverSuite.h"

static CANFeedbackReceiverSuite suite_CANFeedbackReceiverSuite;

static CxxTest::List Tests_CANFeedbackReceiverSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_CANFeedbackReceiverSuite( "unit-tests/CANFeedbackReceiverSuite.h", 18, "CANFeedbackReceiverSuite", suite_CANFeedbackReceiverSuite, Tests_CANFeedbackReceiverSuite );

static class TestDescription_suite_CANFeedbackReceiverSuite_test_NodeSendsMessageOnFrameReceive : public CxxTest::RealTestDescription {
public:
 TestDescription_suite_CANFeedbackReceiverSuite_test_NodeSendsMessageOnFrameReceive() : CxxTest::RealTestDescription( Tests_CANFeedbackReceiverSuite, suiteDescription_CANFeedbackReceiverSuite, 59, "test_NodeSendsMessageOnFrameReceive" ) {}
 void runTest() { suite_CANFeedbackReceiverSuite.test_NodeSendsMessageOnFrameReceive(); }
} testDescription_suite_CANFeedbackReceiverSuite_test_NodeSendsMessageOnFrameReceive;

#include <cxxtest/Root.cpp>
const char* CxxTest::RealWorldDescription::_worldName = "cxxtest";
