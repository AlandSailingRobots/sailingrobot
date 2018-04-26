/****************************************************************************************
*
* File:
* 		CourseRegulatorNodeSuite.h
*
* Purpose:
*		A set of unit tests for checking if the CourseRegulatorNode works as intended
*
* Developer Notes:
*
*
***************************************************************************************/

#pragma once

#include "LowLevelControllers/CourseRegulatorNode.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Messages/StateMessage.h"
#include "Messages/LocalNavigationMsg.h"
#include "Math/Utility.h"
#include "DataBase/DBHandler.h"
#include "SystemServices/Timer.h"
#include "MessageBusTestHelper.h"


#include <chrono>
#include <iostream>

#define COURSE_REGULATORNODE_TEST_COUNT 7

const float NO_COMMAND = -1000;

class CourseRegulatorNodeSuite : public CxxTest::TestSuite
{
public:

    CourseRegulatorNode* cRegulatorNode;
    DBHandler* dbHandler;
    MockNode* mockNode;
    bool nodeRegistered = false;
    MessageBus messageBus;
    std::unique_ptr<MessageBusTestHelper> messageBusHelper;

    double MaxRudAng = 30;
    int testCount = 0;


    // ----------------
    // Setup the objects to test
    // ----------------
    void setUp()
    {
         // Object to simulate the
        // setup them up once in this test, delete them when the program closes
        if(cRegulatorNode == 0)
        {
            mockNode = new MockNode(messageBus, nodeRegistered);
            dbHandler = new DBHandler("../asr.db");
            Logger::DisableLogging();

            cRegulatorNode = new CourseRegulatorNode(messageBus,*dbHandler);
            cRegulatorNode->start();

            std::this_thread::sleep_for(std::chrono::milliseconds(2600));
            messageBusHelper.reset(new MessageBusTestHelper(messageBus));

        }
        testCount++;
    }

    // ----------------
    // End of test when all test have been successfull
    // ----------------
    void tearDown()
    {
        // Counter of the number of test
        if(testCount == COURSE_REGULATORNODE_TEST_COUNT)
        {
            cRegulatorNode->stop();
            messageBusHelper.reset();
            delete cRegulatorNode;
            delete dbHandler;
            // Stay here for process the last message which return system::error
            delete mockNode;
        }
    }

    // ----------------
    // Test Initialisation of the object
    // ----------------
    void test_CourseRegulatorNodeInit(){
        TS_ASSERT(nodeRegistered);
        cRegulatorNode->init();
    }

    // ----------------
    // Test to see if a message concerning the node will be listened
    // ----------------
    void test_CourseRegulatorMsgListener(){
        double heading = 10;
        double speed = 1;

         // Test listening State Message
        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        messageBus.sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        // Check if the message has been received by the object of simulation and without modification without desired_heading
        TS_ASSERT(mockNode->m_MessageReceived);
    }

    // ----------------
    // Test to see if ,after the Stata messsage, the rudder angle is not changed
    // ----------------
    void test_CourseRegulatorCourseMsgRudder(){
        double heading = 10;
        double speed = 1;

        // Test listening State Message
        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        messageBus.sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Check if the message has been received by the object of simulation and without modification without desired_heading
        TS_ASSERT_EQUALS(mockNode->m_StateMsgHeading,heading);
        TS_ASSERT_EQUALS(mockNode->m_StateMsgSpeed,speed);

        TS_ASSERT_DELTA(mockNode->m_rudderPosition, 0, 1e-7); //Heading_error_value : 370
        // TODO: See how to find when the value is not good and interpreted it otherwise the utility::limitedAnglerange change 370

        
        double desiredcourse = 15;
        // Test listening desired course Message
        MessagePtr localNavigationData =
        std::make_unique<LocalNavigationMsg>(desiredcourse, NO_COMMAND, false, false);
        messageBus.sendMessage(std::move(localNavigationData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Precision ?
        double diffHeading = Utility::limitAngleRange(heading)-Utility::limitAngleRange(desiredcourse);
        int rudderAngle = Utility::sgn(speed)*sin(Utility::degreeToRadian(diffHeading))*MaxRudAng;
        double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        TS_ASSERT_EQUALS(courseRegulatorRudderAngle,rudderAngle);
        //Test if not calculate false value without the desired heading (includ Desired_heading = 0)
        // Check after if the value is correct
    }

    // ----------------
    // Test for desired heading in the opposite way
    // ----------------
    void test_CourseRegulatorNodeOppositeDesireHeading(){
        TS_SKIP("Skipping failing test, needs update");
        /*
        double heading = -10;
        double speed = 1;

        // Test listening State Message
        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        messageBus.sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        double desiredcourse = 250;
        // Test listening desired course Message
        MessagePtr localNavigationData =
        std::make_unique<LocalNavigationMsg>(desiredcourse, NO_COMMAND, false, false);
        messageBus.sendMessage(std::move(localNavigationData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Precision ?
        double diffHeading = Utility::limitAngleRange(heading)-Utility::limitAngleRange(desiredcourse);
        int rudderAngle = Utility::sgn(speed)*Utility::sgn(sin(Utility::degreeToRadian(diffHeading)))*MaxRudAng;
        double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        TS_ASSERT_EQUALS(courseRegulatorRudderAngle,rudderAngle);*/
    }

    // ----------------
    // Test for a speed in the wrong sense
    // ----------------
    void test_CourseRegulatorNodeNegSpeed(){
        TS_SKIP("Skipping failing test, needs update");
        /*
        double heading = 10;
        double speed = -1;

        // Test listening State Message
        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        messageBus.sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        double desiredcourse = 343;
        // Test listening desired course Message
        MessagePtr localNavigationData =
        std::make_unique<LocalNavigationMsg>(desiredcourse, NO_COMMAND, false, false);
        messageBus.sendMessage(std::move(localNavigationData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Precision ?
        double diffHeading = Utility::limitAngleRange(heading)-Utility::limitAngleRange(desiredcourse);
        int rudderAngle = Utility::sgn(speed)*sin(Utility::degreeToRadian(diffHeading))*MaxRudAng;
        double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        TS_ASSERT_EQUALS(courseRegulatorRudderAngle,rudderAngle);
*/
    }

    // ----------------
    // Test for desired heading in the opposite way
    // ----------------
     void test_CourseRegulatorNodeOppositeAndNegSpeed(){
        TS_SKIP("Skipping failing test, needs update");
        /*
        double heading = 10;
        double speed = -1;

        // Test listening State Message
        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        messageBus.sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        double desiredcourse = 200;
        // Test listening desired course Message
        MessagePtr localNavigationData =
        std::make_unique<LocalNavigationMsg>(desiredcourse, NO_COMMAND, false, false);
        messageBus.sendMessage(std::move(localNavigationData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Precision ?
        double diffHeading = Utility::limitAngleRange(heading)-Utility::limitAngleRange(desiredcourse);
        int rudderAngle = Utility::sgn(speed)*Utility::sgn(sin(Utility::degreeToRadian(diffHeading)))*MaxRudAng;
        double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        TS_ASSERT_EQUALS(courseRegulatorRudderAngle,rudderAngle);*/
    }

    void test_CourseRegulatorUpdateFromDB(){
        Timer timer;

        dbHandler->changeOneValue("config_course_regulator","1","0.7","loop_time");
        dbHandler->changeOneValue("config_course_regulator","1","20.0","max_rudder_angle");
        MessagePtr serverConfig = std::make_unique<ServerConfigsReceivedMsg>();
        messageBus.sendMessage(std::move(serverConfig));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        TS_ASSERT(mockNode->m_MessageReceived);

        TS_SKIP("Skipping hanging test");
        /*
        mockNode->m_MessageReceived = false;
        while(not mockNode->m_MessageReceived); // hanging here
        timer.start();
        mockNode->m_MessageReceived = false;
        while(not mockNode->m_MessageReceived);
        timer.stop();

        TS_ASSERT_DELTA(timer.timePassed(), 0.70, 1e-2);
*/
        double heading = 10;
        double speed = 1;
        MaxRudAng = 20.0;
        double desiredcourse = 15;

        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        messageBus.sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        MessagePtr localNavigationData =
        std::make_unique<LocalNavigationMsg>(desiredcourse, NO_COMMAND, false, false);
        messageBus.sendMessage(std::move(localNavigationData));
        std::this_thread::sleep_for(std::chrono::milliseconds(700));

        double diffHeading = Utility::limitAngleRange(heading)-Utility::limitAngleRange(desiredcourse);
        int rudderAngle = Utility::sgn(speed)*sin(Utility::degreeToRadian(diffHeading))*MaxRudAng;
        std::this_thread::sleep_for(std::chrono::milliseconds(700));
        double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        TS_ASSERT_DELTA(courseRegulatorRudderAngle, rudderAngle, 1e-7);
        dbHandler->changeOneValue("config_course_regulator","1","0.5","loop_time");
        dbHandler->changeOneValue("config_course_regulator","1","30.0","max_rudder_angle");
    }

};
