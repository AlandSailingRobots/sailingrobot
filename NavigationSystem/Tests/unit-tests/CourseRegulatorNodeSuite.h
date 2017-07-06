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
#include "Messages/ActuatorPositionMsg.h"
#include "../../MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "Messages/StateMessage.h"
#include "Messages/DesiredCourseMsg.h"
#include "Math/Utility.h"
#include "DataBase/DBHandler.h"


// For std::this_thread
#include <chrono>
#include <thread>
#include <iostream>

#define COURSE_REGULATORNODE_TEST_COUNT 8

class CourseRegulatorNodeSuite : public CxxTest::TestSuite
{
public:

    CourseRegulatorNode* cRegulatorNode;
    DBHandler* dbHandler;
    MockNode* mockNode;
    bool nodeRegistered = false;

    double lTime = .5;
    double MaxRudAng = 30;

    std::thread* thr;
    int testCount = 0;

    static MessageBus& msgBus(){
        static MessageBus* mbus = new MessageBus();
        return *mbus;
    }

/*
    static DBHandler& dbHandler(){
        static DBHandler* dbh = new DBHandler("./asr.db"); 
        return *dbh;
    }
*/

    // ----------------
    // Send messages 
    // ----------------
    static void runMessageLoop()
    {
        msgBus().run();
    }

    // ----------------
    // Setup the objects to test
    // ----------------
    void setUp()
    {
            
         // Object to simulate the 
        mockNode = new MockNode(msgBus(), nodeRegistered);
        
        // setup them up once in this test, delete them when the program closes
        if(cRegulatorNode == 0)
        {  
            //attente = new std::chrono::milliseconds(2600);
            //std::cout << " Count of ticks : " << attente->count() << std::endl;
            dbHandler = new DBHandler("./asr.db");
            Logger::DisableLogging();   
            
            cRegulatorNode = new CourseRegulatorNode(msgBus(),*dbHandler,lTime,MaxRudAng,0,0);
            cRegulatorNode->start();
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
            thr = new std::thread(runMessageLoop);
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
            delete cRegulatorNode;
        }
        delete mockNode;
    }

    // ----------------
    // Test Initialisation of the object
    // ----------------
    void test_CourseRegulatorNodeInit(){
        TS_ASSERT(nodeRegistered);
    }

    // ----------------
    // Test to see if a message concerning the node will be listened
    // ----------------
    void test_CourseRegulatorMsgListener(){
        double heading = 10;
        double speed = 1;

         // Test listening State Message
        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        msgBus().sendMessage(std::move(stateData));
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
        msgBus().sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Check if the message has been received by the object of simulation and without modification without desired_heading
        TS_ASSERT_DELTA(mockNode->m_rudderPosition, 0, 1e-7); //Heading_error_value : 370
        // TODO: See how to find when the value is not good and interpreted it otherwise the utility::limitedAnnglerange change 370

        int16_t desiredcourse = 15;
        // Test listening desired course Message
        MessagePtr desiredCourseData = std::make_unique<DesiredCourseMsg>(desiredcourse);
        msgBus().sendMessage(std::move(desiredCourseData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        double rudderAngle = Utility::sgn(speed)*(sin(heading-static_cast<double>(desiredcourse))*MaxRudAng);
        double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        TS_ASSERT_EQUALS(courseRegulatorRudderAngle,rudderAngle);
        //Test if not calculate false value without the desired heading (includ Desired_heading = 0)
        // Check after if the value is correct
    }

    // ----------------
    // Test to see if the message, sent by the node, is with the values
    // ----------------
    void test_CourseRegulatorCourseMsgActuatorData(){
        double rudderPos = 10;
        double sailPos = 20;

        // Test listening State Message
        MessagePtr actuatorData = std::make_unique<ActuatorPositionMsg>(rudderPos,sailPos);
        msgBus().sendMessage(std::move(actuatorData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // Check if the message has been received by the object of simulation and without modification without desired_heading
        TS_ASSERT_EQUALS(mockNode->m_rudderPosition, rudderPos); 
        TS_ASSERT_EQUALS(mockNode->m_sailPosition, sailPos);
    }

    // ----------------
    // Test for desired heading in the opposite way
    // ----------------
    void test_CourseRegulatorNodeOppositeDesireHeading(){
        double heading = -10;
        double speed = 1;

        // Test listening State Message
        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        msgBus().sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int16_t desiredcourse = -110;
        // Test listening desired course Message
        MessagePtr desiredCourseData = std::make_unique<DesiredCourseMsg>(desiredcourse);
        msgBus().sendMessage(std::move(desiredCourseData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        double rudderAngle = Utility::sgn(speed)*Utility::sgn((sin(heading-static_cast<double>(desiredcourse)))*MaxRudAng);
        double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        TS_ASSERT_EQUALS(courseRegulatorRudderAngle,rudderAngle);
    }

    // ----------------
    // Test for a speed in the wrong sense
    // ----------------
    void test_CourseRegulatorNodeNegSpeed(){
        double heading = 10;
        double speed = -1;

        // Test listening State Message
        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        msgBus().sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int16_t desiredcourse = -17;
        // Test listening desired course Message
        MessagePtr desiredCourseData = std::make_unique<DesiredCourseMsg>(desiredcourse);
        msgBus().sendMessage(std::move(desiredCourseData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        double rudderAngle = Utility::sgn(speed)*sin(heading-static_cast<double>(desiredcourse))*MaxRudAng;
        double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        TS_ASSERT_EQUALS(courseRegulatorRudderAngle,rudderAngle);
        
    }

    // ----------------
    // Test for desired heading in the opposite way
    // ----------------
     void test_CourseRegulatorNodeOppositeAndNegSpeed(){
        double heading = 10;
        double speed = -1;

        // Test listening State Message
        MessagePtr stateData = std::make_unique<StateMessage>(heading,60.09726,19.93481,speed,0);
        msgBus().sendMessage(std::move(stateData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        int16_t desiredcourse = 200;
        // Test listening desired course Message
        MessagePtr desiredCourseData = std::make_unique<DesiredCourseMsg>(desiredcourse);
        msgBus().sendMessage(std::move(desiredCourseData));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        double rudderAngle = Utility::sgn(speed)*Utility::sgn((sin(heading-static_cast<double>(desiredcourse)))*MaxRudAng);
        double courseRegulatorRudderAngle = mockNode->m_rudderPosition;
        TS_ASSERT_EQUALS(courseRegulatorRudderAngle,rudderAngle);
    }

    // ----------------
    // Test for update frequency
    // ----------------
    void test_CourseRegulatorUpdateFrequency(){
        double newLoopTime= 0.7;
        dbHandler->changeOneValue("course_regulator","1",".7","loopTime");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        double courseRegulatorFrequence = cRegulatorNode->getFrequencyThread();
        TS_ASSERT_DIFFERS(courseRegulatorFrequence,newLoopTime); 
    }

};

















