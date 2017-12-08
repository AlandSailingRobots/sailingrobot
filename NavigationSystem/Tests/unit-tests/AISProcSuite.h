/****************************************************************************************
*
* File:
* 		AISProcSuite.h
*
* Purpose:
*		Testing the AISProcessing node!
*   One test that tests if we receive an AISData messages
*   And another message that tests if we can add data to the collidable manager
*
*
***************************************************************************************/
#include "WorldState/AISProcessing.h"
#include "../cxxtest/cxxtest/TestSuite.h"
#include "Messages/StateMessage.h"
#include "DataBase/DBHandler.h"
#include "MessageBus/MessageBus.h"
#include "TestMocks/MessageLogger.h"
#include "WorldState/CollidableMgr/CollidableMgr.h"
#include "MessageBusTestHelper.h" 

#define AISPROC_TEST_COUNT 2

class AISProcSuite : public CxxTest::TestSuite {
public:
  AISProcessing* aisProc = 0;
  DBHandler* dbhandler;
  CollidableMgr cMgr;
  MockNode* mockNode;
  bool mockNodeRegistered = false;
  MessageBus messageBus;
  std::unique_ptr<MessageBusTestHelper> messageBusHelper;

  int testCount;

  void setUp() {
    if (aisProc == 0) {
      mockNode = new MockNode(messageBus, mockNodeRegistered);
      dbhandler = new DBHandler("../asr.db");
      aisProc = new AISProcessing(messageBus, *dbhandler, &cMgr);

      aisProc->start();

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      messageBusHelper.reset(new MessageBusTestHelper(messageBus));
    }
    testCount++;
  }

  void tearDown() {
    if (testCount == AISPROC_TEST_COUNT) {
      aisProc->stop();
      messageBusHelper.reset();
      delete mockNode;
      delete aisProc;
    }
  }

  void test_ReceiveMessage(){
    std::vector<AISVessel> AISList;
    std::vector<AISVesselInfo> AISInfo;
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
    AISVesselInfo i1;
    i1.MMSI=1;
    i1.length=15;
    i1.beam = 4;
    AISInfo.push_back(i1);

    MessagePtr mockAISMsg = std::make_unique<AISDataMsg>(AISList, AISInfo, 60.1, 19.1);
    messageBus.sendMessage(std::move(mockAISMsg));

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    TS_ASSERT(mockNode->m_MessageReceived);
  }

  void test_ReceiveData() {
    std::vector<AISVessel> AISList;
    std::vector<AISVesselInfo> AISInfo;
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
    AISVesselInfo i1;
    i1.MMSI=1;
    i1.length=15;
    i1.beam = 4;
    AISInfo.push_back(i1);

    MessagePtr mockAISMsg = std::make_unique<AISDataMsg>(AISList, AISInfo, 60.1, 19.1);
    messageBus.sendMessage(std::move(mockAISMsg));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    cMgr.getAISContacts();
    TS_ASSERT_EQUALS(cMgr.getAISContacts().length(),3);
  }
};
