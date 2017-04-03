

#pragma once

#include "../../SystemServices/CANService.h"
#include "../cxxtest/cxxtest/TestSuite.h"

class CANServiceSuite : public CxxTest::TestSuite {
public:

  

  void setUp() {

  }

  void tearDown() {

  }

  void testRegisterNode(void){
    TS_ASSERT(2 != 1);
  }
};
