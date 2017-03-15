#include "catch.hpp"
#include "thread/ExternalCommand.h"
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <future>         // std::async, std::future
#include <string>


ExternalCommand externalCmd("0",0,0,0);

TEST_CASE("testExternalCommand")
{
	SECTION("Test multiple simultaneous writes")
	{
    class TestHelp {
    public:
      // write to data structure
      static void write(bool autorun, int ruddercmd, int sailcmd)
      {
        std::string timestamp("1");
        for (int i = 0 ; i < 25 ; i++) {
          externalCmd.setData(timestamp, autorun, ruddercmd, sailcmd);
          //std::cout << sailcmd;
        }
      }

      static void read(bool* isSuccess) {
        //bool isSuccess = true;
        for (int i = 0 ; i < 100 && *isSuccess ; i++) {
          if (!(externalCmd.getRudderCommand() >= 0 &&
              externalCmd.getRudderCommand() <= 3)) {
            *isSuccess = false;
          }
          //std::cout << "0";
        }
      }
      

      static bool simultaneousWrite()
      {
        bool returnBool = true;


        std::thread th1 (TestHelp::write, true,1,1);
        std::thread th2 (TestHelp::write, true,3,2);
        std::thread th5 (TestHelp::read, &returnBool);
        std::thread th3 (TestHelp::write, false,1,3);
        std::thread th4 (TestHelp::write, true,2,4);

        th1.join();
        th2.join();
        th3.join();
        th4.join();
        th5.join();
        
        return returnBool;
      }
    };

		// call function asynchronously:
		std::future<bool> fut = std::async (TestHelp::simultaneousWrite);
		fut.wait();
		// retrieve return value
		REQUIRE(fut.get());
	}
}