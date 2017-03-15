#include "catch.hpp"
#include "thread/SystemState.h"
#include <iostream>       // std::cout
#include <thread>         // std::thread
#include <future>         // std::async, std::future
#include "models/PositionModel.h"

// data
SystemStateModel model_init = SystemStateModel(
	GPSModel("",PositionModel(0,0),0,0,0,0),
	WindsensorModel(0,0,0),
	CompassModel(0,0,0,AccelerationModel(0,0,0) ),
	AnalogArduinoModel(0,0,0,0),
	0,
	0
);

SystemState systemState(model_init);

TEST_CASE("testSystemState")
{
	SECTION("Test multiple write/read")
	{
		class TestHelp {
		public:

			// write to data structure
			static void write(int num)
			{
				// data
				SystemStateModel model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,0,0,0),
					WindsensorModel(0,0,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
				model.rudder = num;

				for (int i = 0 ; i < 25 ; i++) {
					systemState.setData(model);
					//std::cout << num;
				}
			}

			// write to data structure
			static void read(bool* isSuccess)
			{
				// data
				SystemStateModel model = SystemStateModel(
					GPSModel("",PositionModel(0,0),0,0,0,0),
					WindsensorModel(0,0,0),
					CompassModel(0,0,0,AccelerationModel(0,0,0) ),
					AnalogArduinoModel(0,0,0,0),
					0,
					0
				);
				systemState.getData(model);

				for (int i = 0 ; i < 50 && *isSuccess ; i++) {
					if (!(model.rudder >= 0 &&
						model.rudder <= 4))
					{
						*isSuccess = false;
					}
					//std::cout << "0";
				}
			}

			static bool simultaneousWriteRead()
			{
				bool isSuccess = true, isSuccess2 = true;

				std::thread th1 (TestHelp::write, 1);
				std::thread th2 (TestHelp::write, 4);
				std::thread thw1 (TestHelp::read, &isSuccess);
				std::thread th3 (TestHelp::write, 2);
				std::thread th4 (TestHelp::write, 3);
				std::thread thw2 (TestHelp::read, &isSuccess2);
				th1.join();
				th2.join();
				th3.join();
				th4.join();
				thw1.join();
				thw2.join();

				if (isSuccess && isSuccess2)
					return true;
				else return false;
			}
		};

		// call function asynchronously:
		std::future<bool> fut = std::async (TestHelp::simultaneousWriteRead);
		fut.wait();
		// retrieve return value
		REQUIRE(fut.get());
	}
}
