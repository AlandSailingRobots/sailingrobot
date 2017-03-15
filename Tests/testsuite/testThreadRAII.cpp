#include "catch.hpp"
#include <iostream>
#include "thread/ThreadRAII.h"
#include <stdexcept>



using namespace std;

TEST_CASE("testThreadRAII"){
	SECTION("test init") {
		
		ThreadRAII *test = new ThreadRAII(std::thread( []() { return 1; }), ThreadRAII::DtorAction::join);
		
		delete test; 

		REQUIRE(true);
	}
/*
	SECTION("regular thread crashing"){
		std::thread *t = new thread([]() { return 1; });

		delete t;

		//std::thread( []() { return 1; }

		REQUIRE(true);
	}


	*/

	

}


