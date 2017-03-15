#include "catch.hpp"
#include <string>
#include "xBee/xBee.h"

TEST_CASE("Test xBee find xml message")
{
	xBee xbee;
	std::string buffer;
	std::string message;

	SECTION("Find complete xml")
	{
		buffer = "--<message>text</message>--";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("<message>text</message>") == 0);
		REQUIRE(buffer.compare("--") == 0);
	}

	SECTION("Find latest xml")
	{
		buffer = "--<message>text1</message>----<message>text2</message>--";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("<message>text2</message>") == 0);
		REQUIRE(buffer.compare("--") == 0);
	}

	SECTION("Find complete xml after adding tags separately")
	{
		buffer = "--<message>te";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("") == 0);
		REQUIRE(buffer.compare("--<message>te") == 0);

		buffer += "xt</message>--";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("<message>text</message>") == 0);
		REQUIRE(buffer.compare("--") == 0);
	}

	SECTION("Find complete xml after adding tags and content separately")
	{
		buffer = "--<message>";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("") == 0);
		REQUIRE(buffer.compare("--<message>") == 0);

		buffer += "text";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("") == 0);
		REQUIRE(buffer.compare("--<message>text") == 0);

		buffer += "</message>--";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("<message>text</message>") == 0);
		REQUIRE(buffer.compare("--") == 0);
	}

	SECTION("Find complete xml after adding split tags")
	{
		buffer = "--<mess";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("") == 0);
		REQUIRE(buffer.compare("--<mess") == 0);

		buffer += "age>text</mess";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("") == 0);
		REQUIRE(buffer.compare("--<message>text</mess") == 0);

		buffer += "age>--";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("<message>text</message>") == 0);
		REQUIRE(buffer.compare("--") == 0);
	}

	SECTION("Find complete xml when an incomplete one comes after")
	{
		buffer = "--<message>text1</message>----<message>te";
		message = xbee.findXmlMessage(&buffer);
		REQUIRE(message.compare("<message>text1</message>") == 0);
		REQUIRE(buffer.compare("----<message>te") == 0);
	}
}