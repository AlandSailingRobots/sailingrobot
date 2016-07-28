#include "xBee.h"
#include "SystemServices/Logger.h"

std::string xBee::findXmlMessage(std::string* buffer) {

	std::string result = "";

	std::string messageStartTag = "<message>";
	std::string messageEndTag = "</message>";
	std::size_t pos_start = buffer->rfind(messageStartTag);
	std::size_t pos_end = buffer->rfind(messageEndTag);

	// in case the start of an incomplete message comes after a complete message
	std::size_t pos_start2 = buffer->rfind(messageStartTag, pos_start-1);

	//std::cout << "Start tag pos: " << pos_start << '\n' << 
    //			 "End tag pos: " << pos_end << std::endl;

	if (pos_start != std::string::npos &&
		pos_end != std::string::npos) {

		if (pos_start < pos_end) {
			result = buffer->substr(pos_start, 
				pos_end - pos_start + messageEndTag.size());
			buffer->erase(0, pos_end + messageEndTag.size());
		}
		else if (pos_start2 < pos_end) {
			result = buffer->substr(pos_start2, 
				pos_end - pos_start2 + messageEndTag.size());
			buffer->erase(0, pos_end + messageEndTag.size());
		}
	}

	if(buffer->size() > 500) {

		buffer->clear();
	}

	return result;
}

int xBee::init(){

	// this setting needs a udev rule in host system to work (alternative is dynamic USB-slot)
	// see Static_USB_Device_Names in project installation folder
	std::string deviceName = "/dev/xbee";
	int fd;

	if( (fd = serialOpen(deviceName.c_str(), BAUD_RATE) ) < 0) {
		Logger::error("XbeeSync::%d Error: %d", (int)__LINE__, errno);
	}

	return fd;
}

std::string xBee::receiveData(int fd) {

	std::string printer;

	int available = serialDataAvail(fd);

	if (available != -1){
		while (available > 0){
			printer += serialGetchar(fd);
			available--;
		}
		m_receivedBuffer += printer;
	}else{
		printer = "Data reception failed.";
	}

	return printer;
}

std::string xBee::receiveXMLData(int fd) {
	std::string printer;

	int available = serialDataAvail(fd);

	if (available != -1){
		while (available > 0){
			printer += serialGetchar(fd);
			available--;
		}
		m_receivedBuffer += printer;
	}else{
		printer = "Data reception failed.";
	}

	return findXmlMessage(&m_receivedBuffer);
}

void xBee::transmitData(int fd, std::string data) {

	serialPuts(fd, data.c_str());
}

/*
std::string xBee::readOutput(int fd){

	std::string printer;

	int available = serialDataAvail(fd);

	if (available != -1){
		while (available > 0){
			printer += serialGetchar(fd);
			available--;
		}
		m_receivedBuffer += printer;
	}else{
		printer = "Data reception failed.";
	}

	return findXmlMessage(&m_receivedBuffer);
}

void xBee::printInput(std::string input, int fd){

	serialPuts(fd, input.c_str());
}

void xBee::sendXML(int fd, std::string output){

	serialPuts(fd, output.c_str());
}

std::string xBee::findXmlMessage(std::string* buffer) {

	std::string result = "";

	std::string messageStartTag = "<message>";
	std::string messageEndTag = "</message>";
	std::size_t pos_start = buffer->rfind(messageStartTag);
	std::size_t pos_end = buffer->rfind(messageEndTag);

	// in case the start of an incomplete message comes after a complete message
	std::size_t pos_start2 = buffer->rfind(messageStartTag, pos_start-1);

	//std::cout << "Start tag pos: " << pos_start << '\n' << 
    //			 "End tag pos: " << pos_end << std::endl;

	if (pos_start != std::string::npos &&
		pos_end != std::string::npos) {

		if (pos_start < pos_end) {
			result = buffer->substr(pos_start, 
				pos_end - pos_start + messageEndTag.size());
			buffer->erase(0, pos_end + messageEndTag.size());
		}
		else if (pos_start2 < pos_end) {
			result = buffer->substr(pos_start2, 
				pos_end - pos_start2 + messageEndTag.size());
			buffer->erase(0, pos_end + messageEndTag.size());
		}
	}

	if(buffer->size() > 500) {

		buffer->clear();
	}

	return result;
}
*/
