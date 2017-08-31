#include "xBeeRelay.h"

xBeeRelay::xBeeRelay() {
	try {
		m_fd = m_xBee.init();
	} catch (const char* e) {
		std::cout << e << std::endl;
	}
	m_curl = curl_easy_init();
	
}

xBeeRelay::~xBeeRelay() {}

size_t xBeeRelay::write_to_string(void *ptr, size_t size, size_t count, void *stream) {
  ((std::string*)stream)->append((char*)ptr, 0, size*count);
  return size*count;
}

void xBeeRelay::doCurl(std::string url, std::string* response) {
	if (m_curl) {
		curl_easy_setopt(m_curl, CURLOPT_URL, url.c_str());
		curl_easy_setopt(m_curl, CURLOPT_WRITEFUNCTION, write_to_string);
		curl_easy_setopt(m_curl, CURLOPT_WRITEDATA, response);

		CURLcode res = curl_easy_perform(m_curl);

		if(res != CURLE_OK) {
			throw ( std::string("Curl error: ") + curl_easy_strerror(res) ).c_str();
		}
	}
}

std::string xBeeRelay::receiveData() {
	return m_xBee.receiveXMLData(m_fd);
}

void xBeeRelay::relayData(std::string data) {
	m_xBee.transmitData(m_fd, data);
}

std::string xBeeRelay::extractMessage(std::string data) {
	const int tagSize = 9;

	if (data != "") {
		unsigned first = data.find("<message>");
		unsigned last = data.find("</message>", first);
		return data.substr(first + tagSize, last - first - tagSize);
	}
	return data;
}

void xBeeRelay::pushLogsToServer(std::string data) {
	std::string response = "";
	std::string serverURL = "http://www.sailingrobots.com/testdata/sync";
	std::string serverCall = "/?serv=pushLogs&id=sailbot&pwd=sailbot&data="+data;
	std::string url = serverURL + serverCall;
	if (data != "") {
		doCurl(url, &response);
	}
}

bool xBeeRelay::checkForConfigUpdates() {
	std::string response = "";
	std::string localURL = "10.168.4.105:80/Remote-sailing-robots/sync";
	std::string serverCall = "/?serv=checkIfNewConfigs&id=boat02&pwd=sailbot";

	std::string url = localURL + serverCall;

	doCurl(url, &response);
	
	return stoi(response);
}

std::string xBeeRelay::getConfigsFromWeb() {
	std::string response = "";
	std::string localURL = "10.168.4.105:80/Remote-sailing-robots/sync";
	std::string serverCall = "/?serv=getAllConfigs&id=boat02&pwd=sailbot";

	std::string url = localURL + serverCall;

	doCurl(url, &response);
	
	return response;
}