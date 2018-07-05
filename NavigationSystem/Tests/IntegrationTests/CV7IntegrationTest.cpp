#include <iostream>
#include <string>
#include <math.h>
#include <unistd.h>
#include <atomic>
#include <chrono>
#include <thread>
#include "logger/Logger.h"
#include "../Math/Utility.h"


using namespace std;
        const string PORTNAME = "/dev/ttyS0";
        
	/**
	 *  mode 0 = mean, 1 = median
	 */
vector<map<string, double>> getHZReadings(int mSeconds, bool mode) {
	int baudRate = 4800;
	int bufferSize = 1;
	CV7 sensor;
	try {
		sensor.loadConfig(PORTNAME, baudRate);
	}
	catch (const char* exception) {
		cout << exception << endl;
	}

	sensor.setBufferSize(bufferSize); //Optional. Default: 30
	sensor.setUseMean(mode);
	vector<map<string, double>> resultVector;
	int polling = 30;
	cout<<"loading values. 1 value/"<<mSeconds<<" ms"<<endl;
	std::atomic<double> start(0);
	std::atomic<double> sectionStart(0);
	std::atomic<double> time(0);
	std::atomic<int> sleepTime(0);
	map<string, double> temp;
	while (polling--) {
		start = clock();
		temp.clear();
		try {
			sectionStart = clock();
			std::string data = sensor.refreshData();
			Logger::info("poll %d", polling);
			Logger::info("refresh time :[%d] mSeconds", 1000* ( (clock() - sectionStart ) / CLOCKS_PER_SEC) );

			sectionStart = clock();
			sensor.parseData(data);
			Logger::info("parse time :[%d] mSeconds", 1000* ((clock() - sectionStart ) / CLOCKS_PER_SEC) );
			temp.insert(make_pair("direction", sensor.getDirection()));
			temp.insert(make_pair("speed", sensor.getSpeed()));
			temp.insert(make_pair("temp", sensor.getTemperature()));
			resultVector.push_back(temp);
		}
		catch (const char* exception) {
			Logger::error(exception);
		}
		time = 1000* ((clock()-start) / CLOCKS_PER_SEC);
		if (time < mSeconds ) {
			sleepTime = mSeconds - time;
			Logger::info("sleep for :[%d] mSeconds. Current time: %s", (clock() - sectionStart ) / CLOCKS_PER_SEC, std::to_string(time).c_str());
			std::this_thread::sleep_for(std::chrono::milliseconds(sleepTime));
			Logger::info("after sleep time :[%d] mSeconds", 1000* ((clock() - sectionStart ) / CLOCKS_PER_SEC) );
		}
		else {
			Logger::info("No sleep needed!!  current time: %s", std::to_string(time).c_str());
		}
		Logger::info("Total poll loop time time :[%d] mSeconds", ( (clock() - sectionStart ) / CLOCKS_PER_SEC));
	}
	cout<<"loading done"<<endl;
	return resultVector;
}
string prettyString (int number, float sin, float cos){
	string resultString = "no " + std::to_string(number);
	resultString += "  sin: " + std::to_string(sin);
	resultString += "  cos: " + std::to_string(cos);
	resultString += "\n";
	return resultString;
}

string sinAll(vector<map<string, double>> values){
	vector<float> fiveVector_s;
	vector<float> tenVector_s;
	vector<float> twentyVector_s;
	vector<float> fortyVector_s;
	vector<float> fiveVector_c;
	vector<float> tenVector_c;
	vector<float> twentyVector_c;
	vector<float> fortyVector_c;
	vector<float> resultFiveVector_s;
	vector<float> resultTenVector_s;
	vector<float> resultTwentyVector_s;
	vector<float> resultFortyVector_s;
	vector<float> resultFiveVector_c;
	vector<float> resultTenVector_c;
	vector<float> resultTwentyVector_c;
	vector<float> resultFortyVector_c;
	vector<string> resultStringVector;
	for (map<string, double> &m : values) {
		double tempSin = sin(m.find("direction")->second * (M_PI / 180));
		double tempCos = cos(m.find("direction")->second * (M_PI / 180));
		resultStringVector.push_back(
				"map: [ dir: " + to_string(m.find("direction")->second) + " sin dir: "
				+ to_string(tempSin)	+ " cos dir: "	+ to_string(tempCos) + "]"
		);
		if (fiveVector_s.size() > 5)		{
			resultFiveVector_s.push_back(Utility::mean(fiveVector_s));
			resultFiveVector_c.push_back(Utility::mean(fiveVector_c));
			fiveVector_s.clear();
			fiveVector_c.clear();
		}
		if (tenVector_s.size() > 10) 		{
			resultTenVector_s.push_back(Utility::mean(tenVector_s));
			resultTenVector_c.push_back(Utility::mean(tenVector_c));
			tenVector_s.clear();
			tenVector_c.clear();
		}
		if (twentyVector_s.size() > 20) 	{
			resultTwentyVector_s.push_back(Utility::mean(twentyVector_s));
			resultTwentyVector_c.push_back(Utility::mean(twentyVector_c));
			twentyVector_s.clear();
			twentyVector_c.clear();
		}
		if (fortyVector_s.size() > 40) 	{
			resultFortyVector_s.push_back(Utility::mean(fortyVector_s));
			resultFortyVector_c.push_back(Utility::mean(fortyVector_c));
			fortyVector_s.clear();
			fortyVector_c.clear();
		}
		fiveVector_s.push_back(tempSin);
		tenVector_s.push_back(tempSin);
		twentyVector_s.push_back(tempSin);
		fortyVector_s.push_back(tempSin);
		fiveVector_c.push_back(tempCos);
		tenVector_c.push_back(tempCos);
		twentyVector_c.push_back(tempCos);
		fortyVector_c.push_back(tempCos);

	}
	string resultString = "resultstring:\n";
	Logger::info(std::string("resultstring : "));
	for (string s : resultStringVector) {
		resultString += s;
		Logger::info(std::string(s));
	}
	resultString += "fiveVector:\n";
	Logger::info(std::string("fiveVector : "));
	int i = 0;
	for (float f : resultFiveVector_s) {
		resultString += prettyString(i, f, resultFiveVector_c.at(i));
		Logger::info( std::string("no ")	+ (std::to_string(i)
			+ std::string("  sin: ") + std::to_string(f)
			+ std::string("  cos: ") + std::to_string(resultFiveVector_c.at(i)))
			);
		i++;
	}
	resultString += "tenVector:\n";
	Logger::info(std::string("tenVector : "));
	i = 0;
	for (float f : resultTenVector_s) {
		resultString += prettyString(i, f, resultTenVector_c.at(i));
		Logger::info( std::string("no ")	+ (std::to_string(i)
			+ std::string("  sin: ") + std::to_string(f)
			+ std::string("  cos: ") + std::to_string(resultTenVector_c.at(i))));
		i++;
	}
	resultString += "twentyVector:\n";
	Logger::info(std::string("twentyVector : "));
	i = 0;
	for (float f : resultTwentyVector_s) {
		resultString += prettyString(i, f, resultTwentyVector_c.at(i));
		Logger::info( std::string("no ")	+ (std::to_string(i)
			+ std::string("  sin: ") + std::to_string(f)
			+ std::string("  cos: ") + std::to_string(resultTwentyVector_c.at(i))));
		i++;
	}
	resultString += "fortyVector:\n";
	Logger::info(std::string("fortyVector : "));
	i = 0;
	for (float f : resultFortyVector_s) {
		resultString += prettyString(i, f, resultFortyVector_c.at(i));
		Logger::info( std::string("no ")	+ (std::to_string(i)
			+ std::string("  sin: ") + std::to_string(f)
			+ std::string("  cos: ") + std::to_string(resultFortyVector_c.at(i))));
		i++;
	}
	return resultString;
}

void runOldExample(){
	int baudRate = 4800;
	unsigned int bufferSize = 10;
	Logger::info(std::string("Running CV7 example:"));
	cout << "Running CV7 example: " << endl;

	CV7 sensor;

	try {
		sensor.loadConfig(PORTNAME, baudRate);
	} catch (const char* exception) {
		Logger::error(std::string( "crash at sensor.loadConfif : ")
			+ std::string( exception));
		cout << exception << endl;
	}
	sensor.setBufferSize(bufferSize); //Optional. Default: 30

	int polling = 50;
	float wd_a, ws_a, wt_a, wd_b, ws_b, wt_b;
	Logger::info(std::string("Entering test loop, looping ")
		+ std::to_string(polling)
		+ std::string("times"));
	while (polling--) {
		try {
			std::string t = sensor.refreshData();
			sensor.parseData(t);
		} catch (const char* exception) {
			Logger::error( std::string("crash at refreshData : ")
				+ std::string(exception));
			cout << exception << endl;
		}
		sensor.setUseMean(true);
		wd_a = sensor.getDirection();
		ws_a = sensor.getSpeed();
		wt_a = sensor.getTemperature();
		sensor.setUseMean(false);
		wd_b = sensor.getDirection();
		ws_b = sensor.getSpeed();
		wt_b = sensor.getTemperature();

		cout << "Mean Angle	: " << wd_a << " Speed: " << ws_a << " Temp: "
				<< wt_a << endl << "Median Angle	: " << wd_b << " Speed: "
				<< ws_b << " Temp: " << wt_b << endl;
		Logger::info( std::string("Mean Angle	 : ") + std::to_string( wd_a)
			+ std::string(" Speed: ") + std::to_string(ws_a)
			+ std::string(" Temp: " + std::to_string(wt_a)));
		Logger::info( std::string("Median Angle : ") + std::to_string(wd_b)
			+ std::string(" Speed: ") + std::to_string(ws_b)
			+ std::string(" Temp: ") + std::to_string(wt_b));
	}

}


int main(int argc, char** argv)
{
	if (!Logger::init("CV7Example.log")) {
		cout<<"error initing logger"<< endl;
		return 0;
	}

	/**
	 *  CV7 integration test
	 */
	Logger::info(std::string("reading info from windsensor with ") +
			std::to_string(2000));
	//		+ " freq");
	vector<map<string, double>> values = getHZReadings(2000, false);
	string log = sinAll(values);
	Logger::info(std::string("reading info from windsensor with ")
			+ std::to_string(1000)
			+ std::string(" freq"));
	values = getHZReadings(1000 , false);
	log = sinAll(values);
	Logger::info(std::string("reading info from windsensor with ")
			+ std::to_string(500)
			+ std::string(" freq"));
	values = getHZReadings(500 , false);
	log = sinAll(values);
	Logger::info(std::string("reading info from windsensor with ")
			+ std::to_string(250)
			+ std::string(" freq"));
	values = getHZReadings(250 , false);
	log = sinAll(values);

	runOldExample();
	
}



 
