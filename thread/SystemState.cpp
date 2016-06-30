#include "SystemState.h"

void SystemState::setData(SystemStateModel model){
	std::lock_guard<std::mutex> lock(m_mutex);
	m_model = model;
}

void SystemState::getData(SystemStateModel& model){
	std::lock_guard<std::mutex> lock(m_mutex);
	model = m_model;
}

void SystemState::setGPSModel(GPSModel gpsModel)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_model.gpsModel = gpsModel;
}

void SystemState::setWindsensorModel(WindsensorModel windsensorModel){
	std::lock_guard<std::mutex> lock(m_mutex);
	m_model.windsensorModel = windsensorModel;
}

void SystemState::setCompassModel(CompassModel compassModel){
	std::lock_guard<std::mutex> lock(m_mutex);
	m_model.compassModel = compassModel;
}

void SystemState::setAnalogArduinoModel(AnalogArduinoModel arduinoModel) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_model.arduinoModel = arduinoModel;
}

void SystemState::setRudder(int value){
	std::lock_guard<std::mutex> lock(m_mutex);
	m_model.rudder = value;
}

void SystemState::setSail(int value){
	std::lock_guard<std::mutex> lock(m_mutex);
	m_model.sail = value;
}
