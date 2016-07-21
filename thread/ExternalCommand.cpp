#include "ExternalCommand.h"


ExternalCommand::ExternalCommand(std::string timestamp, bool autorun, int rudderCommand, int sailCommand) {
  m_timestamp = timestamp;
  m_autorun = autorun;
  m_rudderCommand = rudderCommand;
  m_sailCommand = sailCommand;
}

bool ExternalCommand::setData(std::string timestamp, bool autorun, int rudderCommand, int sailCommand) {
  // critical section (exclusive access by locking mtx):
  bool isNewData = false;
  std::lock_guard<std::mutex> lock(mtx);
  // if(m_timestamp < timestamp) {
    m_timestamp = timestamp;
    m_autorun = autorun;
    m_rudderCommand = rudderCommand;
    m_sailCommand = sailCommand;
    isNewData = true;
  // }

  return isNewData;
}

bool ExternalCommand::getAutorun() {
  // critical section (exclusive access by locking mtx):
  std::lock_guard<std::mutex> lock(mtx);
  bool get = m_autorun;

  return get;
}

int ExternalCommand::getRudderCommand() {
  // critical section (exclusive access by locking mtx):
  std::lock_guard<std::mutex> lock(mtx);
  int get = m_rudderCommand;

  return get;
}

int ExternalCommand::getSailCommand() {
  // critical section (exclusive access by locking mtx):
  std::lock_guard<std::mutex> lock(mtx);
  int get = m_sailCommand;

  return get;
}