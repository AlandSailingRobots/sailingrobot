#include "I2CController.h"


I2CController::I2CController(SystemState *systemState, bool mockArduino, bool mockCompass, int headingBufferSize, double loopTime) {
    m_systemState = systemState;
    m_mockArduino = mockArduino;
    m_mockCompass = mockCompass;
    m_headingBufferSize = headingBufferSize;
    m_loopTime = loopTime;
}

I2CController::~I2CController() {

}

void I2CController::init() {
    m_running = true;
    initCompass(m_mockCompass, m_headingBufferSize);
    initArduino(m_mockArduino);
}

void I2CController::run() {
    std::cout << "-I2CController thread started." << std::endl;
    m_logger.info("-I2CController thread started.");
    while(isRunning()) {
        m_timer.reset();
        //update system state
        m_systemState->setCompassModel(CompassModel(
        m_compass->getHeading(),
        m_compass->getPitch(),
        m_compass->getRoll(),
        AccelerationModel(m_compass->getAccelX(),
                          m_compass->getAccelY(),
                          m_compass->getAccelZ())
        ));
        m_arduino->readValues();
        m_systemState->setAnalogArduinoModel(m_arduino->getModel());

        m_timer.sleepUntil(m_loopTime);
    }
}

void I2CController::initCompass(bool mockCompass, int headningBufferSize) {
    if (!mockCompass) {
        m_compass.reset(new HMC6343(headningBufferSize) );
    } else {
        m_compass.reset(new MockCompass());
    }

    try {
        m_compass->init();
    } catch (const char * error) {
        std::cout << "-I2CController::setupCompass()\tFailed" << std::endl;
        m_logger.error("-I2CController::setupCompass()\tFailed");
    }
    std::cout << "-I2CController::setupCompass()\tOK" << std::endl;
    m_logger.info("-I2CController::setupCompass()\tOK");
}

void I2CController::initArduino(bool mockArduino) {
    if(!mockArduino) {
        m_arduino.reset(new AR_UNO());
    } else {
        m_arduino.reset(new MockAnalogArduino());
    }

    try {
        m_arduino->init();
    } catch (const char * error) {
        std::cout << "-I2CController::initArduino()\tFailed" << std::endl;
        m_logger.error("I2CController::initArduino()\tFailed");
    }
    std::cout << "-I2CController::initArduino()\tOK" << std::endl;
    m_logger.info("-I2CController::initArduino()\tOK");
}

bool I2CController::isRunning() {
    bool running;
    m_mutex.lock();
    running = m_running;
    m_mutex.unlock();
    return running;
}

void I2CController::close() {
    m_mutex.lock();
    m_running = false;
    m_mutex.unlock();
}
