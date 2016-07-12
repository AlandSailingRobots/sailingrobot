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

    // NOTE - Jordan - I think the I2C device code should interact via the I2C controller that way we can make it thread safe.

    initCompass(m_mockCompass, m_headingBufferSize);
    initArduino(m_mockArduino);
}

void I2CController::run() {
    Logger::info("I2CController thread started");
    while(isRunning()) {
        m_timer.reset();

        m_compass->readValues();
        m_compass->readAccel();

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

bool I2CController::initCompass(bool mockCompass, int headningBufferSize) {
    if (!mockCompass) {
        m_compass.reset(new HMC6343(headningBufferSize) );
    } else {
        m_compass.reset(new MockCompass());
    }

    if(not m_compass->init())
    {
        Logger::error("%s Failed to initialise the compass", __PRETTY_FUNCTION__);
        return false;
    }

    Logger::info("Compass initialised");
    return true;
}

bool I2CController::initArduino(bool mockArduino) {
    if(!mockArduino) {
        m_arduino.reset(new AR_UNO());
    } else {
        m_arduino.reset(new MockAnalogArduino());
    }

    if(not m_arduino->init())
    {
        Logger::error("%s Failed to communicate with the arduino", __PRETTY_FUNCTION__);
        return false;
    }

    Logger::info("Arduino initialised");
    return true;
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
