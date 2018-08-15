/****************************************************************************************
 *
 * File:
 *      SimulationNode.cpp
 *
 * Purpose:
 *      Discuss with the simulator via TCP.
 *      Create sensor messages from simulation data and publish them on the message bus.
 *      Listen to actuators command messages and send the command datas to the simulator.
 *
 * Developer Notes:
 *
 *
 ***************************************************************************************/

#include "SimulationNode.h"

#define SERVER_PORT 6900

SimulationNode::SimulationNode(MessageBus& msgBus, bool boatType)
    : ActiveNode(NodeID::Simulator, msgBus),
      m_RudderCommand(0),
      m_SailCommand(0),
      m_TailCommand(0),
      m_CompassHeading(0),
      m_GPSLat(0),
      m_GPSLon(0),
      m_GPSSpeed(0),
      m_GPSCourse(0),
      m_WindDir(0),
      m_WindSpeed(0),
      m_nextDeclination(0),
      collidableMgr(NULL),
      m_boatType(boatType) {
    msgBus.registerNode(*this, MessageType::SailCommand);
    msgBus.registerNode(*this, MessageType::WingSailCommand);
    msgBus.registerNode(*this, MessageType::RudderCommand);
    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::MarineSensorData);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
}

SimulationNode::SimulationNode(MessageBus& msgBus, bool boatType, CollidableMgr* collidableMgr)
    : ActiveNode(NodeID::Simulator, msgBus),
      m_RudderCommand(0),
      m_SailCommand(0),
      m_TailCommand(0),
      m_CompassHeading(0),
      m_GPSLat(0),
      m_GPSLon(0),
      m_GPSSpeed(0),
      m_GPSCourse(0),
      m_WindDir(0),
      m_WindSpeed(0),
      m_nextDeclination(0),
      collidableMgr(collidableMgr),
      m_boatType(boatType) {
    msgBus.registerNode(*this, MessageType::SailCommand);
    msgBus.registerNode(*this, MessageType::WingSailCommand);
    msgBus.registerNode(*this, MessageType::RudderCommand);
    msgBus.registerNode(*this, MessageType::WaypointData);
    msgBus.registerNode(*this, MessageType::MarineSensorData);
    msgBus.registerNode(*this, MessageType::ServerConfigsReceived);
}

void SimulationNode::start() {
    runThread(SimulationThreadFunc);
}

bool SimulationNode::init() {
    bool success = false;
    updateConfigsFromDB();

    int rc = server.start(SERVER_PORT);

    if (rc > 0) {
        Logger::info("Waiting for simulation client...\n");

        // Block until connection, don't timeout
        server.acceptConnection(0);

        success = true;
    } else {
        Logger::error("Failed to start the simulator server");
        success = false;
    }

    return success;
}

void SimulationNode::processMessage(const Message* msg) {
    MessageType type = msg->messageType();

    switch (type) {
        case MessageType::SailCommand:
            processSailCommandMessage((SailCommandMsg*)msg);
            break;
        case MessageType::WingSailCommand:
            processWingSailCommandMessage((WingSailCommandMsg*)msg);
            break;
        case MessageType::RudderCommand:
            processRudderCommandMessage((RudderCommandMsg*)msg);
            break;
        case MessageType::WaypointData:
            processWaypointMessage((WaypointDataMsg*)msg);
            createMarineSensorMessage((WaypointDataMsg*)msg);
            break;
        case MessageType::ServerConfigsReceived:
            updateConfigsFromDB();
            break;
        default:
            return;
    }
}

void SimulationNode::processSailCommandMessage(SailCommandMsg* msg) {
    m_SailCommand = msg->maxSailAngle();
}

void SimulationNode::processWingSailCommandMessage(WingSailCommandMsg* msg) {
    m_TailCommand = msg->tailAngle();
}

void SimulationNode::processRudderCommandMessage(RudderCommandMsg* msg) {
    m_RudderCommand = msg->rudderAngle();
}

void SimulationNode::processWaypointMessage(WaypointDataMsg* msg) {
    waypoint.nextId = msg->nextId();
    waypoint.nextLongitude = msg->nextLongitude();
    waypoint.nextLatitude = msg->nextLatitude();
    waypoint.nextDeclination = msg->nextDeclination();
    waypoint.nextRadius = msg->nextRadius();
    waypoint.nextStayTime = msg->stayTime();
    // waypoint.isCheckpoint = msg->isCheckpoint();
    waypoint.prevId = msg->prevId();
    waypoint.prevLongitude = msg->prevLongitude();
    waypoint.prevLatitude = msg->prevLatitude();
    waypoint.prevDeclination = msg->prevDeclination();
    waypoint.prevRadius = msg->prevRadius();

    m_nextDeclination = msg->nextDeclination();
}

void SimulationNode::processMarineSensorDataMessage(MarineSensorDataMsg* msg) {
    marineSensorData.temperature = msg->temperature();
    marineSensorData.conductivity = msg->conductivity();
    marineSensorData.ph = msg->ph();
    marineSensorData.salinity = msg->salinity();
}

void SimulationNode::createCompassMessage() {
    MessagePtr msg = std::make_unique<CompassDataMsg>(CompassDataMsg(m_CompassHeading, 0, 0));
    m_MsgBus.sendMessage(std::move(msg));
}

void SimulationNode::createGPSMessage() {
    MessagePtr msg = std::make_unique<GPSDataMsg>(GPSDataMsg(true, true, m_GPSLat, m_GPSLon,
                                                             SysClock::unixTime(), m_GPSSpeed,
                                                             m_GPSCourse, 0, GPSMode::LatLonOk));
    m_MsgBus.sendMessage(std::move(msg));
}

void SimulationNode::createWindMessage() {
    MessagePtr windData = std::make_unique<WindDataMsg>(WindDataMsg(m_WindDir, m_WindSpeed, 21));
    m_MsgBus.sendMessage(std::move(windData));
}

void SimulationNode::createMarineSensorMessage(WaypointDataMsg* msg) {
    int id = msg->nextId();

    // float idAsDecimal = id / pow(10, (floor(log10(id)) + 1));
    float IdlastDigitsAsDecimal = id / 10000.0;
    IdlastDigitsAsDecimal -= (int)IdlastDigitsAsDecimal;

    double temperature = 20 + IdlastDigitsAsDecimal;
    float conductivity = 1 + IdlastDigitsAsDecimal;
    float ph = 7 + IdlastDigitsAsDecimal;

    float salinity = Utility::calculateSalinity(temperature, conductivity);

    MessagePtr marineSensorData = std::make_unique<MarineSensorDataMsg>(
        MarineSensorDataMsg(temperature, conductivity, ph, salinity));

    m_MsgBus.sendMessage(std::move(marineSensorData));
}

///--------------------------------------------------------------------------------------
void SimulationNode::processSailBoatData(TCPPacket_t& packet) {
    if (packet.length - 1 == sizeof(SailBoatDataPacket_t)) {
        // The first byte is the packet type, lets skip that
        uint8_t* ptr = packet.data + 1;
        SailBoatDataPacket_t* boatData = (SailBoatDataPacket_t*)ptr;

        m_CompassHeading = Utility::limitAngleRange(90 - boatData->heading -
                                                    m_nextDeclination);  // [0, 360] north east down
        m_GPSLat = boatData->latitude;
        m_GPSLon = boatData->longitude;

        m_GPSSpeed = std::abs(boatData->speed);  // norm of the speed vector
        if (boatData->speed >= 0) {
            m_GPSCourse =
                Utility::limitAngleRange(90 - boatData->course);  // [0, 360] north east down
        } else {
            m_GPSCourse =
                Utility::limitAngleRange(90 - boatData->course + 180);  // [0, 360] north east down
        }

        m_WindDir = Utility::limitAngleRange(
            180 - boatData->windDir);  // [0, 360] clockwize, where the wind come from
        m_WindSpeed = boatData->windSpeed;

        // Send messages
        createCompassMessage();
        createGPSMessage();
        createWindMessage();
    }
}

///--------------------------------------------------------------------------------------
void SimulationNode::processWingBoatData(TCPPacket_t& packet) {
    if (packet.length - 1 == sizeof(WingBoatDataPacket_t)) {
        // The first byte is the packet type, lets skip that
        uint8_t* ptr = packet.data + 1;
        WingBoatDataPacket_t* boatData = (WingBoatDataPacket_t*)ptr;

        m_CompassHeading = Utility::limitAngleRange(90 - boatData->heading -
                                                    m_nextDeclination);  // [0, 360] north east down
        m_GPSLat = boatData->latitude;
        m_GPSLon = boatData->longitude;

        m_GPSSpeed = std::abs(boatData->speed);  // norm of the speed vector
        if (boatData->speed >= 0) {
            m_GPSCourse =
                Utility::limitAngleRange(90 - boatData->course);  // [0, 360] north east down
        } else {
            m_GPSCourse =
                Utility::limitAngleRange(90 - boatData->course + 180);  // [0, 360] north east down
        }

        m_WindDir = Utility::limitAngleRange(
            180 - boatData->windDir);  // [0, 360] clockwize, where the wind come from
        m_WindSpeed = boatData->windSpeed;

        // Send messages
        createCompassMessage();
        createGPSMessage();
        createWindMessage();
    }
}

///--------------------------------------------------------------------------------------
void SimulationNode::processAISContact(TCPPacket_t& packet) {
    if (this->collidableMgr != NULL) {
        // The first byte is the packet type, lets skip that
        uint8_t* ptr = packet.data + 1;
        AISContactPacket_t* aisData = (AISContactPacket_t*)ptr;

        this->collidableMgr->addAISContact(
            aisData->mmsi, aisData->latitude, aisData->longitude, aisData->speed,
            Utility::limitAngleRange(90 - aisData->course) /* [0, 360] north east down*/);
        this->collidableMgr->addAISContact(aisData->mmsi, aisData->length, aisData->beam);
    }
}

///--------------------------------------------------------------------------------------
void SimulationNode::processVisualField(TCPPacket_t& packet) {
    if (this->collidableMgr != NULL) {
        // The first byte is the packet type, lets skip that
        uint8_t* ptr = packet.data + 1;
        VisualFieldPacket_t* data = reinterpret_cast<VisualFieldPacket_t*>(ptr);
        std::map<int16_t, uint16_t> bearingToRelativeObstacleDistance;
        for (int i = 0; i < 24; ++i) {
            bearingToRelativeObstacleDistance[12 - i] = data->relativeObstacleDistances[i];
        }

        Logger::debug("retrieving heading: %d", Utility::wrapAngle(90 - data->heading));

        this->collidableMgr->addVisualField(bearingToRelativeObstacleDistance,
                                            Utility::wrapAngle(90 - data->heading));
    }
}

///--------------------------------------------------------------------------------------
void SimulationNode::sendActuatorDataWing(int socketFD) {
    actuatorDataWing.rudderCommand = -Utility::degreeToRadian(m_RudderCommand);
    actuatorDataWing.tailCommand = -Utility::degreeToRadian(m_TailCommand);

    server.sendData(socketFD, &actuatorDataWing, sizeof(ActuatorDataWingPacket_t));
}

void SimulationNode::sendActuatorDataSail(int socketFD) {
    actuatorDataSail.rudderCommand = -Utility::degreeToRadian(m_RudderCommand);
    actuatorDataSail.sailCommand = Utility::degreeToRadian(m_SailCommand);

    server.sendData(socketFD, &actuatorDataSail, sizeof(ActuatorDataSailPacket_t));
}

///--------------------------------------------------------------------------------------
void SimulationNode::sendWaypoint(int socketFD) {
    server.sendData(socketFD, &waypoint, sizeof(WaypointPacket_t));
}

///--------------------------------------------------------------------------------------
void SimulationNode::sendMarineSensor(int socketFD) {
    server.sendData(socketFD, &marineSensorData, sizeof(MarineSensorDataPacket_t));
}

///--------------------------------------------------------------------------------------
void SimulationNode::SimulationThreadFunc(ActiveNode* nodePtr) {
    SimulationNode* node = dynamic_cast<SimulationNode*>(nodePtr);

    TCPPacket_t packet;
    int simulatorFD = 0;

    while (true) {
        // Don't timeout on a packet read
        node->server.readPacket(packet, 0);

        // We only care about the latest packet, so clear out the old ones
        // node->server.clearSocketBuffer( packet.socketFD );

        // We can safely assume that the first packet we receive will actually be from
        // the simulator as we only should ever accept one connection, the first one/
        if (simulatorFD == 0) {
            simulatorFD = packet.socketFD;
        }
        // First byte is the message type
        switch (packet.data[0]) {
            case SimulatorPacket::SailBoatData:
                node->processSailBoatData(packet);
                break;

            case SimulatorPacket::WingBoatData:
                node->processWingBoatData(packet);
                break;
            case SimulatorPacket::AISData:
                node->processAISContact(packet);
                break;

            case SimulatorPacket::CameraData:
                node->processVisualField(packet);
                break;

            case SimulatorPacket::MarineSensorData:
                node->processSailBoatData(packet);
                break;

            // unknown or deformed packet
            default:

                continue;
        }
        // Reset our packet, better safe than sorry
        packet.socketFD = 0;
        packet.length = 0;

        if (node->m_boatType == 0) {
            node->sendActuatorDataSail(simulatorFD);
        } else if (node->m_boatType == 1) {
            node->sendActuatorDataWing(simulatorFD);
        }

        node->sendWaypoint(simulatorFD);
        // node->sendMarineSensor( simulatorFD );
    }
}
