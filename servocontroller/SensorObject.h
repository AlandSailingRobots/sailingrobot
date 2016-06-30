#ifndef __SENSOROBJECT_H__
#define	__SENSOROBJECT_H__

#include <memory>

#include "MaestroController.h"

class SensorObject {
public:

    SensorObject();
	~SensorObject();

    // void loadConfig(cfg);
    void setController(Actuator * maestro);
	void setChannel(int channel);

    int getDirection();

private:
	int getPosition();
    int m_channel;
    std::shared_ptr<Actuator> m_maestro;

};

#endif
