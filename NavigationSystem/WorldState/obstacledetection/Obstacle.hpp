#ifndef OSBTACLECLASS_HPP
#define OSBTACLECLASS_HPP

#include <chrono>

static unsigned long int counter = 0;

// Class to store the detected obstacles

class Obstacle
{
public:
    Obstacle(float x, float y, float diameter, float heading, int frame_n) : m_id(++counter), m_x(x), m_y(y), m_radius(diameter/2), m_heading(heading), m_last_seen(frame_n) {
    };
    
    bool compare(float x, float y)
    {
        if ((x - m_x)*(x - m_x) + (y - m_y)*(y - m_y) <= m_radius*m_radius)
            return true;
            
        return false;
    };
    
    float getHeading()
    {
        return m_heading;
    };
    
    unsigned long int getId()
    {
        return m_id;
    }
    
    int getLastSeen()
    {
        return m_last_seen;
    }
    
private:
    unsigned long int m_id; // the obstacle ID, for tracking purposes
    float m_x; // X position in the frame
    float m_y; // Y position in the frame
    float m_radius; // Radius of the circonference circumscribing the obstacle
    float m_heading; // Estimated obstacle heading using the boat reference system
    int m_last_seen; // Store the last frame n. in which the obstacle appeared
};

#endif // OSBTACLECLASS_HPP
// 
