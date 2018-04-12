
#ifndef CANBUS_ERROR_DEFS_H
#define CANBUS_ERROR_DEFS_H

#define NO_ERRORS 0

#define ERROR_SENSOR_PH_SYNTAX 1                    // Syntax error. Happens if we send wrong command to sensor
#define ERROR_SENSOR_PH_NOT_READY 2                 // Not ready. Means we did not wait long enough for sensor to actually get a reading
#define ERROR_SENSOR_PH_NO_DATA 3                   // No data from sensor. Probably sensor failure? Check specifications from sensor documentation

#define ERROR_SENSOR_CONDUCTIVETY_SYNTAX 4          // Syntax error. Happens if we send wrong command to sensor
#define ERROR_SENSOR_CONDUCTIVETY_NOT_READY 5       // Not ready. Means we did not wait long enough for sensor to actually get a reading
#define ERROR_SENSOR_CONDUCTIVETY_NO_DATA 6         // No data from sensor. Probably sensor failure? Check specifications from sensor documentation

#define ERROR_SENSOR_TEMPERATURE_SYNTAX 7           // Syntax error. Happens if we send wrong command to sensor
#define ERROR_SENSOR_TEMPERATURE_NOT_READY 8        // Not ready. Means we did not wait long enough for sensor to actually get a reading
#define ERROR_SENSOR_TEMPERATURE_NO_DATA 9          // No data from sensor. Probably sensor failure? Check specifications from sensor documentation

#endif
