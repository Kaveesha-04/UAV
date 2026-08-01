#ifndef GPS_H
#define GPS_H

#include "main.h"
#include <stdint.h>

// නියම Location දත්ත ගබඩා කරගැනීමට
extern float gps_lat;
extern float gps_lon;
extern uint8_t gps_fix;

void GPS_Parse(char *nmea);
float GPS_Distance(float lat1, float lon1, float lat2, float lon2);
float GPS_Bearing(float lat1, float lon1, float lat2, float lon2);

#endif /* GPS_H */
