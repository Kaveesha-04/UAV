#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define EARTH_RADIUS 6371000.0f // meters
#define DEG_TO_RAD 0.01745329251f

float gps_lat = 0.0f;
float gps_lon = 0.0f;
uint8_t gps_fix = 0;

// Convert NMEA DDMM.MMMM to Decimal Degrees
static float NMEA_to_Decimal(float nmea_coord, char direction) {
    int degrees = (int)(nmea_coord / 100);
    float minutes = nmea_coord - (degrees * 100);
    float decimal = degrees + (minutes / 60.0f);
    
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

void GPS_Parse(char *nmea) {
    // Basic $GPGGA parser
    if (strncmp(nmea, "$GPGGA", 6) == 0) {
        int comma_count = 0;
        char *p = nmea;
        
        float raw_lat = 0.0f;
        char lat_dir = 'N';
        float raw_lon = 0.0f;
        char lon_dir = 'E';
        
        while (*p) {
            if (*p == ',') {
                comma_count++;
                p++; // Move to start of field
                
                if (comma_count == 2 && *p != ',') raw_lat = atof(p);
                else if (comma_count == 3 && *p != ',') lat_dir = *p;
                else if (comma_count == 4 && *p != ',') raw_lon = atof(p);
                else if (comma_count == 5 && *p != ',') lon_dir = *p;
                else if (comma_count == 6 && *p != ',') gps_fix = atoi(p);
                
                continue;
            }
            p++;
        }
        
        if (gps_fix > 0) {
            gps_lat = NMEA_to_Decimal(raw_lat, lat_dir);
            gps_lon = NMEA_to_Decimal(raw_lon, lon_dir);
        }
    }
}

// Calculate distance between two coordinates in meters (Haversine formula)
float GPS_Distance(float lat1, float lon1, float lat2, float lon2) {
    float dLat = (lat2 - lat1) * DEG_TO_RAD;
    float dLon = (lon2 - lon1) * DEG_TO_RAD;
    
    lat1 = lat1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    
    float a = sin(dLat/2) * sin(dLat/2) +
              sin(dLon/2) * sin(dLon/2) * cos(lat1) * cos(lat2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return EARTH_RADIUS * c;
}

// Calculate initial bearing from point 1 to point 2 in degrees (0-360)
float GPS_Bearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = (lon2 - lon1) * DEG_TO_RAD;
    
    lat1 = lat1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    
    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    
    float bearing = atan2(y, x) * (180.0f / M_PI);
    
    if (bearing < 0) {
        bearing += 360.0f;
    }
    return bearing;
}
