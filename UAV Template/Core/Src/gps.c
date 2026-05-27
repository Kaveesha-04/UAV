#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

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

// Convert a single hex character to its integer value
static int parse_hex_char(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Validate the NMEA XOR checksum
static int validate_nmea_checksum(const char *nmea) {
    if (nmea[0] != '$') return 0;
    
    int checksum = 0;
    int i = 1;
    // XOR all characters between '$' and '*'
    while (nmea[i] != '*' && nmea[i] != '\0') {
        checksum ^= nmea[i];
        i++;
    }
    
    // Check if '*' exists and has two characters following it
    if (nmea[i] == '*' && nmea[i+1] != '\0' && nmea[i+2] != '\0') {
        int high = parse_hex_char(nmea[i+1]);
        int low = parse_hex_char(nmea[i+2]);
        if (high != -1 && low != -1) {
            return (checksum == ((high << 4) | low));
        }
    }
    return 0; // Checksum invalid or missing
}

void GPS_Parse(char *nmea) {
    // Basic $GPGGA and $GNGGA parser
    if (strncmp(nmea, "$GPGGA", 6) == 0 || strncmp(nmea, "$GNGGA", 6) == 0) {
        // 1. MUST Validate Checksum first to avoid parsing phantom coordinates from noisy UART
        if (!validate_nmea_checksum(nmea)) {
            return;
        }

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
        
        // 2. MUST Ensure we actually received coordinate data (prevents overwriting with 0.0 on lost fix)
        if (gps_fix > 0 && raw_lat != 0.0f && raw_lon != 0.0f) {
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
    
    float a = sinf(dLat/2.0f) * sinf(dLat/2.0f) +
              sinf(dLon/2.0f) * sinf(dLon/2.0f) * cosf(lat1) * cosf(lat2);
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f-a));
    
    return EARTH_RADIUS * c;
}

// Calculate initial bearing from point 1 to point 2 in degrees (0-360)
float GPS_Bearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = (lon2 - lon1) * DEG_TO_RAD;
    
    lat1 = lat1 * DEG_TO_RAD;
    lat2 = lat2 * DEG_TO_RAD;
    
    float y = sinf(dLon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dLon);
    
    float bearing = atan2f(y, x) * (180.0f / M_PI);
    
    if (bearing < 0) {
        bearing += 360.0f;
    }
    return bearing;
}
