#ifndef AUTO_SURVEY_H
#define AUTO_SURVEY_H

#include <stdint.h>

// Maximum number of waypoints in a survey mission
// 50 waypoints = ~400 bytes RAM — enough for most survey areas
#define SURVEY_MAX_WAYPOINTS 50

// Waypoint reached threshold in meters
// GPS CEP is ~2.5m, so 3m ensures reliable advancement
#define SURVEY_WP_RADIUS 3.0f

// Survey Mission State
typedef enum {
  SURVEY_IDLE,    // No mission loaded
  SURVEY_RUNNING, // Actively flying waypoints
  SURVEY_PAUSED,  // Mission paused, holding position
  SURVEY_DONE     // All waypoints completed
} Survey_State;

// Reset the survey — clears all waypoints
void Survey_Reset(void);

// Add a waypoint to the queue
// Returns 1 on success, 0 if queue is full
uint8_t Survey_AddWaypoint(float lat, float lon);

// Start the survey mission (loads first waypoint)
void Survey_Start(void);

// Pause the survey (drone should switch to LOITER)
void Survey_Pause(void);

// Resume a paused survey
void Survey_Resume(void);

// Abort the survey entirely
void Survey_Abort(void);

// Call every loop iteration when survey is running
// Updates wp_lat/wp_lon when current waypoint is reached
// Returns 1 if survey is actively navigating, 0 if idle/done
uint8_t Survey_Update(float current_lat, float current_lon,
                      float *wp_lat, float *wp_lon);

// Getters for telemetry
Survey_State Survey_GetState(void);
uint8_t Survey_GetCurrentIndex(void);
uint8_t Survey_GetTotalCount(void);

#endif /* AUTO_SURVEY_H */
