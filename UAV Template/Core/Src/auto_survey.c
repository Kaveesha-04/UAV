#include "auto_survey.h"
#include "gps.h"

// Waypoint storage
static struct {
  float lat;
  float lon;
} survey_waypoints[SURVEY_MAX_WAYPOINTS];

static uint8_t survey_count = 0;       // Total waypoints loaded
static uint8_t survey_current = 0;     // Current waypoint index
static Survey_State survey_state = SURVEY_IDLE;

void Survey_Reset(void) {
  survey_count = 0;
  survey_current = 0;
  survey_state = SURVEY_IDLE;
}

uint8_t Survey_AddWaypoint(float lat, float lon) {
  if (survey_count >= SURVEY_MAX_WAYPOINTS) {
    return 0; // Queue full
  }
  survey_waypoints[survey_count].lat = lat;
  survey_waypoints[survey_count].lon = lon;
  survey_count++;
  return 1;
}

void Survey_Start(void) {
  if (survey_count == 0) return; // Nothing to fly
  survey_current = 0;
  survey_state = SURVEY_RUNNING;
}

void Survey_Pause(void) {
  if (survey_state == SURVEY_RUNNING) {
    survey_state = SURVEY_PAUSED;
  }
}

void Survey_Resume(void) {
  if (survey_state == SURVEY_PAUSED) {
    survey_state = SURVEY_RUNNING;
  }
}

void Survey_Abort(void) {
  survey_state = SURVEY_IDLE;
  // Don't reset waypoints — allow re-start if desired
}

uint8_t Survey_Update(float current_lat, float current_lon,
                      float *wp_lat_ptr, float *wp_lon_ptr) {
  if (survey_state != SURVEY_RUNNING || survey_count == 0) {
    return 0;
  }

  // Load current target waypoint
  *wp_lat_ptr = survey_waypoints[survey_current].lat;
  *wp_lon_ptr = survey_waypoints[survey_current].lon;

  // Check if we've reached the current waypoint
  float dist = GPS_Distance(current_lat, current_lon,
                             survey_waypoints[survey_current].lat,
                             survey_waypoints[survey_current].lon);

  if (dist <= SURVEY_WP_RADIUS) {
    // Waypoint reached! Advance to next
    survey_current++;

    if (survey_current >= survey_count) {
      // Mission complete!
      survey_state = SURVEY_DONE;
      return 0;
    }

    // Load next waypoint
    *wp_lat_ptr = survey_waypoints[survey_current].lat;
    *wp_lon_ptr = survey_waypoints[survey_current].lon;
  }

  return 1; // Still navigating
}

Survey_State Survey_GetState(void) {
  return survey_state;
}

uint8_t Survey_GetCurrentIndex(void) {
  return survey_current;
}

uint8_t Survey_GetTotalCount(void) {
  return survey_count;
}
