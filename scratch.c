#include <stdio.h>
#include <math.h>
#include <stdint.h>

int main() {
    float battery_voltage = 12.4f;
    float roll_actual = -15.3f;
    float pitch_actual = 20.1f;
    float yaw_actual = 90.5f;
    float alt = 1.5f;
    float heading = 180.0f;
    float gps_lat = 6.9271f;
    float gps_lon = 79.8612f;
    uint8_t gps_fix = 1;
    float base_throttle = 1500.0f;
    uint8_t mag_type = 13;
    
    struct { float Kp, Ki, Kd, Kf; } pid_roll = {0.6f, 0.0f, 0.01f, 0.0f};
    struct { float Kp, Ki, Kd, Kf; } pid_pitch = {0.6f, 0.0f, 0.01f, 0.0f};
    struct { float Kp, Ki, Kd, Kf; } pid_yaw = {1.0f, 0.0f, 0.0f, 0.0f};
    
    int current_mode = 0;
    int16_t mag_x = -100, mag_y = 200, mag_z = 300;
    int16_t nrf_yaw = 500, nrf_pitch = 0, nrf_roll = -500;
    int STATE_ARMED = 1;
    int current_state = 1;

    const char *s_r = (roll_actual < 0) ? "-" : "";
    const char *s_p = (pitch_actual < 0) ? "-" : "";
    const char *s_y = (yaw_actual < 0) ? "-" : "";
    const char *s_a = (alt < 0) ? "-" : "";
    const char *s_d = (heading < 0) ? "-" : "";
    const char *s_glat = (gps_lat < 0) ? "-" : "";
    const char *s_glon = (gps_lon < 0) ? "-" : "";

    char uart_buf[512];
    sprintf(
        uart_buf,
        "{\"v\":%d.%02d,\"r\":%s%d.%d,\"p\":%s%d.%d,\"y\":%s%d.%d,\"a\":%s%"
        "d.%d,\"d\":%s%d.%d,\"glat\":%s%d.%05d,\"glon\":%s%d.%05d,\"gf\":%"
        "d,\"t\":%d,\"mt\":%d,"
        "\"pid_r\":[%d.%02d,%d.%02d,%d.%02d,%d.%02d],\"pid_p\":[%d.%02d,%d.%02d,%d."
        "%02d,%d.%02d],\"pid_y\":[%d.%02d,%d.%02d,%d.%02d,%d.%02d],\"md\":%d,\"mx\":%d,"
        "\"my\":%d,\"mz\":%d,\"ry\":%d,\"rp\":%d,\"rr\":%d,\"arm\":%d}\n",
        (int)battery_voltage, (int)(battery_voltage * 100) % 100, s_r,
        (int)fabsf(roll_actual), (int)(fabsf(roll_actual) * 10) % 10, s_p,
        (int)fabsf(pitch_actual), (int)(fabsf(pitch_actual) * 10) % 10, s_y,
        (int)fabsf(yaw_actual), (int)(fabsf(yaw_actual) * 10) % 10, s_a,
        (int)fabsf(alt), (int)(fabsf(alt) * 10) % 10, s_d,
        (int)fabsf(heading), (int)(fabsf(heading) * 10) % 10, s_glat,
        (int)fabsf(gps_lat), (int)(fabsf(gps_lat) * 100000) % 100000,
        s_glon, (int)fabsf(gps_lon),
        (int)(fabsf(gps_lon) * 100000) % 100000, gps_fix,
        (int)base_throttle, mag_type, (int)pid_roll.Kp,
        (int)(pid_roll.Kp * 100) % 100, (int)pid_roll.Ki,
        (int)(pid_roll.Ki * 100) % 100, (int)pid_roll.Kd,
        (int)(pid_roll.Kd * 100) % 100, (int)pid_roll.Kf,
        (int)(pid_roll.Kf * 100) % 100, (int)pid_pitch.Kp,
        (int)(pid_pitch.Kp * 100) % 100, (int)pid_pitch.Ki,
        (int)(pid_pitch.Ki * 100) % 100, (int)pid_pitch.Kd,
        (int)(pid_pitch.Kd * 100) % 100, (int)pid_pitch.Kf,
        (int)(pid_pitch.Kf * 100) % 100, (int)pid_yaw.Kp,
        (int)(pid_yaw.Kp * 100) % 100, (int)pid_yaw.Ki,
        (int)(pid_yaw.Ki * 100) % 100, (int)pid_yaw.Kd,
        (int)(pid_yaw.Kd * 100) % 100, (int)pid_yaw.Kf,
        (int)(pid_yaw.Kf * 100) % 100, (int)current_mode, mag_x, mag_y,
        mag_z, nrf_yaw, nrf_pitch, nrf_roll,
        (current_state == STATE_ARMED ? 1 : 0));
    
    printf("%s", uart_buf);
    return 0;
}
