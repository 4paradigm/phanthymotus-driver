#ifndef FLIGHT_CTRL_H
#define FLIGHT_CTRL_H

#include <stdint.h>

int flight_ctrl_init(void);
int64_t flight_ctrl_takeoff(void);
int64_t flight_ctrl_land(void);
int64_t flight_ctrl_confirm_landing(void);
int64_t flight_ctrl_land_auto_confirm(void);
int64_t flight_ctrl_go_home(void);
int64_t flight_ctrl_cancel_go_home(void);
int64_t flight_ctrl_joystick_move(float vx, float vy, float vz, float vyaw);
int64_t flight_ctrl_emergency_brake(void);
int64_t flight_ctrl_obtain_authority(void);
int64_t flight_ctrl_release_authority(void);
int64_t flight_ctrl_turn_on_motors(void);
int64_t flight_ctrl_turn_off_motors(void);
int64_t flight_ctrl_slow_rotate_start(void);
int64_t flight_ctrl_slow_rotate_stop(void);
int64_t flight_ctrl_set_home(double lat, double lon);
int64_t flight_ctrl_set_obstacle_avoidance(int enabled, const char *direction);
void flight_ctrl_cleanup(void);

#endif
