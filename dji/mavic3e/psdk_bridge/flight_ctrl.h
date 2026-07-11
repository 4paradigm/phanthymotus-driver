#ifndef FLIGHT_CTRL_H
#define FLIGHT_CTRL_H

int flight_ctrl_init(void);
int flight_ctrl_takeoff(void);
int flight_ctrl_land(void);
int flight_ctrl_go_home(void);
int flight_ctrl_cancel_go_home(void);
int flight_ctrl_joystick_move(float vx, float vy, float vz, float vyaw);
int flight_ctrl_emergency_brake(void);
int flight_ctrl_obtain_authority(void);
int flight_ctrl_release_authority(void);
int flight_ctrl_set_home(double lat, double lon);
int flight_ctrl_set_obstacle_avoidance(int enabled, const char *direction);
void flight_ctrl_cleanup(void);

#endif
