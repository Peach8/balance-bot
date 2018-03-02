#ifndef MB_CONTROLLER_H
#define MB_CONTROLLER_H

#include "mb_pid.h"
#include "mb_structs.h"
#include "mb_defs.h"
#include <math.h>
#define CFG_PATH "pid.cfg"

int mb_initialize_controller();
int mb_load_controller_config();
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, mb_odometry_t* mb_odometry);
int mb_destroy_controller();
float medianfilter(float value, int counter, float *valueholds, int n, int debugflag);

PID_t * pos_pid;
PID_t * linear_velo_pid;
PID_t * pitch_angle_pid;

PID_t * heading_pid;
PID_t * angular_velo_pid;

PID_t * left_pid;
PID_t * right_pid;

pid_parameters_t pos_pid_params;
pid_parameters_t linear_velo_pid_params;
pid_parameters_t pitch_angle_pid_params;

pid_parameters_t heading_pid_params;
pid_parameters_t angular_velo_pid_params;

pid_parameters_t left_pid_params;
pid_parameters_t right_pid_params;

#endif

