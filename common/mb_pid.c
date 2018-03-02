/*******************************************************************************
* mb_pid.c
*
* TODO: implement these functions to build a generic PID controller
*       with a derivative term low pass filter
*
*******************************************************************************/

#include "mb_pid.h"
#include "mb_defs.h"
#define ITERM_MIN -1.0
#define ITERM_MAX 1.0


PID_t * PID_Init(float Kp, float Ki, float Kd, float dFilterHz, float updateHz) {
	PID_t *pid =  malloc(sizeof(PID_t));
	pid->pidInput = 0;
	pid->pidOutput = 0;
	pid->pTerm = 0;
	pid->iTerm = 0;
	pid->dTerm = 0;
	pid->prevInput = 0;
	pid->iTermMin = 0;
	pid->iTermMax = 0;
	pid->outputMin = 0;
	pid->outputMax = 0;

	PID_SetTunings(pid, Kp, Ki, Kd);
	PID_SetDerivativeFilter(pid, dFilterHz);
	PID_SetUpdateRate(pid, updateHz);

	return pid;
}

float PID_Compute(PID_t* pid, float error, int flag) {

	pid->pTerm = pid->kp*error;
	
	pid->dTerm = pid->kd * (error - pid->prevInput) * pid->updateHz;
	pid->prevInput = error;
	if (flag) {
		pid->dTerm = rc_march_filter(&pid->dFilter, pid->dTerm);
	}

	pid->iTerm += pid->ki * error / pid->updateHz;
	if (pid->iTerm < pid->iTermMin){
		pid->iTerm = pid->iTermMin;
	}
    else if (pid->iTerm > pid->iTermMax){
    	pid->iTerm = pid->iTermMax;
    }

    float output = pid->pTerm + pid->dTerm + pid->iTerm;
    if (output < pid->outputMin) {
    	output = pid->outputMin;
    }
    else if (output > pid->outputMax) {
    	output = pid->outputMax;
    }
    return output;
}

void PID_SetTunings(PID_t* pid, float Kp, float Ki, float Kd) {
	// scale gains by update rate in seconds for proper units
	pid->kp = Kp;
	pid->ki = Ki;
	pid->kd = Kd;
}

void PID_SetOutputLimits(PID_t* pid, float min, float max){
	pid->outputMin = min;
	pid->outputMax = max;
}

void PID_SetIntegralLimits(PID_t* pid, float min, float max){
	if (min < ITERM_MIN) {
		pid->iTermMin = ITERM_MIN;
	}
	else {
		pid->iTermMin = min;
	}
	if (max > ITERM_MAX) {
		pid->iTermMax = ITERM_MAX;
	}
	else {
		pid->iTermMax = max;	
	}	
}

void PID_ResetIntegrator(PID_t* pid){
	pid->ki = 0;
	pid->iTerm = 0;
}

void PID_SetDerivativeFilter(PID_t* pid, float dFilterHz){
	pid->dFilterHz = dFilterHz;
	pid->dFilter = rc_empty_filter();
	rc_butterworth_lowpass(&pid->dFilter, 3, DT, dFilterHz);
}

void PID_SetUpdateRate(PID_t* pid, float updateHz){
	pid->updateHz = updateHz;
}
