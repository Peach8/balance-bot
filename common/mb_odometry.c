/*******************************************************************************
* mb_odometry.c
*
* TODO: Implement these functions to add odometry and dead rekoning 
*
*******************************************************************************/

#include "../balancebot/balancebot.h"

#define DTHETA_THRESH 0.001

void mb_initialize_odometry(mb_odometry_t* mb_odometry, float x, float y, float theta){
	mb_odometry->x = x;
	mb_odometry->y = y;
	mb_odometry->theta = theta;
}

void mb_update_odometry(mb_odometry_t* mb_odometry, mb_state_t* mb_state){
	// dead reckoning, no IMU
	mb_odometry->x += mb_odometry->delta_pos * cos(mb_odometry->theta);
	mb_odometry->y += mb_odometry->delta_pos * sin(mb_odometry->theta);
	mb_odometry->theta = fmod((mb_odometry->theta + mb_odometry->delta_theta_odo), 2.0* M_PI);
	//printf("%.5f \t %.5f \t %.5f \n", mb_odometry->x, mb_odometry->y, mb_odometry->theta);
	//if (mb_odometry->theta < 0.0){mb_odometry->theta = mb_odometry->theta + 2.0 * M_PI;}
}


float mb_clamp_radians(float angle){
    return 0;
}