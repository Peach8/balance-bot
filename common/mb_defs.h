/*******************************************************************************
* mb_defs.h
*
*   defines for your bot
*   You will need to fill this in based on the data sheets, schematics, etc. 
*      and your specific configuration...
* 
*******************************************************************************/
#ifndef MB_DEFS_H
#define MB_DEFS_H

#define DEFAULT_PWM_FREQ        25000	// period of motor drive pwm?
#define RIGHT_MOTOR             1 		// id of right motor
#define LEFT_MOTOR              2 		// id of left motor
#define RIGHT_DIR               48 		// gpio1.16  P9.15
#define LEFT_DIR                60 		// gpio1.28  P9.12
#define MOT_EN                  20 		// gpio0.20  P9.41
#define MOT_1_POL               1 		// polarity of left motor
#define MOT_2_POL               1 		// polarity of right motor
#define ENC_1_POL               -1 		// polarity of left encoder
#define ENC_2_POL               1 		// polarity of right encoder
#define GEAR_RATIO              20.4	// gear ratio of motor
#define ENCODER_RES             48 		// encoder counts per motor shaft revolution
#define WHEEL_DIAMETER          0.0804 	// diameter of wheel in meters
#define WHEEL_BASE              0.194 	// wheel separation distance in meters
#define FWD_VEL_SENSITIVITY     1.5		// sensitivity of RC control for moving
#define TURN_VEL_SENSITIVITY    3.0		// sensitivity of RC control for turning
#define SAMPLE_RATE_HZ          100 	// main filter and control loop speed
#define DT                      0.01 	// 1/sample_rate
#define PRINTF_HZ               10 		// rate of print loop
#define RC_CTL_HZ               100 	// rate of RC data update

#endif
