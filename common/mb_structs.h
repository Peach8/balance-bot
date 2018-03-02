#ifndef MB_STRUCTS_H
#define MB_STRUCTS_H

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float   alpha;             // body angle (rad) from IMU
    float   theta;             // heading (rad) from IMU
    float   prevTheta;         // previous heading
    int     left_encoder;      // left encoder counts since last reading
    int     right_encoder;     // right encoder counts since last reading

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    //TODO: Add more variables to this state as needed
    int     idle;

    float   left_enc_med_filt;
    float   left_enc_fb;
    float   left_enc_low;
    float   right_enc_med_filt;
    float   right_enc_fb;
    float   right_enc_low;
    float   motor_ref;

    float   velo_REF_STATE;
    float   velo_fb;

    float   angle_REF_STATE;
    float   angle_fb;

    float   velo_pterm;
    float   velo_iterm;
    float   velo_dterm;
    float   velo_error;

    float   angle_pterm;
    float   angle_iterm;
    float   angle_dterm;  

    float   pos_pterm;
    float   pos_iterm;
    float   pos_dterm; 
    float   pos_error;  

    float   head_pterm;
    float   head_iterm;
    float   head_dterm; 
    float   head_error;  

    int task;
    int ref_count;
    int hasone;
    int hastwo;     
};

typedef struct mb_setpoints mb_setpoints_t;
struct mb_setpoints{

    float fwd_velocity; // fwd velocity in m/s
    float turn_velocity; // turn velocity in rad/s
    int manual_ctl;


    // task waypoints
    
    // you can put these in one waypoints array if you want, but
    // the idea is that you need 8 points for each iteration of
    // the square; remember we start at (0,0,0), looking down the
    // positive x-axis; so the 8 points together look like:
    // (1,0,0) -> (1,0,pi/2) -> (1,1,pi/2) -> (1,1,pi) -> (0,1,pi)
    // -> (0,1,3pi/2) -> (0,0,3pi/2) -> (0,0,0);
    // what I haven't thought much about is how we figure out when
    // we have reached some waypoint...maybe have a threshold for the
    // pid error? there might be a more straightforward way to do it though
   // float square_x[8] = {1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
    //float square_y[8] = {0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0};
    //float square_theta[8] = {0.0, M_PI/2.0, M_PI/2.0, M_PI, M_PI, 3.0*M_PI/2.0, 3.0*M_PI/2.0, 0.0};
};

typedef struct mb_odometry mb_odometry_t;
struct mb_odometry{

    float x;        //x position from initialization in m
    float y;        //y position from initialization in m
    float theta;    //orientation from initialization in rad
    float delta_pos;
    float delta_theta_odo;
};

#endif