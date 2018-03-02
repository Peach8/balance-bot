#include "mb_controller.h"
#include "mb_defs.h"

FILE *file_encoder;
FILE *file_velo;
FILE *file_angle;
FILE *file_velo_pid;
FILE *file_angle_pid;
FILE *file_pos_pid;
FILE *file_deltatheta_dr;
FILE *file_deltatheta_imu;
FILE *file_headingstep;
FILE *square;

float velsteps[5] = {-0.2, -0.1, 0.0, 0.1, 0.2};


float square_x[8] = {1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0};
float square_y[8] = {0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0};
float square_theta[8] = {0.0, -M_PI/2.0, -M_PI/2.0, -M_PI, -M_PI, -3.0*M_PI/2.0, -3.0*M_PI/2.0, 0.0};

/* POSITION CONTROL LOOP VARS */
float posx_REF_STATE = 1.0;
float posy_REF_STATE = 0.0;
float posx_fb;
float posy_fb;
float pos_err;
rc_filter_t pos_low_pass;



/* LINEAR VELO CONTROL LOOP VARS */
float linear_velo_REF_STATE = 0.0;
float linear_velo_BIAS = 0.0;
float linear_velo_fb;
float linear_velo_err;
rc_filter_t linear_velo_low_pass;

/* PITCH ANGLE CONTROL LOOP VARS */
float pitch_angle_REF_STATE;
float pitch_angle_BIAS = -0.07; //-.7,.9
float pitch_angle_fb;
float pitch_angle_err;
rc_filter_t pitch_angle_low_pass;

//-------------------------------

/* HEADING CONTROL LOOP VARS */
float heading_REF_STATE = 0.0;
float heading_fb;
float heading_err;
rc_filter_t heading_low_pass;

/* ANGULAR VELO CONTROL LOOP VARS */
float angular_velo_BIAS = 0.2;
float angular_velo_REF_STATE = 0.0;
float angular_velo_fb;
float angular_velo_err;
rc_filter_t angular_velo_low_pass;

//-------------------------------

/* MOTOR VELO VARS */
float last_valid_left = 1.0;
float last_valid_right = 1.0;
float duty_REF_STATE;
float delta_duty;
float motor_velo_BIAS = 0.1;
int step_count = 0;
float left_enc_med_filt;
float right_enc_med_filt;
float left_enc_fb;
float right_enc_fb;
float left_velo_fb;
float right_velo_fb;
float left_velo_err;
float right_velo_err;
float left_enc_low;
float right_enc_low;
int left_counter = 1;
int right_counter = 1; 
int left_n = 10;
int right_n = 10;
float *right_valueholds;
float *left_valueholds;
rc_filter_t left_low_pass;
rc_filter_t right_low_pass;

/*******************************************************************************
* int mb_initialize()
*
* this initializes all the PID controllers from the configuration file
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
int mb_initialize_controller(){
    mb_load_controller_config();
    file_velo = fopen("velo_vals.txt", "w");

    file_encoder = fopen("enc_vals.txt", "w");
    file_deltatheta_imu = fopen("imu_theta.txt", "w");
    file_deltatheta_dr = fopen("dr_theta.txt", "w");
    file_angle = fopen("angle_vals.txt", "w");
    file_headingstep = fopen("headingstep.txt", "w");
    square = fopen("square.txt", "w");


    right_valueholds = (float *) malloc(sizeof(float) * right_n);
    left_valueholds = (float *) malloc(sizeof(float) * left_n);
    int v = 0;
    for (v = 0; v < right_n; v++){
        right_valueholds[v] = 0.0;
        left_valueholds[v] = 0.0;
    }
    
    left_pid = PID_Init(
        left_pid_params.kp,
        left_pid_params.ki,
        left_pid_params.kd,
        left_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
    PID_SetIntegralLimits(left_pid, -1.0, 1.0);
    PID_SetOutputLimits(left_pid, -1.0, 1.0);
    left_low_pass = rc_empty_filter();
    rc_butterworth_lowpass(&left_low_pass, 3, 1/left_pid->dFilterHz, 2*M_PI*2.5);

    right_pid = PID_Init(
        right_pid_params.kp,
        right_pid_params.ki,
        right_pid_params.kd,
        right_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
    PID_SetIntegralLimits(right_pid, -1.0, 1.0);
    PID_SetOutputLimits(right_pid, -1.0, 1.0);
    right_low_pass = rc_empty_filter();
    rc_butterworth_lowpass(&right_low_pass, 3, 1/right_pid->dFilterHz, 2*M_PI*2.5);

    pitch_angle_pid = PID_Init(
        pitch_angle_pid_params.kp,
        pitch_angle_pid_params.ki,
        pitch_angle_pid_params.kd,
        pitch_angle_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
    PID_SetIntegralLimits(pitch_angle_pid, -1.0, 1.0);
    PID_SetOutputLimits(pitch_angle_pid, -1.0, 1.0);
    // pitch_angle_low_pass = rc_empty_filter();
    // rc_first_order_lowpass(&pitch_angle_low_pass, 1/pitch_angle_pid->dFilterHz, 0.05);

    linear_velo_pid = PID_Init(
        linear_velo_pid_params.kp,
        linear_velo_pid_params.ki,
        linear_velo_pid_params.kd,
        linear_velo_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
    PID_SetIntegralLimits(linear_velo_pid, -1.0, 1.0);
    PID_SetOutputLimits(linear_velo_pid, -M_PI/3.0, M_PI/3.0);
    // linear_velo_low_pass = rc_empty_filter();
    // rc_first_order_lowpass(&linear_velo_low_pass, 1/linear_velo_pid->dFilterHz, 3);    

    pos_pid = PID_Init(
        pos_pid_params.kp,
        pos_pid_params.ki,
        pos_pid_params.kd,
        pos_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
    PID_SetIntegralLimits(pos_pid, -1.0, 1.0);
    PID_SetOutputLimits(pos_pid, -1.0, 1.0);
    // pos_low_pass = rc_empty_filter();
    // rc_first_order_lowpass(&pos_low_pass, 1/pos_pid->dFilterHz, 3);     

    heading_pid = PID_Init(
        heading_pid_params.kp,
        heading_pid_params.ki,
        heading_pid_params.kd,
        heading_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
    PID_SetIntegralLimits(heading_pid, -1.0, 1.0);
    PID_SetOutputLimits(heading_pid, -0.5, 0.5);
    // heading_low_pass = rc_empty_filter();
    // rc_first_order_lowpass(&heading_low_pass, 1/heading_pid->dFilterHz, 3);       

    angular_velo_pid = PID_Init(
        angular_velo_pid_params.kp,
        angular_velo_pid_params.ki,
        angular_velo_pid_params.kd,
        angular_velo_pid_params.dFilterHz,
        SAMPLE_RATE_HZ
        );
    PID_SetIntegralLimits(angular_velo_pid, -1.0, 1.0);
    PID_SetOutputLimits(angular_velo_pid, -1.0, 1.0);
    // angular_velo_low_pass = rc_empty_filter();
    // rc_first_order_lowpass(&angular_velo_low_pass, 1/angular_velo_pid->dFilterHz, 3);

    return 0;
}

/*******************************************************************************
* int mb_load_controller_config()
*
* this provides a basic configuration load routine
* you can use this as is or modify it if you want a different format
*
* return 0 on success
*
*******************************************************************************/
int mb_load_controller_config(){
    FILE* file = fopen(CFG_PATH, "r");
    if (file == NULL){
        printf("Error opening pid.cfg\n");
    }

    fscanf(file, "%f %f %f %f", 
        &left_pid_params.kp,
        &left_pid_params.ki,
        &left_pid_params.kd,
        &left_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &right_pid_params.kp,
        &right_pid_params.ki,
        &right_pid_params.kd,
        &right_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &pitch_angle_pid_params.kp,
        &pitch_angle_pid_params.ki,
        &pitch_angle_pid_params.kd,
        &pitch_angle_pid_params.dFilterHz
        );    

    fscanf(file, "%f %f %f %f",
        &linear_velo_pid_params.kp,
        &linear_velo_pid_params.ki,
        &linear_velo_pid_params.kd,
        &linear_velo_pid_params.dFilterHz
        );

    fscanf(file, "%f %f %f %f",
        &angular_velo_pid_params.kp,
        &angular_velo_pid_params.ki,
        &angular_velo_pid_params.kd,
        &angular_velo_pid_params.dFilterHz
        );    

    fscanf(file, "%f %f %f %f",
        &pos_pid_params.kp,
        &pos_pid_params.ki,
        &pos_pid_params.kd,
        &pos_pid_params.dFilterHz
        );  

    fscanf(file, "%f %f %f %f",
        &heading_pid_params.kp,
        &heading_pid_params.ki,
        &heading_pid_params.kd,
        &heading_pid_params.dFilterHz
        );    

    fclose(file);
    printf("%.3f\n", angular_velo_pid_params.kp);
    return 0;
}

/*******************************************************************************
* int mb_controller_update()
* 
* TODO: Write your cascaded PID controller here
* take inputs from the global mb_state
* write outputs to the global mb_state
*
* return 0 on success
*
*******************************************************************************/
int mb_controller_update(mb_state_t* mb_state, mb_setpoints_t* mb_setpoints, mb_odometry_t* mb_odometry) {
    // update pose feedback
    posx_fb = mb_odometry->x;
    posy_fb = mb_odometry->y;
    heading_fb = mb_odometry->theta;

    // compute new delta pose
    left_enc_fb = mb_state->left_encoder;
    right_enc_fb = mb_state->right_encoder;
    float leftchecked = left_enc_fb;
    float rightchecked = right_enc_fb;
    if (fabs(leftchecked) > 70.0) {
        leftchecked = last_valid_left;
    }
    else {
        last_valid_left = leftchecked;
    }
    if (fabs(rightchecked) > 70.0) {
        rightchecked = last_valid_right;
    }
    else {
        last_valid_right = rightchecked;
    }

    left_enc_med_filt  = medianfilter(leftchecked, left_counter, left_valueholds, left_n, 1);
    right_enc_med_filt = medianfilter(rightchecked, right_counter, right_valueholds, right_n, 0);
    left_counter++;
    right_counter++;
    left_enc_low  = rc_march_filter(&left_low_pass, left_enc_med_filt);
    right_enc_low = rc_march_filter(&right_low_pass, right_enc_med_filt);
    float coeff = (M_PI * WHEEL_DIAMETER/ENCODER_RES/GEAR_RATIO) * SAMPLE_RATE_HZ;
    
    //file_encoder = fopen("enc_vals.txt", "w");

    mb_odometry->delta_pos = (((M_PI * WHEEL_DIAMETER/ENCODER_RES/GEAR_RATIO) * left_enc_low) +
                              ((M_PI * WHEEL_DIAMETER/ENCODER_RES/GEAR_RATIO) * right_enc_low)) / 2;
    mb_odometry->delta_theta_odo = (((M_PI * WHEEL_DIAMETER/ENCODER_RES/GEAR_RATIO) * right_enc_low) -
                                    ((M_PI * WHEEL_DIAMETER/ENCODER_RES/GEAR_RATIO) * left_enc_low)) / WHEEL_BASE;

    // check for manual or autonomous task control
    if (mb_setpoints->manual_ctl) {
        linear_velo_REF_STATE = -1.0 * mb_setpoints->fwd_velocity + linear_velo_BIAS;
        angular_velo_REF_STATE = mb_setpoints->turn_velocity;
    }
    else{
        // compute new pose error
        if (posx_REF_STATE > 0.0){
            pos_err = fabs(posx_REF_STATE - posx_fb);
        }
        else{
            pos_err = fabs(posx_REF_STATE - posx_fb);
        }
        //pos_err = sqrtf(pow(posx_REF_STATE - posx_fb, 2) + pow(posy_REF_STATE - posy_fb, 2));
        heading_err = heading_REF_STATE - heading_fb;

        // task state machines
        if(mb_state->task == 1){
            //printf("%.5f \t %.5f \n", heading_REF_STATE, heading_err);
            /*if(!mb_state->hasone){
                mb_state->ref_count = 0;
                posx_REF_STATE = square_x[0];
                posy_REF_STATE = square_y[0];
                heading_REF_STATE = square_theta[0];
                angular_velo_REF_STATE = 0.0;
                mb_state->hasone = 1;
            }
            linear_velo_REF_STATE = 0.3;
            if(mb_state->ref_count % 2 == 0){
                angular_velo_REF_STATE = 0.0;
                if(pos_err < 0.3){
                    printf("Hit POS goal\n");
                    mb_state->ref_count +=1;
                    posx_REF_STATE = square_x[mb_state->ref_count];
                    posy_REF_STATE = square_y[mb_state->ref_count];
                    //angular_velo_REF_STATE = 
                    heading_REF_STATE = square_theta[mb_state->ref_count];
                }
            }
            else{
                angular_velo_REF_STATE = 0.1;
                if(fabs(heading_err) < 0.6){
                    printf("Hit HEAD goal\n");
                    mb_state->ref_count +=1;
                    angular_velo_REF_STATE = 0.0;
                    posx_REF_STATE = square_x[mb_state->ref_count];
                    posy_REF_STATE = square_y[mb_state->ref_count];
                    heading_REF_STATE = square_theta[mb_state->ref_count];
                }
            }*/

            //if(mb_state->ref_count % 2 == 0){
            //    linear_velo_REF_STATE = 0.1;//PID_Compute(pos_pid, pos_err, 0);
            //}
            //else{
            //    linear_velo_REF_STATE = 0.0;
            //}
            if(step_count < 1000){heading_REF_STATE = M_PI;}
            else{heading_REF_STATE = 0.0;}




        }
        else if(mb_state->task ==2){
            if(!mb_state->hastwo){
                printf("Initalized task two\n");
                mb_state->hastwo = 1;
                posx_REF_STATE = 11.0;
                posy_REF_STATE = 0.0;
                linear_velo_REF_STATE = 0.5;
                //heading_REF_STATE = 0.0;
            }

        }
        else if(mb_state->task == 3){
            //printf("Task 3\n");
            posx_REF_STATE = 0.0;
            posy_REF_STATE = 0.0;
            //heading_REF_STATE = M_PI;
        }


        //angular_velo_REF_STATE = PID_Compute(heading_pid, heading_err, 0);
        //linear_velo_REF_STATE = PID_Compute(pos_pid, pos_err, 0);
        //printf("x %.4f y %.2f linref %.2f linfbs %.2f angref %.3f angfb %.3f\n", posx_fb, posy_fb, linear_velo_REF_STATE,linear_velo_fb, angular_velo_REF_STATE, angular_velo_fb);

    }
    //linear_velo_REF_STATE = 0.0;
    //fprintf(file_headingstep, "%.3f,%.3f\n", heading_REF_STATE, heading_fb);

    /************************/
    /* LINEAR VELO CONTROLLER */
    /************************/
    //linear_velo_REF_STATE = 0.2;
    //if(step_count < 1000){heading_REF_STATE = M_PI;}
    //else{heading_REF_STATE = 0;}
    //fprintf(file_headingstep, "%.3f,%.3f\n", heading_REF_STATE, heading_fb);

//linear_velo_REF_STATE = 0;
    linear_velo_fb = mb_odometry->delta_pos * SAMPLE_RATE_HZ;
    fprintf(file_velo, "%.3f,%.3f\n",linear_velo_REF_STATE,linear_velo_fb);
    fprintf(square,"%.3f, %.3f\n", posx_fb, posy_fb);
    linear_velo_err = linear_velo_REF_STATE - linear_velo_fb;
    pitch_angle_REF_STATE = PID_Compute(linear_velo_pid, linear_velo_err, 1) + pitch_angle_BIAS;
    

    /*************************/
    /* PITCH ANGLE CONTROLLER */
    /*************************/
    pitch_angle_fb = mb_state->alpha;
    pitch_angle_err = pitch_angle_REF_STATE - pitch_angle_fb;
    duty_REF_STATE = PID_Compute(pitch_angle_pid, pitch_angle_err, 0) + motor_velo_BIAS;


    //----------------------------------------------------


    /***************************/
    /* ANGULAR VELO CONTROLLER */
    /***************************/
    // angular_velo_fb = (mb_state->theta - mb_state->prevTheta);
    angular_velo_fb = mb_odometry->delta_theta_odo;
    float imufb = mb_state->prevTheta - mb_state->theta;

    //file_deltatheta_imu = fopen("imu_theta", "w");
    //file_deltatheta_dr

    // if(angular_velo_fb < -6.0){
    //     float newtheta = mb_state->theta + 2.0 * M_PI;
    //     angular_velo_fb = (newtheta - mb_state->prevTheta);
    // }
    // else if(angular_velo_fb > 6.0){
    //     float newthetaprev = mb_state->prevTheta + 2.0 * M_PI;
    //     angular_velo_fb = (mb_state->theta - newthetaprev);
    // }
    angular_velo_fb = angular_velo_fb * SAMPLE_RATE_HZ;
    imufb = imufb* SAMPLE_RATE_HZ;
    fprintf(file_deltatheta_dr, "%.4f,%.4f\n", angular_velo_fb, imufb);
    angular_velo_err = angular_velo_REF_STATE - angular_velo_fb;
    delta_duty = PID_Compute(angular_velo_pid, angular_velo_err, 0);// + angular_velo_BIAS;
    //printf("Ref %.3f delta_duty %.3f\n", angular_velo_REF_STATE, delta_duty);
    if (fabs(angular_velo_REF_STATE) < 0.02){ 
            delta_duty = 0.0;
    }

    mb_state->left_cmd = duty_REF_STATE - delta_duty;
    mb_state->right_cmd = duty_REF_STATE + delta_duty;




    /*if (mb_setpoints->manual_ctl) {
    printf("LinREF %.3f delta %.3f\n", linear_velo_REF_STATE, delta_duty);

        mb_state->left_cmd = duty_REF_STATE; // - delta_duty;
        mb_state->right_cmd = duty_REF_STATE; // + delta_duty;
    }
    else {
        if (fabs(angular_velo_REF_STATE) < 0.02){ 
            delta_duty = 0.0;
        }
        else {
            delta_duty = PID_Compute(angular_velo_pid, angular_velo_err, 0);// + angular_velo_BIAS;
        }

        delta_duty = 0.0;
        //duty_REF_STATE = 0.5;

        //float right_err = duty_REF_STATE - coeff * right_enc_low;
        //float left_err = duty_REF_STATE - coeff * left_enc_low;
        mb_state->right_cmd = duty_REF_STATE;
        mb_state->left_cmd  = duty_REF_STATE;
        mb_state->left_cmd -= delta_duty;
        mb_state->right_cmd += delta_duty;
        //fprintf(file_encoder, "%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n", coeff * leftchecked, coeff * left_enc_med_filt, coeff * left_enc_low, left_err, coeff * rightchecked, coeff *  right_enc_med_filt, coeff *right_enc_low, right_err, duty_REF_STATE);


        /*if(angular_velo_fb > 0.0){
            mb_state->right_cmd += delta_duty;
            mb_state->left_cmd -= delta_duty;
        }
        else{
            mb_state->right_cmd += delta_duty;
            mb_state->left_cmd -= delta_duty;

        }

        if (linear_velo_fb > 0.0) {
            mb_state->right_cmd -= delta_duty;
            mb_state->left_cmd += delta_duty;
        }
        else if (linear_velo_fb < 0.0) {
            mb_state->right_cmd += delta_duty;
            mb_state->left_cmd -= delta_duty;        
        }
    }*/
    //duty_REF_STATE = ((int)(step_count/1000)+1) * 0.1;
    step_count++;
    

    //float right_err = duty_REF_STATE - coeff * right_enc_low;
    //float left_err = duty_REF_STATE - coeff * left_enc_low;
    //mb_state->left_cmd = PID_Compute(left_pid, left_err, 0);
    //mb_state->right_cmd = PID_Compute(right_pid, right_err, 0);

    //fprintf(file_encoder, "%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n", coeff * leftchecked, coeff * left_enc_med_filt, coeff * left_enc_low, left_err, coeff * rightchecked, coeff *  right_enc_med_filt, coeff *right_enc_low, right_err, duty_REF_STATE);

    if (mb_state->right_cmd > 1.0){mb_state->right_cmd = 1.0;}
    else if(mb_state->right_cmd < -1.0){mb_state->right_cmd = -1.0;}

    if (mb_state->left_cmd > 1.0){mb_state->left_cmd = 1.0;}
    else if(mb_state->left_cmd < -1.0){mb_state->left_cmd = -1.0;}
    //printf("Left %.3f Right %.3f\n", mb_state->left_cmd, mb_state->right_cmd);
    
    //----------------------------------------------------




    // save vars to mb_state for writing to files in print thread
    mb_state->left_enc_med_filt = left_enc_med_filt;
    mb_state->left_enc_fb = left_enc_fb;
    mb_state->left_enc_low = left_enc_low;
    mb_state->motor_ref = duty_REF_STATE;

    mb_state->right_enc_med_filt = right_enc_med_filt;
    mb_state->right_enc_fb = right_enc_fb;
    mb_state->right_enc_low = right_enc_low;


    mb_state->velo_REF_STATE = linear_velo_REF_STATE;
    mb_state->velo_fb = linear_velo_fb;

    mb_state->angle_REF_STATE = pitch_angle_REF_STATE;
    mb_state->angle_fb = pitch_angle_fb;

    mb_state->velo_pterm = linear_velo_pid->pTerm;
    mb_state->velo_dterm = linear_velo_pid->dTerm;
    mb_state->velo_iterm = linear_velo_pid->iTerm;
    mb_state->velo_error = linear_velo_err;

    mb_state->angle_pterm = pitch_angle_pid->pTerm;
    mb_state->angle_dterm = pitch_angle_pid->dTerm;
    mb_state->angle_iterm = pitch_angle_pid->iTerm;

    mb_state->pos_pterm = pos_pid->pTerm;
    mb_state->pos_dterm = pos_pid->dTerm;
    mb_state->pos_iterm = pos_pid->iTerm;
    mb_state->pos_error = pos_err;


    mb_state->head_pterm = heading_pid->pTerm;
    mb_state->head_dterm = heading_pid->dTerm;
    mb_state->head_iterm = heading_pid->iTerm;
    mb_state->head_error = heading_err;
    fprintf(file_angle, "%.10f,%.10f\n", mb_state->angle_REF_STATE, mb_state->angle_fb);



    //fprintf(file_encoder, "%.10f,%.10f,%.10f,%.10f, %.10f, %.10f, %.10f\n", mb_state.left_enc_fb, mb_state.left_enc_med_filt, mb_state.left_enc_low, mb_state.right_enc_fb, mb_state.right_enc_med_filt, mb_state.right_enc_low, mb_state.motor_ref);
    //fprintf(file_encoder, "%.10f,%.10f,%.10f,%.10f, %.10f, %.10f, %.10f\n", leftchecked, left_enc_med_filt, left_enc_low, rightchecked, right_enc_med_filt, right_enc_low, duty_REF_STATE);

    return 0;
}


/*******************************************************************************
* int mb_destroy_controller()
* 
* TODO: Free all resources associated with your controller
*
* return 0 on success
*
*******************************************************************************/

int mb_destroy_controller(){
    return 0;
}

float medianfilter(float value, int counter, float *valueholds, int n, int debugflag){
    float output; 
    if(counter < n){
        output = value;
    }
    int i;
    //if(debugflag){
    //printf("%.3f %.3f %.3f %.3f %.3f \n", valueholds[0], valueholds[1], valueholds[2], valueholds[3], valueholds[4]);}
    float temparr[n];
    int j;
    for(j = 0; j<n; j++){
        temparr[j] = valueholds[j];
    }
    int k; 
    float holder;
    for (j = 0; j<n; j++){
        for (k = j+1; k<n; k++){
            if(temparr[k] < temparr[j]){
                holder = temparr[j];
                temparr[j] = temparr[k];
                temparr[k] = holder;
            }
        }
    }
    if(n%2 ==0){
        output = (temparr[n/2] + temparr[n/2-1]) /2.0;
    }
    else{
        output = temparr[n/2];
    }
    //if(debugflag){
    //printf("%.3f %.3f %.3f %.3f %.3f input %.3f, output %.3f\n", temparr[0], temparr[1], temparr[2], temparr[3], temparr[4], value, output);}
    for (i = 0; i < n-1; i++){
        valueholds[i] = valueholds[i+1];
    }
    valueholds[n-1] = value;
    return output;
}
