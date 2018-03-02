/*******************************************************************************
* balancebot.c
*
* Main template code for the balanceBot
* 
*******************************************************************************/
#include "balancebot.h"
#include <math.h>

/* Files */
FILE *file_velo;
FILE *file_angle;
FILE *file_velo_pid;
FILE *file_angle_pid;
FILE *file_pos_pid;
FILE *file_head_pid;


/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(){
	// always initialize cape library first
	if(rc_initialize()){
		fprintf(stderr,"ERROR: failed to initialize rc_initialize(), are you root?\n");
		return -1;
	}

    file_velo = fopen("velo_vals.txt", "w");
    file_angle = fopen("angle_vals.txt", "w");
    file_velo_pid = fopen("velo_pid_vals.txt", "w");
    file_angle_pid = fopen("angle_pid_vals.txt", "w"); 
    file_pos_pid = fopen("pos_pid_vals.txt", "w");    
    file_head_pid = fopen("head_pid_vals.txt", "w");    


	//set cpu freq to max performance
	rc_set_cpu_freq(FREQ_1000MHZ);

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		printf("starting print thread... \n");
		pthread_t  printf_thread;
		pthread_create(&printf_thread, NULL, printf_loop, (void*) NULL);
	}


	mb_setpoints.manual_ctl = 0;
	mb_setpoints.fwd_velocity = 0;
	mb_setpoints.turn_velocity = 0;

	// start control thread
	printf("starting setpoint thread... \n");
	pthread_t  setpoint_control_thread;
	pthread_create(&setpoint_control_thread, NULL, setpoint_control_loop, (void*) NULL);


	// TODO: start motion capture message recieve thread

	// set up IMU configuration
	printf("initializing imu... \n");
	rc_imu_config_t imu_config = rc_default_imu_config();
	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	imu_config.orientation = ORIENTATION_Z_DOWN;
	//rc_calibrate_gyro_routine();

	if(rc_initialize_imu_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU! Exiting.\n");
		return -1;
	}

	rc_nanosleep(10E9); // wait for imu to stabilize

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_initialize_controller();

	printf("initializing motors...\n");
	mb_initialize_motors();

	printf("resetting encoders...\n");
	rc_set_encoder_pos(1, 0);
	rc_set_encoder_pos(2, 0);

	printf("initializing odometry...\n");
	mb_initialize_odometry(&mb_odometry, 0.0,0.0,0.0);

	printf("enabling motors...\n");
	mb_enable_motors();
	// testfun();
	printf("attaching imu interupt...\n");
	rc_set_imu_interrupt_func(&balancebot_controller);



	printf("we are running!!!...\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
	mb_state.task = 1;
	mb_state.ref_count = 0;
	mb_state.hasone = 0;
	mb_state.hastwo = 0;

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){

		// all the balancing is handled in the imu interupt function
		// other functions are handled in other threads
		// there is no need to do anything here but sleep

		// always sleep at some point
		usleep(100000);
	}
	
	// exit cleanly
	mb_disable_motors();
	rc_cleanup();
	 
	return 0;
}


/*******************************************************************************
* void balancebot_controller()
*
* discrete-time balance controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must implement this function to keep the balancebot balanced
* 
*
*******************************************************************************/
void balancebot_controller(){

	//lock state mutex
	pthread_mutex_lock(&state_mutex);
	// Read IMU
	mb_state.alpha = -1.0 * imu_data.dmp_TaitBryan[TB_PITCH_X];
	mb_state.theta = imu_data.dmp_TaitBryan[TB_YAW_Z];
	mb_state.theta = mb_state.theta + M_PI; 

	// Read encoders
	mb_state.left_encoder = ENC_1_POL * rc_get_encoder_pos(1);
    mb_state.right_encoder = ENC_2_POL * rc_get_encoder_pos(2);

    //printf("L %d R %d\n", rc_get_encoder_pos(1), rc_get_encoder_pos(2));

    // Update odometry 
    mb_update_odometry(&mb_odometry, &mb_state);

    // Calculate controller outputs
    mb_controller_update(&mb_state, &mb_setpoints, &mb_odometry);

    //unlock state mutex
    pthread_mutex_unlock(&state_mutex);

    // reset encoders to 0
    rc_set_encoder_pos(1, 0);
    rc_set_encoder_pos(2, 0);
    

    if (!mb_setpoints.manual_ctl) {
    	mb_set_motor(RIGHT_MOTOR, mb_state.right_cmd);
   		mb_set_motor(LEFT_MOTOR, mb_state.left_cmd);
    	// mb_set_motor(RIGHT_MOTOR, 0.0);
   		// mb_set_motor(LEFT_MOTOR, 0.0);	   		
   	}
    if (mb_setpoints.manual_ctl) {
    	mb_set_motor(RIGHT_MOTOR, mb_state.right_cmd);
   		mb_set_motor(LEFT_MOTOR, mb_state.left_cmd);
   	}
   

   	mb_state.prevTheta = mb_state.theta;
}


/*******************************************************************************
*  setpoint_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry, and Optitrak
*
*
*******************************************************************************/
void* setpoint_control_loop(void* ptr){

	// start dsm listener for radio control
	rc_initialize_dsm();

	while(1){
		if (rc_is_new_dsm_data()) {
	 		
		// 	// TODO: Handle the DSM data from the Spektrum radio reciever
		// 	// You may also implement switching between manual and autonomous mode
		// 	// using channel 5 of the DSM data.

		 	mb_setpoints.fwd_velocity = FWD_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(1);
		 	mb_setpoints.turn_velocity = TURN_VEL_SENSITIVITY * rc_get_dsm_ch_normalized(2);
			
			if(rc_get_dsm_ch_normalized(5) > 0.0){
				mb_state.task = 0;
				mb_setpoints.manual_ctl = 1;
				mb_state.hasone = 0;
				mb_state.hastwo = 0;
				//printf("MANUAL AT ONE\n");
			}
		 	else{
		 		mb_setpoints.manual_ctl = 0;
		 		if (rc_get_dsm_ch_normalized(6) > 0.5){ //down on controller, task one
		// 			//printf("SIGNAL FOR TASK ONE\n");
		 			mb_state.task = 1;
		 			mb_state.hastwo = 0;
		 		}
		 		else if (fabs(rc_get_dsm_ch_normalized(6)) < 0.5){ //down on 
		// 			//printf("SIGNAL FOR TASK TWO\n");
		 			mb_state.task = 2;
		 			mb_state.hasone = 0;
		 		}
		 		else{
		 			mb_state.task = 3;
		 		}
		}

	 	//
	 	usleep(1000000 / RC_CTL_HZ);
	}
}
}



/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("                 SENSORS               |           ODOMETRY          |");
			printf("\n");
			printf("    α    |");
			printf("    θ    |");
			printf("  L Enc  |");
			printf("  R Enc  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    θ    |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			printf("%7.3f |", mb_state.alpha);
			printf("%7.3f  |", mb_odometry.theta);
			printf("%7d  |", mb_state.left_encoder);
			printf("%7d  |", mb_state.right_encoder);
			fflush(stdout);
		}
		usleep(1000000 / PRINTF_HZ);

		// print to files
	    //fprintf(file_encoder, "%.10f,%.10f,%.10f,%.10f, %.10f, %.10f, %.10f\n", mb_state.left_enc_fb, mb_state.left_enc_med_filt, mb_state.left_enc_low, mb_state.right_enc_fb, mb_state.right_enc_med_filt, mb_state.right_enc_low, mb_state.motor_ref);
	    fprintf(file_angle, "%.10f,%.10f\n", mb_state.angle_REF_STATE, mb_state.angle_fb);
	    fprintf(file_velo_pid, "%.10f,%.10f,%.10f,%.10f\n", mb_state.velo_pterm, mb_state.velo_iterm, mb_state.velo_dterm, mb_state.velo_error);   		// print to files
	    fprintf(file_angle_pid, "%.10f,%.10f,%.10f\n", mb_state.angle_pterm, mb_state.angle_iterm, mb_state.angle_dterm);
	    fprintf(file_pos_pid, "%.10f,%.10f,%.10f, %.10f\n", mb_state.pos_pterm, mb_state.pos_iterm, mb_state.pos_dterm, mb_state.pos_error);
	    fprintf(file_head_pid, "%.10f,%.10f,%.10f, %.10f\n", mb_state.head_pterm, mb_state.head_iterm, mb_state.head_dterm, mb_state.head_error);
	}
	return NULL;
} 

int testfun(){
	FILE *f = fopen("values.txt", "w");

	//rc_nanosleep(5E9);

	mb_set_motor(LEFT_MOTOR, 0.5);
	mb_set_motor(RIGHT_MOTOR, 0.5);

	int i;
	while (!rc_get_encoder_pos(1) > 0);
	for(i = 0; i < 1000; i++){
		int oneval = ENC_1_POL * rc_get_encoder_pos(1);
    	int twoval = ENC_2_POL * rc_get_encoder_pos(2);
    	float onevel = (3.141592 * WHEEL_DIAMETER/ENCODER_RES/GEAR_RATIO) * oneval;
    	float twovel = (3.141592 * WHEEL_DIAMETER/ENCODER_RES/GEAR_RATIO) * twoval;
    	fprintf(f, "%.10f,%.10f\n",onevel, twovel);

	    // reset encoders to 0
	    rc_set_encoder_pos(1, 0);
	    rc_set_encoder_pos(2, 0); 
    	usleep(10000);
	}
	fclose(f);
	return 1;
}
