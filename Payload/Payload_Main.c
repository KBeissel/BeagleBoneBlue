/**
 * @file Payload_Main.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <signal.h>
#include <math.h> // for M_PI
#include <rc/math/kalman.h>
#include <rc/math/filter.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include <rc/bmp.h>
#include <rc/mpu.h>
 // bus for Robotics Cape and BeagleboneBlue is 2
 // change this for your platform
#define I2C_BUS 2

/*******************************************************************/
// Function Declarations
/*******************************************************************/
/*For Altitude Reading*/
#define Nx 3
#define Ny 1
#define Nu 1
#define SAMPLE_RATE	200	// hz
#define	DT		(1.0/SAMPLE_RATE)
#define ACCEL_LP_TC	20*DT	// fast LP filter for accel
#define PRINT_HZ	10
#define BMP_RATE_DIV	10	// optionally sample bmp less frequently than mpu

/*For Main Program Status*/
void on_pause_press();
void on_pause_release();

/*******************************************************************/
// Variable Assingments
/*******************************************************************/
/*General*/
int StageNumber = 0; //Used for stage transition
double BaseAltitude;
/*For Altitude Reading*/
static int running = 0;
static rc_mpu_data_t mpu_data;
static rc_bmp_data_t bmp_data;
static rc_kalman_t kf = RC_KALMAN_INITIALIZER;
static rc_vector_t u = RC_VECTOR_INITIALIZER;
static rc_vector_t y = RC_VECTOR_INITIALIZER;
static rc_filter_t acc_lp = RC_FILTER_INITIALIZER;




static void __dmp_handler(void)
{
	int i;
	double accel_vec[3];
	static int bmp_sample_counter = 0;

	// make copy of acceleration reading before rotating
	for (i = 0; i < 3; i++) accel_vec[i] = mpu_data.accel[i];
	// rotate accel vector
	rc_quaternion_rotate_vector_array(accel_vec, mpu_data.dmp_quat);

	// do first-run filter setup
	if (kf.step == 0) {
		kf.x_est.d[0] = bmp_data.alt_m;
		rc_filter_prefill_inputs(&acc_lp, accel_vec[2] - 9.80665);
		rc_filter_prefill_outputs(&acc_lp, accel_vec[2] - 9.80665);
	}

	// calculate acceleration and smooth it just a tad
	rc_filter_march(&acc_lp, accel_vec[2] - 9.80665);
	u.d[0] = acc_lp.newest_output;

	// don't bother filtering Barometer, kalman will deal with that
	y.d[0] = bmp_data.alt_m;
	if (rc_kalman_update_lin(&kf, u, y)) running = 0;

	// now check if we need to sample BMP this loop
	bmp_sample_counter++;
	if (bmp_sample_counter >= BMP_RATE_DIV) {
		// perform the i2c reads to the sensor, on bad read just try later
		if (rc_bmp_read(&bmp_data)) return;
		bmp_sample_counter = 0;
	}

	return;
}
/*
int BAROMETER_SETUP(void) {
	rc_mpu_config_t mpu_conf;
	rc_matrix_t F = RC_MATRIX_INITIALIZER;
	rc_matrix_t G = RC_MATRIX_INITIALIZER;
	rc_matrix_t H = RC_MATRIX_INITIALIZER;
	rc_matrix_t Q = RC_MATRIX_INITIALIZER;
	rc_matrix_t R = RC_MATRIX_INITIALIZER;
	rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

	// allocate appropirate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);
	rc_vector_zeros(&u, Nu);
	rc_vector_zeros(&y, Ny);

	// define system -DT; // accel bias
	F.d[0][0] = 1.0;
	F.d[0][1] = DT;
	F.d[0][2] = 0.0;
	F.d[1][0] = 0.0;
	F.d[1][1] = 1.0;
	F.d[1][2] = -DT; // subtract accel bias
	F.d[2][0] = 0.0;
	F.d[2][1] = 0.0;
	F.d[2][2] = 1.0; // accel bias state

	G.d[0][0] = 0.5 * DT * DT;
	G.d[0][1] = DT;
	G.d[0][2] = 0.0;

	H.d[0][0] = 1.0;
	H.d[0][1] = 0.0;
	H.d[0][2] = 0.0;

	// covariance matrices
	Q.d[0][0] = 0.000000001;
	Q.d[1][1] = 0.000000001;
	Q.d[2][2] = 0.0001; // don't want bias to change too quickly
	R.d[0][0] = 1000000.0;

	// initial P, cloned from converged P while running
	Pi.d[0][0] = 1258.69;
	Pi.d[0][1] = 158.6114;
	Pi.d[0][2] = -9.9937;
	Pi.d[1][0] = 158.6114;
	Pi.d[1][1] = 29.9870;
	Pi.d[1][2] = -2.5191;
	Pi.d[2][0] = -9.9937;
	Pi.d[2][1] = -2.5191;
	Pi.d[2][2] = 0.3174;

	// initialize the kalman filter
	if (rc_kalman_alloc_lin(&kf, F, G, H, Q, R, Pi) == -1) return -1;
	// initialize the little LP filter to take out accel noise
	if (rc_filter_first_order_lowpass(&acc_lp, DT, ACCEL_LP_TC)) return -1;


	// init barometer and read in first data
	printf("initializing barometer\n");
	if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16)) return -1;
	if (rc_bmp_read(&bmp_data)) return -1;

	// init DMP
	printf("initializing DMP\n");
	mpu_conf = rc_mpu_default_config();
	mpu_conf.dmp_sample_rate = SAMPLE_RATE;
	mpu_conf.dmp_fetch_accel_gyro = 1;
	if (rc_mpu_initialize_dmp(&mpu_data, mpu_conf)) return -1;

	// wait for dmp to settle then start filter callback
	printf("waiting for sensors to settle");
	rc_usleep(3000000);
	rc_mpu_set_dmp_callback(__dmp_handler);
	FILE *ALTITUDE_FILE;
	ALTITUDE_FILE = fopen("ALTITUDE_FILE.txt", "w");
	// print a header
	fprintf(ALTITUDE_FILE, " altitude |");
	fprintf(ALTITUDE_FILE, "  velocity |");
	fprintf(ALTITUDE_FILE, " accel_bias |");
	fprintf(ALTITUDE_FILE, " alt (bmp) |");
	fprintf(ALTITUDE_FILE, " vert_accel |\n");
	fclose(ALTITUDE_FILE);

	return 0;
}
*/
/*
int ACCELEROMETER_SETUP(void) {
	rc_mpu_data_t data; //struct to hold new data
	FILE *Leveling_File;
	Leveling_File = fopen("Leveling_File.txt", "w");

	// use defaults for now, except also enable magnetometer.
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	//conf.show_warnings = enable_warnings;
	fprintf(Leveling_File, "Accel XYZ(G)    |");
	fprintf(Leveling_File, "Accel XYZ(raw ADC) \n");
	if (rc_mpu_initialize(&data, conf)) {
		fprintf(Leveling_File, "rc_mpu_initialize_failed\n");
		return -1;
	}
	fclose(Leveling_File);
	return 0;
}
*/

/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;
	
	/*NEED START UP DETAIL FILE*/
	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();


	//printf("\nPress and release pause button to turn green LED on and off\n");
	//printf("hold pause button down for 2 seconds to exit\n");


	/*******************************************************************/
	// BAROMETER & MPU Accelerometer Setup
	/*******************************************************************/
	//BAROMETER_SETUP();
	rc_mpu_config_t mpu_conf;
	rc_matrix_t F = RC_MATRIX_INITIALIZER;
	rc_matrix_t G = RC_MATRIX_INITIALIZER;
	rc_matrix_t H = RC_MATRIX_INITIALIZER;
	rc_matrix_t Q = RC_MATRIX_INITIALIZER;
	rc_matrix_t R = RC_MATRIX_INITIALIZER;
	rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

	// allocate appropirate memory for system
	rc_matrix_zeros(&F, Nx, Nx);
	rc_matrix_zeros(&G, Nx, Nu);
	rc_matrix_zeros(&H, Ny, Nx);
	rc_matrix_zeros(&Q, Nx, Nx);
	rc_matrix_zeros(&R, Ny, Ny);
	rc_matrix_zeros(&Pi, Nx, Nx);
	rc_vector_zeros(&u, Nu);
	rc_vector_zeros(&y, Ny);

	// define system -DT; // accel bias
	F.d[0][0] = 1.0;
	F.d[0][1] = DT;
	F.d[0][2] = 0.0;
	F.d[1][0] = 0.0;
	F.d[1][1] = 1.0;
	F.d[1][2] = -DT; // subtract accel bias
	F.d[2][0] = 0.0;
	F.d[2][1] = 0.0;
	F.d[2][2] = 1.0; // accel bias state

	G.d[0][0] = 0.5 * DT * DT;
	G.d[0][1] = DT;
	G.d[0][2] = 0.0;

	H.d[0][0] = 1.0;
	H.d[0][1] = 0.0;
	H.d[0][2] = 0.0;

	// covariance matrices
	Q.d[0][0] = 0.000000001;
	Q.d[1][1] = 0.000000001;
	Q.d[2][2] = 0.0001; // don't want bias to change too quickly
	R.d[0][0] = 1000000.0;

	// initial P, cloned from converged P while running
	Pi.d[0][0] = 1258.69;
	Pi.d[0][1] = 158.6114;
	Pi.d[0][2] = -9.9937;
	Pi.d[1][0] = 158.6114;
	Pi.d[1][1] = 29.9870;
	Pi.d[1][2] = -2.5191;
	Pi.d[2][0] = -9.9937;
	Pi.d[2][1] = -2.5191;
	Pi.d[2][2] = 0.3174;

	// initialize the kalman filter
	if (rc_kalman_alloc_lin(&kf, F, G, H, Q, R, Pi) == -1) return -1;
	// initialize the little LP filter to take out accel noise
	if (rc_filter_first_order_lowpass(&acc_lp, DT, ACCEL_LP_TC)) return -1;


	// init barometer and read in first data
	printf("initializing barometer\n");
	if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16)) return -1;
	if (rc_bmp_read(&bmp_data)) return -1;

	// init DMP
	printf("initializing DMP\n");
	mpu_conf = rc_mpu_default_config();
	mpu_conf.dmp_sample_rate = SAMPLE_RATE;
	mpu_conf.dmp_fetch_accel_gyro = 1;
	if (rc_mpu_initialize_dmp(&mpu_data, mpu_conf)) return -1;

	// wait for dmp to settle then start filter callback
	printf("waiting for sensors to settle");
	rc_usleep(3000000);
	rc_mpu_set_dmp_callback(__dmp_handler);
	FILE *ALTITUDE_FILE;
	ALTITUDE_FILE = fopen("ALTITUDE_FILE.txt", "w");
	// print a header
	fprintf(ALTITUDE_FILE, " altitude |");
	fprintf(ALTITUDE_FILE, "  velocity |");
	fprintf(ALTITUDE_FILE, " accel_bias |");
	fprintf(ALTITUDE_FILE, " alt (bmp) |");
	fprintf(ALTITUDE_FILE, " vert_accel |\n");
	fclose(ALTITUDE_FILE);
	
	



	//ACCELEROMETER_SETUP();
	//rc_mpu_data_t data; //struct to hold new data



	StageNumber = 1; //Start up stage is completed, Begin Ride Along Stage
	BaseAltitude = kf.x_est.d[0];
					 
	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		// do things based on the state
		if(rc_get_state()==RUNNING){
			rc_led_set(RC_LED_GREEN, 1);
			rc_led_set(RC_LED_RED, 0);
		}
		else{
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
		}
		
		
		//Read Bar sensor data
		ALTITUDE_FILE = fopen("ALTITUDE_FILE.txt", "a");
		fprintf(ALTITUDE_FILE, "%8.2fm |", kf.x_est.d[0]);
		fprintf(ALTITUDE_FILE, "%7.2fm/s |", kf.x_est.d[1]);
		fprintf(ALTITUDE_FILE, "%7.2fm/s^2|", kf.x_est.d[2]);
		fprintf(ALTITUDE_FILE, "%9.2fm |", bmp_data.alt_m);
		fprintf(ALTITUDE_FILE, "%7.2fm/s^2|\n", acc_lp.newest_output);
		fclose(ALTITUDE_FILE);
		/*
		// Read MPU sensor data
		FILE *Leveling_File;
		Leveling_File = fopen("Leveling_File.txt", "a");
		if (rc_mpu_read_accel(&data) < 0) {
			fprintf(Leveling_File, "read accel data failed\n");
			
		}
		else {
			fprintf(Leveling_File, "%6.2f %6.2f %6.2f |", data.accel[0] * MS2_TO_G, \
				data.accel[1] * MS2_TO_G, \
				data.accel[2] * MS2_TO_G);
			fprintf(Leveling_File, "%6d %6d %6d \n", data.raw_accel[0], \
				data.raw_accel[1], \
				data.raw_accel[2]);
		}
		fclose(Leveling_File);
		
		//Ride Along Stage
		if (StageNumber == 1) {
			if ((kf.x_est.d[0] - BaseAltitude) > 20) {
				//Rocket has launched. The rocket/payload is 20 meters above base altitude.
				if (acc_lp.newest_output < 0) {
					//Payload/Rocket is descending using Vertical Acceleration Calculation
					if (kf.x_est.d[1] < 0.01) {
						//The Payload has landed based on velocity being less then 1 cm per second.
						StageNumber == 2;
					}
				}
			}
		}
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		*/
		// always sleep at some point
		rc_usleep(100000);
	}
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	/*******************************************************************/
	/*******************************************************************/
	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	rc_mpu_power_off();
	rc_bmp_power_off();
	return 0;
}


/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
