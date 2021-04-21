/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <getopt.h>
#include <stdlib.h> // for atoi
#include <signal.h>
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/servo.h>

// function declarations
void on_pause_press();
void on_pause_release();


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


	printf("\nPress and release pause button to turn green LED on and off\n");
	printf("hold pause button down for 2 seconds to exit\n");

	int ch1 = 1; // channel to test 0-8, 0 means all channels
	int ch2 = 2; // channel to test 0-8, 0 means all channels
	int ch3 = 3; // channel to test 0-8, 0 means all channels
	int ch4 = 4; // channel to test 0-8, 0 means all channels
	int width_us = 0; // Pulse Width in microseconds Targeted 1000-2000us, Full Range 600-2400us.

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
		

		// read adc to make sure battery is connected
		if (rc_adc_init()) {
			fprintf(stderr, "ERROR: failed to run rc_adc_init()\n");
			return -1;
		}
		if (rc_adc_batt() < 6.0) {
			fprintf(stderr, "ERROR: battery disconnected or insufficiently charged to drive servos\n");
			return -1;
		}
		rc_adc_cleanup();

		// initialize PRU
		if (rc_servo_init()) return -1;

		// turn on power
		printf("Turning On 6V Servo Power Rail\n");
		rc_servo_power_rail_en(1);

		//Fully Extended
		width_us = 2000;
		printf("Sending Servo Pulse Signal...");

		//rc_servo_send_oneshot_pulse_normalized(0, 1.0);
		
		rc_servo_send_pulse_us(ch1, width_us);
		rc_servo_send_pulse_us(ch2, width_us);
		rc_servo_send_pulse_us(ch3, width_us);
		rc_servo_send_pulse_us(ch4, width_us);
		
		printf("Signal Sent");
		/*
		// sleep for 1 sec
		rc_usleep(1000000);

		//Fully Retracted
		width_us = 1000;
		printf("Sending Servo Pulse Signal...");
		rc_servo_send_oneshot_pulse_normalized(0, 0.5);
		
		rc_servo_send_pulse_us(ch1, width_us);
		rc_servo_send_pulse_us(ch2, width_us);
		rc_servo_send_pulse_us(ch3, width_us);
		rc_servo_send_pulse_us(ch4, width_us);
		
		printf("Signal Sent");

		// sleep for 1 sec
		rc_usleep(1000000);

		//Half Extended
		width_us = 1500;
		printf("Sending Servo Pulse Signal...");
		rc_servo_send_oneshot_pulse_normalized(0, 0.0);
		
		rc_servo_send_pulse_us(ch1, width_us);
		rc_servo_send_pulse_us(ch2, width_us);
		rc_servo_send_pulse_us(ch3, width_us);
		rc_servo_send_pulse_us(ch4, width_us);
		
		printf("Signal Sent");
		*/
		// sleep for 1 sec
		rc_usleep(3000000);

		// turn off power rail and cleanup
		rc_servo_power_rail_en(0);
		rc_servo_cleanup();

		
		
		
		
		
		
		
		
		
		
		
		
		
		// always sleep at some point
		rc_usleep(100000);
	}

	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
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
