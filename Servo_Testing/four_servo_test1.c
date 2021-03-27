/*
This program is to test four servo on channel 1, 2, 3, 4.
First it fully extends linear actuator.
Second it fully retracts linear actuator.
Third it go to half extended linear actuator.
*/


#include <stdio.h>
#include <getopt.h>
#include <stdlib.h> // for atoi
#include <signal.h>
#include <rc/time.h>
#include <rc/adc.h>
#include <rc/dsm.h>
#include <rc/servo.h>

int main() {
    int ch1 = 1; // channel to test 0-8, 0 means all channels
    int ch2 = 2; // channel to test 0-8, 0 means all channels
    int ch3 = 3; // channel to test 0-8, 0 means all channels
    int ch4 = 4; // channel to test 0-8, 0 means all channels
    int width_us = 0; // Pulse Width in microseconds Targeted 1000-2000us, Full Range 600-2400us.

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

    rc_servo_send_pulse_us(ch1, width_us);
    rc_servo_send_pulse_us(ch2, width_us);
    rc_servo_send_pulse_us(ch3, width_us);
    rc_servo_send_pulse_us(ch4, width_us);

    printf("Signal Sent");

    // sleep for 1 sec
    rc_usleep(1000000);

    //Fully Retracted
    width_us = 1000;
    printf("Sending Servo Pulse Signal...");

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

    rc_servo_send_pulse_us(ch1, width_us);
    rc_servo_send_pulse_us(ch2, width_us);
    rc_servo_send_pulse_us(ch3, width_us);
    rc_servo_send_pulse_us(ch4, width_us);

    printf("Signal Sent");

    // sleep for 1 sec
    rc_usleep(1000000);

    // turn off power rail and cleanup
    rc_servo_power_rail_en(0);
    rc_servo_cleanup();

    return 0;


}