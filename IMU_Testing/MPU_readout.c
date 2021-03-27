/*
This program is to readout MPU data
Using normal mode

*/
#include <stdio.h>
#include <signal.h>
#include <getopt.h>
#include <rc/mpu.h>
#include <rc/time.h>
// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2

int main() {
	rc_mpu_data_t data; //struct to hold new data
	

	// use defaults for now, except also enable magnetometer.
	rc_mpu_config_t conf = rc_mpu_default_config();
	conf.i2c_bus = I2C_BUS;
	conf.show_warnings = enable_warnings;
	if (rc_mpu_initialize(&data, conf)) {
		fprintf(stderr, "rc_mpu_initialize_failed\n");
		return -1;
	}
	// set how many times you want to run loop
	int running = 60;

	printf("     Accel XYZ(G)    |");
	printf("  Accel XYZ(raw ADC) \n");

	//now just wait, print_data will run
	while (running>0) {
		printf("\r");
		// read sensor data
		if (rc_mpu_read_accel(&data) < 0) {
			printf("read accel data failed\n");
			break;
		}
		printf("%6.2f %6.2f %6.2f |", data.accel[0] * MS2_TO_G, \
			data.accel[1] * MS2_TO_G, \
			data.accel[2] * MS2_TO_G);
		printf("%6d %6d %6d \n", data.raw_accel[0], \
			data.raw_accel[1], \
			data.raw_accel[2]);
		running -= 1;
		rc_usleep(100000);
	}
	printf("\n");
	rc_mpu_power_off();
	return 0;
}